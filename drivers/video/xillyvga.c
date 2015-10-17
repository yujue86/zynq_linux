/*
 * Xillyvga frame buffer driver
 *
 * Author: Xillybus Ltd.
 *
 * 2012 (c) Xillybus Ltd.
 * 2002-2007 (c) MontaVista Software, Inc.
 * 2007 (c) Secret Lab Technologies, Ltd.
 * 2009 (c) Xilinx Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/slab.h>


#define DRIVER_NAME		"xillyvga"

#define fpga_vga_ctrl_reg 0x0002
#define fpga_reads_per_frame_reg 0x0003
#define fpga_buf_addr_reg 0x0004

/*
 * The hardware only handles a single mode: 1024x768 24 bit true
 * color. Each pixel gets a word (32 bits) of memory.  Within each word,
 * the 8 most significant bits are ignored, the next 8 bits are the red
 * level, the next 8 bits are the green level and the 8 least
 * significant bits are the blue level.
 */

#define BYTES_PER_PIXEL	4
#define BITS_PER_PIXEL	(BYTES_PER_PIXEL * 8)

#define RED_SHIFT	16
#define GREEN_SHIFT	8
#define BLUE_SHIFT	0

#define PALETTE_ENTRIES_NO	16	/* passed to fb_alloc_cmap() */

struct xillyvga_platform_data {
	u32 xres, yres;		/* resolution of screen in pixels */
	u32 xvirt, yvirt;	/* resolution of memory buffer */
};

/*
 * Default xillyvga configuration
 */
static struct xillyvga_platform_data xillyvga_fb_default_pdata = {
	.xres = 1024,
	.yres = 768,
	.xvirt = 1024,
	.yvirt = 768,
};

/*
 * Here are the default fb_fix_screeninfo and fb_var_screeninfo structures
 */
static struct fb_fix_screeninfo xillyvga_fb_fix = {
	.id =		"Xilinx",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.accel =	FB_ACCEL_NONE
};

static struct fb_var_screeninfo xillyvga_fb_var = {
	.bits_per_pixel =	BITS_PER_PIXEL,

	.red =		{ RED_SHIFT, 8, 0 },
	.green =	{ GREEN_SHIFT, 8, 0 },
	.blue =		{ BLUE_SHIFT, 8, 0 },
	.transp =	{ 0, 0, 0 },

	.activate =	FB_ACTIVATE_NOW
};


struct xillyvga_drvdata {

	struct fb_info	info;		/* FB driver info record */

	phys_addr_t	regs_phys;	/* phys. address of the control
						registers */
	u32 __iomem	*registers;	/* virt. address of the control
						registers */
	void		*fb_virt;	/* virt. address of the frame buffer */
	dma_addr_t	fb_phys;	/* phys. address of the frame buffer */

	u8 		flags;		/* features of the driver */

	u32		reg_ctrl_default;

	u32		pseudo_palette[PALETTE_ENTRIES_NO];
					/* Fake palette of 16 colors */
	void *afi;  /* Pointer to AFI control registers. To be removed. */
};

#define to_xillyvga_drvdata(_info) \
	container_of(_info, struct xillyvga_drvdata, info)

static int
xillyvga_fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
	unsigned transp, struct fb_info *fbi)
{
	u32 *palette = fbi->pseudo_palette;

	if (regno >= PALETTE_ENTRIES_NO)
		return -EINVAL;

	if (fbi->var.grayscale) {
		/* Convert color to grayscale.
		 * grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
			(red * 77 + green * 151 + blue * 28 + 127) >> 8;
	}

	/* fbi->fix.visual is always FB_VISUAL_TRUECOLOR */

	/* We only handle 8 bits of each color. */
	red >>= 8;
	green >>= 8;
	blue >>= 8;
	palette[regno] = (red << RED_SHIFT) | (green << GREEN_SHIFT) |
			 (blue << BLUE_SHIFT);

	return 0;
}

static int
xillyvga_fb_blank(int blank_mode, struct fb_info *fbi)
{
	struct xillyvga_drvdata *drvdata = to_xillyvga_drvdata(fbi);

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		/* Turn on the display. */
		iowrite32(1, &drvdata->registers[fpga_vga_ctrl_reg]);

		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:

		/* Turn off the display. Continues useless memory reading */
		iowrite32(5, &drvdata->registers[fpga_vga_ctrl_reg]);

	default:
		break;

	}
	return 0; /* success */
}

static struct fb_ops xillyvga_ops =
{
	.owner			= THIS_MODULE,
	.fb_setcolreg		= xillyvga_fb_setcolreg,
	.fb_blank		= xillyvga_fb_blank,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
};

/* ---------------------------------------------------------------------
 * Bus independent setup/teardown
 */

#define AFIBASE 0xF8008000

#define AFI0BASE 0x00000000
#define AFI1BASE 0x00001000
#define AFI2BASE 0x00002000
#define AFI3BASE 0x00003000

/* Offsets */
#define AFI_RDCHAN_CTRL 0x00
#define AFI_WRCHAN_CTRL 0x14

static int xillyvga_assign(struct device *dev,
			   struct xillyvga_drvdata *drvdata,
			   unsigned long physaddr,
			   struct xillyvga_platform_data *pdata)
{
	int rc;
	int fbsize = pdata->xvirt * pdata->yvirt * BYTES_PER_PIXEL;

	if (!request_mem_region(physaddr, 128, DRIVER_NAME)) {
		dev_err(dev, "Couldn't request memory region at 0x%08lX\n",
			physaddr);
		rc = -ENODEV;
		goto err_region;
	}

	drvdata->regs_phys = physaddr;
	drvdata->registers = ioremap(physaddr, 128);

	if (!drvdata->registers) {
		dev_err(dev, "Couldn't ioremap memory region at 0x%08lX\n",
			physaddr);
		rc = -ENODEV;
		goto err_map;
	}

	/*
	 * UGLY HACK ALERT:
	 * The HP2 AXI3 bus interface is used in 32-bit mode, which must be
	 * set in the relevant registers. This may have been done OK by
	 * FSBL, but let's not trust that. May not work if a driver has
	 * taken over this memory region (which doesn't seems to be planned)
	 */

	if  (!request_mem_region(AFIBASE + AFI2BASE, 0x1000, DRIVER_NAME)) {
		printk(KERN_ERR DRIVER_NAME ": AFI request_mem_region failed. Aborting.\n");
		rc = -EBUSY;
		goto failed_request_afi;
	}

	drvdata->afi = ioremap(AFIBASE + AFI2BASE, 0x1000);

	if (!drvdata->afi) {
		printk(KERN_ERR DRIVER_NAME ": Failed to map AFI I/O memory. Aborting.\n");
		rc = -EBUSY;
		goto failed_iomap_afi;
	}

	iowrite32(ioread32(drvdata->afi + AFI_RDCHAN_CTRL) | 1,
		  drvdata->afi + AFI_RDCHAN_CTRL);
	iowrite32(ioread32(drvdata->afi + AFI_WRCHAN_CTRL) | 1,
		  drvdata->afi + AFI_WRCHAN_CTRL);


	/* Allocate the framebuffer memory */
	drvdata->fb_virt = dma_alloc_coherent(dev, PAGE_ALIGN(fbsize),
					      &drvdata->fb_phys, GFP_KERNEL);

	if (!drvdata->fb_virt) {
		dev_err(dev, "Could not allocate frame buffer memory\n");
		rc = -ENOMEM;
		goto err_fbmem;
	}

	/*
	 * There is a general understanding that DMA allocations are
	 * page aligned. If the returned address isn't 64-byte aligned,
	 * the FPGA will not work properly, so fail here and now. If this
	 * ever happens, the allocation will have to be done with
	 * __get_free_pages().
	 */

	if ( ((u32) drvdata->fb_virt) & 0x3f) {
		dev_err(dev, "Frame buffer memory badly aligned\n");
		rc = -ENOMEM;
		goto err_noalign;
	}

	/* Clear (turn to black) the framebuffer */
	memset_io((void __iomem *)drvdata->fb_virt, 0, fbsize);

	/* Tell the hardware where the frame buffer is */
	iowrite32(drvdata->fb_phys, &drvdata->registers[fpga_buf_addr_reg]);

	/*
	 * Tell the hardware what the buffer's size is, in 16 pixel chunks.
	 * This requires Height x Width to be a multiple of 16, which is
	 * rarely an issue.
	 */

	iowrite32(pdata->xvirt * pdata->yvirt / 16,
		  &drvdata->registers[fpga_reads_per_frame_reg]);

	/* Turn on the display */
	iowrite32(1, &drvdata->registers[fpga_vga_ctrl_reg]);

	/* Fill struct fb_info */
	drvdata->info.device = dev;
	drvdata->info.screen_base = (void __iomem *)drvdata->fb_virt;
	drvdata->info.fbops = &xillyvga_ops;
	drvdata->info.fix = xillyvga_fb_fix;
	drvdata->info.fix.smem_start = drvdata->fb_phys;
	drvdata->info.fix.smem_len = fbsize;
	drvdata->info.fix.line_length = pdata->xvirt * BYTES_PER_PIXEL;

	drvdata->info.pseudo_palette = drvdata->pseudo_palette;
	drvdata->info.flags = FBINFO_DEFAULT;
	drvdata->info.var = xillyvga_fb_var;
	drvdata->info.var.xres = pdata->xres;
	drvdata->info.var.yres = pdata->yres;
	drvdata->info.var.xres_virtual = pdata->xvirt;
	drvdata->info.var.yres_virtual = pdata->yvirt;

	/* Allocate a colour map */
	rc = fb_alloc_cmap(&drvdata->info.cmap, PALETTE_ENTRIES_NO, 0);
	if (rc) {
		dev_err(dev, "Fail to allocate colormap (%d entries)\n",
			PALETTE_ENTRIES_NO);
		goto err_cmap;
	}

	/* Register new frame buffer */
	rc = register_framebuffer(&drvdata->info);
	if (rc) {
		dev_err(dev, "Could not register frame buffer\n");
		goto err_regfb;
	}

	/* Put a banner in the log (for DEBUG) */
	dev_dbg(dev, "regs: phys=%lx, virt=%p\n", physaddr,
		drvdata->registers);

	/* Put a banner in the log (for DEBUG) */
	dev_dbg(dev, "fb: phys=%llx, virt=%p, size=%x\n",
		(unsigned long long)drvdata->fb_phys, drvdata->fb_virt, fbsize);

	return 0;	/* success */

err_regfb:
	fb_dealloc_cmap(&drvdata->info.cmap);

err_cmap:
	/* Turn off the display */
	iowrite32(0, &drvdata->registers[fpga_vga_ctrl_reg]);

err_noalign:
	dma_free_coherent(dev, PAGE_ALIGN(fbsize), drvdata->fb_virt,
			  drvdata->fb_phys);

err_fbmem:
	iounmap(drvdata->afi);

failed_iomap_afi:
	release_mem_region(AFIBASE + AFI0BASE, 0x1000);

failed_request_afi:
	iounmap(drvdata->registers);

err_map:
	release_mem_region(physaddr, 128);

err_region:
	kfree(drvdata);
	dev_set_drvdata(dev, NULL);

	return rc;
}

static int xillyvga_release(struct device *dev)
{
	struct xillyvga_drvdata *drvdata = dev_get_drvdata(dev);

#if !defined(CONFIG_FRAMEBUFFER_CONSOLE) && defined(CONFIG_LOGO)
	xillyvga_fb_blank(VESA_POWERDOWN, &drvdata->info);
#endif

	/* Turn off the display */
	iowrite32(0, &drvdata->registers[fpga_vga_ctrl_reg]);

	unregister_framebuffer(&drvdata->info);

	fb_dealloc_cmap(&drvdata->info.cmap);

	dma_free_coherent(dev, PAGE_ALIGN(drvdata->info.fix.smem_len),
			  drvdata->fb_virt, drvdata->fb_phys);

	iounmap(drvdata->afi);
	release_mem_region(AFIBASE + AFI0BASE, 0x1000);

	iounmap(drvdata->registers);
	release_mem_region(drvdata->regs_phys, 128);

	kfree(drvdata);
	dev_set_drvdata(dev, NULL);

	return 0;
}

/* ---------------------------------------------------------------------
 * OF bus binding
 */

static int xillyvga_of_probe(struct platform_device *op)
{
	struct xillyvga_platform_data pdata;
	struct resource res;
	int rc;
	struct xillyvga_drvdata *drvdata;

	/* Copy with the default pdata (not a ptr reference!) */
	pdata = xillyvga_fb_default_pdata;

	/* Allocate the driver data region */
	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&op->dev, "Couldn't allocate device private record\n");
		return -ENOMEM;
	}

	rc = of_address_to_resource(op->dev.of_node, 0, &res);
	if (rc) {
		dev_err(&op->dev, "invalid address\n");
		goto err;
	}

	dev_set_drvdata(&op->dev, drvdata);

	return xillyvga_assign(&op->dev, drvdata, res.start, &pdata);

 err:
	kfree(drvdata);
	return -ENODEV;
}

static int xillyvga_of_remove(struct platform_device *op)
{
	return xillyvga_release(&op->dev);
}

/* Match table for of_platform binding */
static struct of_device_id xillyvga_of_match[]  = {
	{ .compatible = "xillybus,xillyvga-1.00.a", },
	{ .compatible = "xlnx,xillyvga-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, xillyvga_of_match);

static struct platform_driver xillyvga_of_driver = {
	.probe = xillyvga_of_probe,
	.remove = xillyvga_of_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xillyvga_of_match,
	},
};

module_platform_driver(xillyvga_of_driver);

MODULE_AUTHOR("Eli Billauer, Xillybus Ltd.");
MODULE_DESCRIPTION("Xillyvga frame buffer driver");
MODULE_LICENSE("GPL");
