/*******************************************************************************
  This contains the functions to handle the platform driver.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: AlphaScale
*******************************************************************************/

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include "asmmac.h"

#ifdef CONFIG_OF
static int __devinit asmmac_probe_config_dt(struct platform_device *pdev,
					    struct plat_asmmacenet_data *plat,
					    const char **mac)
{
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return -ENODEV;

	*mac = of_get_mac_address(np);
	plat->interface = of_get_phy_mode(np);
	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(struct asmmac_mdio_bus_data),
					   GFP_KERNEL);

	/*
	 * Currently only the properties needed on SPEAr600
	 * are provided. All other properties should be added
	 * once needed on other platforms.
	 */
	if (of_device_is_compatible(np, "st,spear600-gmac") ||
		of_device_is_compatible(np, "snps,dwmac-3.70a") ||
		of_device_is_compatible(np, "snps,dwmac")) {
		plat->has_gmac = 1;
		plat->pmt = 1;
	}

	return 0;
}
#else
static int __devinit asmmac_probe_config_dt(struct platform_device *pdev,
					    struct plat_asmmacenet_data *plat,
					    const char **mac)
{
	return -ENOSYS;
}
#endif /* CONFIG_OF */

/**
 * asmmac_pltfr_probe
 * @pdev: platform device pointer
 * Description: platform_device probe function. It allocates
 * the necessary resources and invokes the main to init
 * the net device, register the mdio bus etc.
 */
static int __devinit asmmac_pltfr_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
        struct resource *irqres;
	void __iomem *addr = NULL;
	struct asmmac_priv *priv = NULL;
	struct plat_asmmacenet_data *plat_dat = NULL;
	const unsigned char *mac = NULL;
        unsigned char macbuf[6];

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		pr_err("%s: ERROR: memory allocation failed"
		       "cannot get the I/O addr 0x%x\n",
		       __func__, (unsigned int)res->start);
		return -EBUSY;
	}

	addr = ioremap(res->start, resource_size(res));
	if (!addr) {
		pr_err("%s: ERROR: memory mapping failed", __func__);
		ret = -ENOMEM;
		goto out_release_region;
	}

        plat_dat = pdev->dev.platform_data;
        /* Custom initialisation (if needed)*/
	if (plat_dat->init) {
		ret = plat_dat->init(pdev);
		if (unlikely(ret))
			goto out_unmap;
	}

	priv = asmmac_dvr_probe(&(pdev->dev), plat_dat, addr);
	if (!priv) {
		pr_err("%s: main driver probe failed", __func__);
		goto out_unmap;
	}

	/* Get MAC address if available (DT) */
        macbuf[0]=0x00;
        macbuf[1]=0xe0;
        macbuf[2]=0xa3;
        macbuf[3]=0xa4;
        macbuf[4]=0x98;
        macbuf[5]=0x67;

        mac=macbuf;
	if (mac)
        {
		memcpy(priv->dev->dev_addr, mac, ETH_ALEN);
        }

	/* Get the MAC information */
        irqres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        priv->dev->irq = irqres->start;
	if (priv->dev->irq == -ENXIO) {
		pr_err("%s: ERROR: MAC IRQ configuration "
		       "information not found\n", __func__);
		ret = -ENXIO;
		goto out_unmap;
	}

	/*
	 * On some platforms e.g. SPEAr the wake up irq differs from the mac irq
	 * The external wake up irq can be passed through the platform code
	 * named as "eth_wake_irq"
	 *
	 * In case the wake up interrupt is not passed from the platform
	 * so the driver will continue to use the mac irq (ndev->irq)
	 */
	priv->wol_irq = platform_get_irq_byname(pdev, "eth_wake_irq");
	if (priv->wol_irq == -ENXIO)
		priv->wol_irq = priv->dev->irq;

	priv->lpi_irq = platform_get_irq_byname(pdev, "eth_lpi");

	platform_set_drvdata(pdev, priv->dev);

	pr_debug("ASMMAC platform driver registration completed");

	return 0;

out_unmap:
	iounmap(addr);
	platform_set_drvdata(pdev, NULL);

out_release_region:
	release_mem_region(res->start, resource_size(res));

	return ret;
}

/**
 * asmmac_pltfr_remove
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and calls the platforms hook and release the resources (e.g. mem).
 */
static int asmmac_pltfr_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct asmmac_priv *priv = netdev_priv(ndev);
	struct resource *res;
	int ret = asmmac_dvr_remove(ndev);

	if (priv->plat->exit)
		priv->plat->exit(pdev);

	platform_set_drvdata(pdev, NULL);

	iounmap((void __force __iomem *)priv->ioaddr);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	return ret;
}

#ifdef CONFIG_PM
static int asmmac_pltfr_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	return asmmac_suspend(ndev);
}

static int asmmac_pltfr_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	return asmmac_resume(ndev);
}

int asmmac_pltfr_freeze(struct device *dev)
{
	int ret;
	struct plat_asmmacenet_data *plat_dat = dev->platform_data;
	struct net_device *ndev = dev->driver_data;
	struct platform_device *pdev = to_platform_device(dev);

	ret = asmmac_freeze(ndev);
	if (plat_dat->exit)
		plat_dat->exit(pdev);

	return ret;
}

int asmmac_pltfr_restore(struct device *dev)
{
	struct plat_asmmacenet_data *plat_dat = dev->platform_data;
	struct net_device *ndev = dev->driver_data;
	struct platform_device *pdev = to_platform_device(dev);

	if (plat_dat->init)
		plat_dat->init(pdev);

	return asmmac_restore(ndev);
}

#else
static const struct dev_pm_ops asmmac_pltfr_pm_ops;
#endif /* CONFIG_PM */

static const struct of_device_id asmmac_dt_ids[] = {
	{ .compatible = "st,spear600-gmac"},
	{ .compatible = "snps,dwmac-3.70a"},
	{ .compatible = "snps,dwmac"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, asmmac_dt_ids);

struct platform_driver asmmac_pltfr_driver = {
	.probe = asmmac_pltfr_probe,
	.remove = asmmac_pltfr_remove,
	.driver = {
		   .name = "mac9260",
		   .owner = THIS_MODULE,     
                   .suspend = asmmac_pltfr_suspend,
                   .resume = asmmac_pltfr_resume,
        },   
};

MODULE_DESCRIPTION("ASMMAC 10/100/1000 Ethernet PLATFORM driver");
MODULE_AUTHOR("AlphaScale");
MODULE_LICENSE("GPL");
