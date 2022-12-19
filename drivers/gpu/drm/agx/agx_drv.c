// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2022 Tobias Heider <me@tobhe.de> */

#include <linux/module.h>
#include <linux/of_device.h>

#include <drm/drm_drv.h>

#define DRIVER_NAME     "agx"
#define DRIVER_DESC     "Apple AGX GPU driver"

struct agx_device {
	struct device *dev;
	struct platform_device *pdev;
	struct drm_device *ddev;
};

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,agx-t8103" },
	{ .compatible = "apple,agx-t8112" },
	{ .compatible = "apple,agx-t6000" },
	{ .compatible = "apple,agx-t6001" },
	{ .compatible = "apple,agx-t6002" },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static const struct drm_driver agx_drm_driver = {
	.driver_features	= DRIVER_RENDER | DRIVER_GEM | DRIVER_SYNCOBJ,
	.major			= 1,
	.minor			= 0,

#if 0
	.gem_create_object	= agx_gem_create_object,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table = panfrost_gem_prime_import_sg_table,
	.gem_prime_mmap		= drm_gem_prime_mmap,
#endif
};

static int
agx_probe(struct platform_device *pdev)
{
	struct agx_device *agxdev;
	struct drm_device *ddev;
	int err;

	agxdev = devm_kzalloc(&pdev->dev, sizeof(*agxdev), GFP_KERNEL);
	if (!agxdev)
		return -ENOMEM;

	agxdev->pdev = pdev;
	agxdev->dev = &pdev->dev;

	/* Allocate and initialize the DRM device. */
	ddev = drm_dev_alloc(&agx_drm_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	ddev->dev_private = agxdev;
	agxdev->ddev = ddev;

	/*
	 * Register the DRM device with the core and the connectors with
	 * sysfs
	 */
	err = drm_dev_register(ddev, 0);
	if (err < 0)
		return err;

	return 0;
}

static int
agx_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver agx_platform_driver = {
	.probe		= agx_probe,
	.remove		= agx_remove,
	.driver		= {
		.name	= "agx-drm",
		.of_match_table	= of_match,
	},
};
module_platform_driver(agx_platform_driver);

MODULE_AUTHOR("Tobias Heider <me@tobhe.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual MIT/GPL");
