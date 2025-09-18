// SPDX-License-Identifier: (GPL-2.0+ OR MIT)

/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#ifndef __AML_GPIO_H__
#define __AML_GPIO_H__

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/gpio/driver.h>

enum gpiod_pull_type {
	GPIOD_PULL_DIS = 0,
	GPIOD_PULL_DOWN = 1,
	GPIOD_PULL_UP = 2,
};

#ifdef CONFIG_GPIOLIB
int gpiod_set_pull(struct gpio_desc *desc, unsigned int value);
#else /* CONFIG_GPIOLIB */
static inline int gpiod_set_pull(struct gpio_desc *desc, unsigned int value)
{
	/* GPIO can never have been requested */
	WARN_ON(1);
	return -EINVAL;
}
#endif /* CONFIG_GPIOLIB */

#if IS_ENABLED(CONFIG_AMLOGIC_GPIOLIB_SYSFS)
int gpiochip_sysfs_register(struct gpio_device *gdev);
#else
static inline int gpiochip_sysfs_register(struct gpio_device *gdev)
{
	return 0;
}
#endif

#endif //__AML_GPIO_H__
