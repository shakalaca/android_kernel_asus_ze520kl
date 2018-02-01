/* Copyright (c) 2011, 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

/*ASUS_BSP Freddy:
* msm_show_resume_irq_mask manual debug node is "/sys/module/msm_show_resume_irq/parameters/debug_mask"
*
* irq-gic-v3.c will include "irq-gic-common.h", which extern msm_show_resume_irq_mask.
* pinctrl-msm.c  extern msm_show_resume_irq_mask.
*
* If "debug_mask" or "msm_show_resume_irq_mask" is True, "gic_show_resume_irq" and "msm_show_resume_irq_mask" will dump more info.
*/
/*[PM] Disable mask  for print IRQ triggered  during gic_show_resume_irq() in kernel/drivers/irqchip/irq-gic.c */

int msm_show_resume_irq_mask=1;

module_param_named(
	debug_mask, msm_show_resume_irq_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);
