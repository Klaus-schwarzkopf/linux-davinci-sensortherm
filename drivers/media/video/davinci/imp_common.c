/*
* Copyright (C) 2008 Texas Instruments Inc
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <media/davinci/vpss.h>
#include <media/davinci/imp_hw_if.h>
#include <mach/cputype.h>

static int serializer_initialized;
static struct imp_serializer imp_serializer_info;
static struct imp_hw_interface *imp_hw_if;

int imp_set_preview_config(struct device *dev,
			   struct imp_logical_channel *channel,
			   struct prev_channel_config *chan_config)
{
	int ret = 0, len = 0;

	if (channel->config_state == STATE_NOT_CONFIGURED) {
		channel->config = imp_hw_if->alloc_config_block(dev);
		/* allocate buffer for holding user configuration */
		channel->user_config = imp_hw_if->alloc_user_config_block(dev,
								IMP_PREVIEWER,
								&len);
		if (ISNULL(channel->user_config)) {
			dev_err(dev,
				"memory allocate failed for user config\n");
			return -EFAULT;
		}
		channel->user_config_size = len;
	}

	if (ISNULL(chan_config->config)) {
		/* put defaults for user configuration */
		imp_hw_if->set_user_config_defaults(dev,
						    IMP_PREVIEWER,
						    channel->user_config);
		dev_dbg(dev, "imp_set_preview_config.. default.\n");
	} else {
		dev_dbg(dev, "imp_set_preview_config.. user config\n");
		memcpy(channel->user_config, chan_config->config,
		       channel->user_config_size);
	}

	/* Update the user configuration in the hw config block */
	ret = imp_hw_if->set_preview_config(dev,
					    channel->user_config,
					    channel->config);

	if (ret < 0)
		dev_err(dev, "set preview config failed\n");

	channel->config_state = STATE_CONFIGURED;
	return ret;
}
EXPORT_SYMBOL(imp_set_preview_config);

int imp_set_resizer_config(struct device *dev,
			   struct imp_logical_channel *channel,
			   struct rsz_channel_config *chan_config)
{
	int ret = 0, len;

	dev_dbg(dev, "imp_set_resizer_config. len = %d\n", chan_config->len);

	if (channel->config_state == STATE_NOT_CONFIGURED) {
		channel->config =
			imp_hw_if->alloc_config_block(dev);

		if (ISNULL(channel->config)) {
			dev_err(dev, "memory allocation failed\n");
			return -EFAULT;
		}
		/* allocate buffer for holding user configuration */
		channel->user_config = imp_hw_if->alloc_user_config_block(dev,
								IMP_RESIZER,
								&len);
		if (ISNULL(channel->user_config)) {
			dev_err(dev, "memory allocation failed\n");
			if (!chan_config->chain)
				kfree(channel->config);
			return -EFAULT;
		}
		channel->user_config_size = len;
		dev_dbg(dev, "imp_set_resizer_config, len = %d.\n", len);
	}

	if (ISNULL(chan_config->config)) {
		/* put defaults for user configuration */
		imp_hw_if->set_user_config_defaults(dev,
						    IMP_RESIZER,
						    channel->user_config);
		dev_dbg(dev, "imp_set_resizer_config, default\n");
	} else {
		memcpy(channel->user_config, chan_config->config,
			channel->user_config_size);
		dev_dbg(dev, "imp_set_resizer_config, user setting\n");
	}

	/* Update the user configuration in the hw config block or
	   if chained, copy it to the shared block and allow previewer
	   to configure it */
	ret = imp_hw_if->set_resizer_config(dev,
					    chan_config->chain,
					    channel->user_config,
					    channel->config);

	if (ret < 0)
		dev_err(dev, "set resizer config failed\n");

	channel->chained = chan_config->chain;
	channel->config_state = STATE_CONFIGURED;

	return ret;
}
EXPORT_SYMBOL(imp_set_resizer_config);

int imp_get_preview_config(struct device *dev,
			   struct imp_logical_channel *channel,
			   struct prev_channel_config *chan_config)
{
	if (channel->config_state != STATE_CONFIGURED) {
		dev_err(dev, "channel not configured\n");
		return -EINVAL;
	}

	if (ISNULL(chan_config->config)) {
		dev_err(dev, "NULL ptr\n");
		return -EINVAL;
	}

	if (copy_to_user((void *)chan_config->config,
			 (void *)channel->user_config,
			 channel->user_config_size)) {
		dev_err(dev, "Error in copy to user\n");
		return -EFAULT;
	}
	return 0;
}
EXPORT_SYMBOL(imp_get_preview_config);

int imp_get_resize_config(struct device *dev,
			  struct imp_logical_channel *channel,
			  struct rsz_channel_config *chan_config)
{
	dev_dbg(dev, "imp_get_resize_config:\n");

	if (channel->config_state != STATE_CONFIGURED) {
		dev_err(dev, "channel not configured\n");
		return -EINVAL;
	}

	if (ISNULL(chan_config->config)) {
		dev_err(dev, "NULL ptr\n");
		return -EINVAL;
	}

	if (copy_to_user((void *)chan_config->config,
			 (void *)channel->user_config,
			 channel->user_config_size)) {
		dev_err(dev, "Error in copy to user\n");
		return -EFAULT;
	}
	return 0;
}
EXPORT_SYMBOL(imp_get_resize_config);

struct prev_module_if *imp_get_module_interface(struct device *dev,
						unsigned short module_id)
{
	struct prev_module_if *module_if;
	unsigned int index = 0;
	while (1) {
		module_if = imp_hw_if->prev_enum_modules(dev, index);
		if (ISNULL(module_if))
			break;
		if (module_if->module_id == module_id)
			break;
		index++;
	}
	return module_if;
}
EXPORT_SYMBOL(imp_get_module_interface);

int imp_init_serializer(void)
{
	if (!serializer_initialized) {
		memset((void *)&imp_serializer_info, (char)0,
		       sizeof(struct imp_serializer));
		init_completion(&imp_serializer_info.sem_isr);
		imp_serializer_info.sem_isr.done = 0;
		imp_serializer_info.array_count = 0;
		mutex_init(&imp_serializer_info.array_sem);
		printk(KERN_NOTICE "imp serializer initialized\n");
		serializer_initialized = 1;
		imp_hw_if = imp_get_hw_if();
	}
	return 0;
}
EXPORT_SYMBOL(imp_init_serializer);

/* TODO: need to verify wether driver can support this */
int imp_common_reconfig_resizer(struct device *dev,
			struct rsz_reconfig *reconfig,
			struct imp_logical_channel *chan)
{
	if (chan->config_state != STATE_CONFIGURED) {
		dev_err(dev, "Configure channel first before reconfig\n");
		return -EINVAL;
	}
	if (ISNULL(imp_hw_if->reconfig_resizer)) {
		dev_err(dev, "reconfig is not supported\n");
		return -EINVAL;
	}

	return imp_hw_if->reconfig_resizer(dev, reconfig, chan->config);
}
EXPORT_SYMBOL(imp_common_reconfig_resizer);

static __init int imp_common_init(void)
{
	return 0;
}
static void imp_cleanup(void)
{
}

MODULE_LICENSE("GPL");

module_init(imp_common_init);
module_exit(imp_cleanup);
