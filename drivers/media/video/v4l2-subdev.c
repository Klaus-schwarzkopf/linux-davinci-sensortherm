/*
 * V4L2 sub-device
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *	    Sakari Ailus <sakari.ailus@maxwell.research.nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

static int subdev_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	if (!sd->initialized)
		return -EAGAIN;

	return 0;
}

static int subdev_close(struct file *file)
{
	return 0;
}

static long subdev_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	switch (cmd) {
	case VIDIOC_QUERYCTRL:
		return v4l2_subdev_queryctrl(sd, arg);

	case VIDIOC_QUERYMENU:
		return v4l2_subdev_querymenu(sd, arg);

	case VIDIOC_G_CTRL:
		return v4l2_subdev_g_ctrl(sd, arg);

	case VIDIOC_S_CTRL:
		return v4l2_subdev_s_ctrl(sd, arg);

	case VIDIOC_G_EXT_CTRLS:
		return v4l2_subdev_g_ext_ctrls(sd, arg);

	case VIDIOC_S_EXT_CTRLS:
		return v4l2_subdev_s_ext_ctrls(sd, arg);

	case VIDIOC_TRY_EXT_CTRLS:
		return v4l2_subdev_try_ext_ctrls(sd, arg);

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static long subdev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return __video_usercopy(file, cmd, arg, subdev_do_ioctl);
}

const struct v4l2_file_operations v4l2_subdev_fops = {
	.owner = THIS_MODULE,
	.open = subdev_open,
	.unlocked_ioctl = subdev_ioctl,
	.release = subdev_close,
};

void v4l2_subdev_init(struct v4l2_subdev *sd, const struct v4l2_subdev_ops *ops)
{
	INIT_LIST_HEAD(&sd->list);
	BUG_ON(!ops);
	sd->ops = ops;
	sd->v4l2_dev = NULL;
	sd->flags = 0;
	sd->name[0] = '\0';
	sd->grp_id = 0;
	sd->dev_priv = NULL;
	sd->host_priv = NULL;
	sd->initialized = 1;
}
EXPORT_SYMBOL(v4l2_subdev_init);
