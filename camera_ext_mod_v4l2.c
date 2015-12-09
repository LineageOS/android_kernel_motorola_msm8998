/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <media/v4l2-ioctl.h>
#include "camera_ext.h"

struct camera_ext_v4l2 {
	struct regulator *cdsi_reg;
};

struct camera_ext_v4l2 *g_v4l2_data;

static int input_enum(struct file *file, void *fh, struct v4l2_input *inp)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_input_enum(gb_dev, inp);
}

static int input_get(struct file *file, void *fh, unsigned int *i)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_input_get(gb_dev, i);
}

static int input_set(struct file *file, void *fh,
			unsigned int i)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_input_set(gb_dev, i);
}

static int fmt_enum(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_format_enum(gb_dev, fmt);
}

static int fmt_get(struct file *file, void *fh,
			struct v4l2_format *fmt)
{
	struct device *gb_dev;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("%s: unsupport buffer type %d\n", __func__, fmt->type);
		return -EINVAL;
	}

	gb_dev = video_drvdata(file);
	return gb_camera_ext_format_get(gb_dev, fmt);
}

static int fmt_set(struct file *file, void *fh,
			struct v4l2_format *fmt)
{
	struct device *gb_dev;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("%s: unsupport buffer type %d\n", __func__, fmt->type);
		return -EINVAL;
	}

	gb_dev = video_drvdata(file);
	return gb_camera_ext_format_set(gb_dev, fmt);
}

static int frmsize_enum(struct file *file, void *fh,
			struct v4l2_frmsizeenum *frmsize)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_frmsize_enum(gb_dev, frmsize);
}

static int frmival_enum(struct file *file, void *fh,
			struct v4l2_frmivalenum *frmival)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_frmival_enum(gb_dev, frmival);
}

static int stream_on(struct file *file, void *fh,
			enum v4l2_buf_type buf_type)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_stream_on(gb_dev);
}

static int stream_off(struct file *file, void *fh,
			enum v4l2_buf_type buf_type)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_stream_off(gb_dev);
}

static int stream_parm_get(struct file *file, void *fh,
			struct v4l2_streamparm *parm)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_stream_parm_get(gb_dev, parm);
}

static int stream_parm_set(struct file *file, void *fh,
			struct v4l2_streamparm *parm)
{
	struct device *gb_dev = video_drvdata(file);

	return gb_camera_ext_stream_parm_set(gb_dev, parm);
}

/* This device is used to query mod capabilities and config mod stream.
 * It does not support video buffer related operations.
 */
static const struct v4l2_ioctl_ops camera_ext_v4l2_ioctl_ops = {
	.vidioc_enum_input		= input_enum,
	.vidioc_g_input			= input_get,
	.vidioc_s_input			= input_set,
	.vidioc_enum_fmt_vid_cap	= fmt_enum,
	.vidioc_g_fmt_vid_cap		= fmt_get,
	.vidioc_s_fmt_vid_cap		= fmt_set,
	.vidioc_enum_framesizes		= frmsize_enum,
	.vidioc_enum_frameintervals	= frmival_enum,
	.vidioc_streamon		= stream_on,
	.vidioc_streamoff		= stream_off,
	.vidioc_g_parm			= stream_parm_get,
	.vidioc_s_parm			= stream_parm_set,
};

static int mod_v4l2_reg_control(bool on)
{
	int rc;

	/* NO-OP if there is no platform device probed */
	if (!g_v4l2_data)
		return 0;

	if (on)
		rc = regulator_enable(g_v4l2_data->cdsi_reg);
	else
		rc = regulator_disable(g_v4l2_data->cdsi_reg);

	return rc;
}


static int mod_v4l2_open(struct file *file)
{
	int rc;
	struct device *gb_dev = video_drvdata(file);

	rc = mod_v4l2_reg_control(true);
	if (rc < 0)
		return rc;

	rc = gb_camera_ext_power_on(gb_dev);
	if (rc < 0)
		mod_v4l2_reg_control(false);

	return rc;
}

static int mod_v4l2_close(struct file *file)
{
	int ret;
	struct device *gb_dev = video_drvdata(file);

	ret = gb_camera_ext_power_off(gb_dev);

	mod_v4l2_reg_control(false);

	return ret;
}

static struct v4l2_file_operations camera_ext_mod_v4l2_fops = {
	.owner	 = THIS_MODULE,
	.open	 = mod_v4l2_open,
	.ioctl	 = video_ioctl2,
	.release = mod_v4l2_close,
};

int camera_ext_mod_v4l2_init(struct camera_ext *cam_dev, struct device *gb_dev)
{
	int retval;

	snprintf(cam_dev->v4l2_dev.name, sizeof(cam_dev->v4l2_dev.name),
				"%s", CAMERA_EXT_DEV_NAME);
	retval = v4l2_device_register(NULL, &cam_dev->v4l2_dev);
	if (retval) {
		pr_err("failed to register v4l2 device\n");
		return -ENODEV;
	}

	cam_dev->vdev_mod.v4l2_dev = &cam_dev->v4l2_dev;
	cam_dev->vdev_mod.release = video_device_release;
	cam_dev->vdev_mod.fops = &camera_ext_mod_v4l2_fops;
	cam_dev->vdev_mod.ioctl_ops = &camera_ext_v4l2_ioctl_ops;
	cam_dev->vdev_mod.vfl_type = VFL_TYPE_GRABBER;

	retval = video_register_device(&cam_dev->vdev_mod,
				VFL_TYPE_GRABBER, -1);
	if (retval) {
		pr_err("%s: failed to register video device. rc %d\n",
			__func__, retval);
		goto error_reg_vdev;
	}

	video_set_drvdata(&cam_dev->vdev_mod, gb_dev);
	return retval;

error_reg_vdev:
	v4l2_device_unregister(&cam_dev->v4l2_dev);
	return retval;
}

void camera_ext_mod_v4l2_exit(struct camera_ext *cam_dev)
{
	video_unregister_device(&cam_dev->vdev_mod);
	v4l2_device_unregister(&cam_dev->v4l2_dev);
}

static int camera_ext_v4l2_probe(struct platform_device *pdev)
{
	struct camera_ext_v4l2 *data;

	if (!pdev->dev.of_node) {
		/* Platform data not currently supported */
		dev_err(&pdev->dev, "%s: of devtree not found\n", __func__);
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cdsi_reg = devm_regulator_get(&pdev->dev, "camera_ext_cdsi");
	if (IS_ERR(data->cdsi_reg))  {
		dev_err(&pdev->dev, "%s: failed to get cdsi regulator.\n",
			__func__);
		return PTR_ERR(data->cdsi_reg);
	}

	g_v4l2_data = data;

	return 0;
}

static int camera_ext_v4l2_remove(struct platform_device *pdev)
{
	g_v4l2_data = NULL;

	return 0;
}

static const struct of_device_id camera_ext_match[] = {
	{.compatible = "mmi,mods-camera-ext",},
	{},
};

static const struct platform_device_id camera_ext_id_table[] = {
	{"camera_ext", 0},
	{},
};

static struct platform_driver camera_ext_v4l2_driver = {
	.driver = {
		.name = "camera_ext_v4l2",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(camera_ext_match),
	},
	.probe = camera_ext_v4l2_probe,
	.remove = camera_ext_v4l2_remove,
	.id_table = camera_ext_id_table,
};

int camera_ext_v4l2_driver_init(void)
{
	return platform_driver_register(&camera_ext_v4l2_driver);
}

void camera_ext_v4l2_driver_exit(void)
{
	platform_driver_unregister(&camera_ext_v4l2_driver);
}
