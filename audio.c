/*
 * Greybus audio driver
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <sound/simple_card.h>

#include "greybus.h"
#include "audio.h"


#define GB_AUDIO_MGMT_DRIVER_NAME		"gb_audio_mgmt"
#define GB_MODS_AUDIO_DRIVER_NAME		"gb_mods_audio"

static struct gb_snd_codec	snd_codec;
/*
 * XXX this is sort of cruddy but I get warnings if
 * we don't have dev.release handler set.
 */
static void default_release(struct device *dev)
{
}

static void gb_mods_audio_release(struct kref *kref)
{
	struct gb_snd_codec *codec =
		container_of(kref, struct gb_snd_codec, mods_aud_kref);

	pr_debug("%s\n", __func__);

	kfree(codec->vol_range);
	codec->vol_range = NULL;
	kfree(codec->use_cases);
	codec->use_cases = NULL;
	kfree(codec->aud_devices);
	codec->aud_devices = NULL;
	kfree(codec->spkr_preset);
	codec->spkr_preset = NULL;
	codec->mods_aud_connection = NULL;
}

static void gb_mods_i2s_release(struct kref *kref)
{
	struct gb_snd_codec *codec =
		container_of(kref, struct gb_snd_codec, mods_i2s_kref);

	pr_debug("%s\n", __func__);
	gb_i2s_mgmt_free_cfgs(codec);
	codec->mgmt_connection = NULL;
}

void gb_mods_audio_get(struct gb_snd_codec *codec)
{
	pr_debug("%s\n", __func__);
	gb_connection_get(codec->mods_aud_connection);
	kref_get(&codec->mods_aud_kref);
}

void gb_mods_audio_put(struct gb_snd_codec *codec)
{
	struct gb_connection *conn = codec->mods_aud_connection;

	pr_debug("%s\n", __func__);
	kref_put(&codec->mods_aud_kref, gb_mods_audio_release);
	gb_connection_put(conn);
}

void gb_mods_i2s_put(struct gb_snd_codec *codec)
{
	struct gb_connection *conn = codec->mgmt_connection;

	pr_debug("%s\n", __func__);
	kref_put(&codec->mods_i2s_kref, gb_mods_i2s_release);
	gb_connection_put(conn);
}

void gb_mods_i2s_get(struct gb_snd_codec *codec)
{
	pr_debug("%s\n", __func__);
	gb_connection_get(codec->mgmt_connection);
	kref_get(&codec->mods_i2s_kref);
}

static int gb_i2s_mgmt_connection_init(struct gb_connection *connection)
{
	int ret;

	mutex_lock(&snd_codec.lock);
	snd_codec.mgmt_connection = connection;
	connection->private = &snd_codec;

	ret = gb_i2s_mgmt_get_cfgs(&snd_codec, connection);
	if (ret) {
		pr_err("can't get i2s configurations: %d\n", ret);
		goto err;
	}

	kref_init(&snd_codec.mods_i2s_kref);
	gb_connection_get(snd_codec.mgmt_connection);
	/* report audio devices available to user space now
	 * if mods audio connection is also initialized
	*/
	if (snd_codec.report_devices && snd_codec.mods_aud_connection)
		snd_codec.report_devices(&snd_codec);
	mutex_unlock(&snd_codec.lock);
	return 0;

err:
	snd_codec.mgmt_connection = NULL;
	mutex_unlock(&snd_codec.lock);
	return ret;
}

static int gb_mods_audio_connection_init(struct gb_connection *connection)
{
	struct gb_audio_get_volume_db_range_response *get_vol;
	struct gb_audio_get_supported_usecases_response *get_use_cases;
	struct gb_audio_get_devices_response *get_devices;
	struct gb_audio_get_speaker_preset_eq_response *get_preset;
	int ret;
	int mods_vol_step;
	int mods_vol_range_step;

	mutex_lock(&snd_codec.lock);
	snd_codec.mods_aud_connection = connection;

	get_vol = kmalloc(sizeof(*get_vol), GFP_KERNEL);
	if (!get_vol) {
		ret = -ENOMEM;
		goto out;
	}
	ret = gb_mods_aud_get_vol_range(get_vol, connection);
	if (ret) {
		dev_err(&connection->bundle->dev, "failed to get aud dev vol range: %d\n",
				ret);
		goto free_get_vol;
	}
	snd_codec.vol_range = get_vol;

	get_use_cases = kmalloc(sizeof(*get_use_cases), GFP_KERNEL);
	if (!get_use_cases) {
		ret = -ENOMEM;
		goto set_vol_null;
	}
	ret = gb_mods_aud_get_supported_usecase(get_use_cases,
						  connection);
	if (ret) {
		dev_err(&connection->bundle->dev, "failed to get aud dev supp usecases %d\n",
				ret);
		goto free_use_case;
	}
	snd_codec.use_cases = get_use_cases;

	get_devices = kmalloc(sizeof(*get_devices), GFP_KERNEL);
	if (!get_devices) {
		ret = -ENOMEM;
		goto get_dev_null;
	}
	ret = gb_mods_aud_get_devices(get_devices,
						  connection);
	if (ret) {
		dev_err(&connection->bundle->dev, "failed to get aud devices %d\n",
			ret);
		goto free_aud_dev;
	}
	snd_codec.aud_devices = get_devices;

	/* if speaker device is supported query EQ preset needed by mod */
	if ((le32_to_cpu(get_devices->devices.out_devices) &
			GB_AUDIO_DEVICE_OUT_LOUDSPEAKER) &&
		gb_i2s_audio_is_ver_supported(snd_codec.mods_aud_connection,
				GB_MODS_AUDIO_VERSION_SPKR_PRESET_MAJOR,
				GB_MODS_AUDIO_VERSION_SPKR_PRESET_MINOR)) {
		get_preset = kmalloc(sizeof(*get_preset), GFP_KERNEL);
		if (!get_preset) {
			ret = -ENOMEM;
			goto free_aud_dev;
		}
		ret = gb_mods_aud_get_speaker_preset_eq(get_preset, connection);
		if (ret) {
			dev_warn(&connection->bundle->dev,
					"failed to get spkr preset eq %d\n",
					ret);
			get_preset->preset_eq = GB_AUDIO_SPEAKER_PRESET_EQ_NONE;
		}
		snd_codec.spkr_preset = get_preset;
	}

	/* set current use case and sys volume */
	ret = gb_mods_aud_set_playback_usecase(
			connection,
			BIT(snd_codec.playback_use_case));
	if (ret)
		pr_warn("%s: failed to set mods codec playback use case\n",
				__func__);

	ret = gb_mods_aud_set_sys_vol(connection, snd_codec.sys_vol_mb);
	if (ret)
		pr_warn("%s: failed to set mods codec sys volume\n", __func__);

	mods_vol_range_step = snd_codec.vol_range->vol_range.step;
	if (mods_vol_range_step != 0) {
		/* calculate remote codec vol step and set it*/
		mods_vol_step =
		(snd_codec.mods_vol_step*MODS_VOL_STEP)/mods_vol_range_step;
		ret = gb_mods_aud_set_vol(connection, mods_vol_step);
		if (ret)
			pr_warn("%s:failed to set mods codec volume\n",
				__func__);
	}

	kref_init(&snd_codec.mods_aud_kref);
	gb_connection_get(snd_codec.mods_aud_connection);
	/* report audio devices available to user space now
	 * if i2s mgmt connection is also initialized
	*/
	if (snd_codec.report_devices && snd_codec.mgmt_connection)
		snd_codec.report_devices(&snd_codec);
	mutex_unlock(&snd_codec.lock);

	connection->private = &snd_codec;

	return 0;

free_aud_dev:
	kfree(get_devices);
	snd_codec.aud_devices = NULL;
get_dev_null:
	snd_codec.use_cases = NULL;
free_use_case:
	kfree(get_use_cases);
set_vol_null:
	snd_codec.vol_range = NULL;
free_get_vol:
	kfree(get_vol);
out:
	snd_codec.mods_aud_connection = NULL;
	mutex_unlock(&snd_codec.lock);
	return ret;
}

static void gb_mods_audio_connection_exit(struct gb_connection *connection)
{
	struct gb_snd_codec	*codec =
			(struct gb_snd_codec *)connection->private;

	mutex_lock(&codec->lock);
	gb_mods_audio_put(codec);
	mutex_unlock(&codec->lock);
}

static void gb_i2s_mgmt_connection_exit(struct gb_connection *connection)
{
	struct gb_snd_codec *snd_codec =
			(struct gb_snd_codec *)connection->private;

	mutex_lock(&snd_codec->lock);
	gb_mods_i2s_put(snd_codec);
	mutex_unlock(&snd_codec->lock);
}

static int gb_i2s_mgmt_report_event_recv(u8 type, struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct gb_i2s_mgmt_report_event_request *req = op->request->payload;
	char *event_name;

	if (type != GB_I2S_MGMT_TYPE_REPORT_EVENT) {
		dev_err(&connection->bundle->dev, "Invalid request type: %d\n",
			type);
		return -EINVAL;
	}

	if (op->request->payload_size < sizeof(*req)) {
		dev_err(&connection->bundle->dev,
			"Short request received (%zu < %zu)\n",
			op->request->payload_size, sizeof(*req));
		return -EINVAL;
	}

	switch (req->event) {
	case GB_I2S_MGMT_EVENT_UNSPECIFIED:
		event_name = "UNSPECIFIED";
		break;
	case GB_I2S_MGMT_EVENT_HALT:
		/* XXX Should stop streaming now */
		event_name = "HALT";
		break;
	case GB_I2S_MGMT_EVENT_INTERNAL_ERROR:
		event_name = "INTERNAL_ERROR";
		break;
	case GB_I2S_MGMT_EVENT_PROTOCOL_ERROR:
		event_name = "PROTOCOL_ERROR";
		break;
	case GB_I2S_MGMT_EVENT_FAILURE:
		event_name = "FAILURE";
		break;
	case GB_I2S_MGMT_EVENT_OUT_OF_SEQUENCE:
		event_name = "OUT_OF_SEQUENCE";
		break;
	case GB_I2S_MGMT_EVENT_UNDERRUN:
		event_name = "UNDERRUN";
		break;
	case GB_I2S_MGMT_EVENT_OVERRUN:
		event_name = "OVERRUN";
		break;
	case GB_I2S_MGMT_EVENT_CLOCKING:
		event_name = "CLOCKING";
		break;
	case GB_I2S_MGMT_EVENT_DATA_LEN:
		event_name = "DATA_LEN";
		break;
	default:
		dev_warn(&connection->bundle->dev,
			 "Unknown I2S Event received: %d\n", req->event);
		return -EINVAL;
	}

	dev_warn(&connection->bundle->dev, "I2S Event received: %d - '%s'\n",
		 req->event, event_name);

	return 0;
}

static int gb_mods_audio_event_recv(u8 type, struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct gb_audio_report_devices_request *req = op->request->payload;
	struct gb_snd_codec	*codec =
			(struct gb_snd_codec *)connection->private;

	if (type != GB_AUDIO_DEVICES_REPORT_EVENT) {
		dev_err(&connection->bundle->dev, "Invalid request type: %d\n",
			type);
		return -EINVAL;
	}

	if (!codec) {
		dev_err(&connection->bundle->dev, "snd_codec not yet initialized\n");
		return -EAGAIN;
	}

	if (op->request->payload_size < sizeof(*req)) {
		dev_err(&connection->bundle->dev, "Short request received (%zu < %zu)\n",
			op->request->payload_size, sizeof(*req));
		return -EINVAL;
	}
	codec->aud_devices->devices.in_devices =
						cpu_to_le32(req->devices.in_devices);
	codec->aud_devices->devices.out_devices =
						cpu_to_le32(req->devices.out_devices);
	if (codec->report_devices)
		codec->report_devices(codec);

	dev_dbg(&connection->bundle->dev, "available audio devices changed\n");

	return 0;
}

static int gb_audio_register_mods_codec(struct platform_driver *plat)
{
	int err;

	err = platform_driver_register(plat);
	if (err) {
		pr_err("Can't register mods codec driver: %d\n", -err);
		return err;
	}

	snd_codec.codec_dev.name = "mods_codec";
	snd_codec.codec_dev.id = 0;
	snd_codec.codec_dev.dev.release = default_release; /* XXX - suspicious */
	/* initialize mod vol step to 0xff, 0db attenuation by default */
	snd_codec.mods_vol_step = 0xff;
	snd_codec.codec_dev.dev.platform_data = &snd_codec;

	err = platform_device_register(&snd_codec.codec_dev);
	if (err) {
		platform_driver_unregister(plat);
		pr_err("mods codec platform dev register failed\n");
		return -EINVAL;
	}

	return 0;
}

static struct gb_protocol gb_i2s_mgmt_protocol = {
	.name			= GB_AUDIO_MGMT_DRIVER_NAME,
	.id			= GREYBUS_PROTOCOL_I2S_MGMT,
	.major			= GB_I2S_MGMT_VERSION_MAJOR,
	.minor			= GB_I2S_MGMT_VERSION_MINOR,
	.connection_init	= gb_i2s_mgmt_connection_init,
	.connection_exit	= gb_i2s_mgmt_connection_exit,
	.request_recv		= gb_i2s_mgmt_report_event_recv,
};

static struct gb_protocol gb_mods_audio_protocol = {
	.name			= GB_MODS_AUDIO_DRIVER_NAME,
	.id			= GREYBUS_PROTOCOL_MODS_AUDIO,
	.major			= GB_MODS_AUDIO_VERSION_MAJOR,
	.minor			= GB_MODS_AUDIO_VERSION_MINOR,
	.connection_init	= gb_mods_audio_connection_init,
	.connection_exit	= gb_mods_audio_connection_exit,
	.request_recv		= gb_mods_audio_event_recv,
};

/*
 * This is the basic hook get things initialized and registered w/ gb
 */

static int __init gb_audio_protocol_init(void)
{
	int err;

	mutex_init(&snd_codec.lock);

	err = gb_protocol_register(&gb_i2s_mgmt_protocol);
	if (err) {
		pr_err("Can't register i2s mgmt protocol driver: %d\n", -err);
		return err;
	}

	err = gb_protocol_register(&gb_mods_audio_protocol);
	if (err) {
		pr_err("Can't register mods Audio protocol: %d\n", -err);
		goto err_unregister_i2s_mgmt;
	}

	/* mods codec is registered with platform and will be used when
	 * pcm is routed through platform dependent I2S Intf
	 * instead of pcm tunneling.
	*/
	err = gb_audio_register_mods_codec(&gb_audio_mods_driver);
	if (err) {
		pr_err("Can't register mods codec driver: %d\n", err);
		goto err_unregister_mods_aud;
	}
	return 0;

err_unregister_mods_aud:
	gb_protocol_deregister(&gb_mods_audio_protocol);
err_unregister_i2s_mgmt:
	gb_protocol_deregister(&gb_i2s_mgmt_protocol);
	return err;
}
module_init(gb_audio_protocol_init);

static void __exit gb_audio_protocol_exit(void)
{
	platform_device_unregister(&snd_codec.codec_dev);
	platform_driver_unregister(&gb_audio_mods_driver);
	gb_protocol_deregister(&gb_mods_audio_protocol);
	gb_protocol_deregister(&gb_i2s_mgmt_protocol);
}
module_exit(gb_audio_protocol_exit);

MODULE_LICENSE("GPL v2");
