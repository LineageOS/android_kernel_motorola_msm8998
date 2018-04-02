/*
 * Copyright (C) 2015 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mods_codec_dev.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "audio.h"
#include "kernel_ver.h"

struct mods_codec_dai {
	struct gb_snd_codec *snd_codec;
	struct mods_codec_device *m_dev;
	atomic_t playback_pcm_triggered;
	atomic_t capture_pcm_triggered;
	bool is_params_set;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct snd_pcm_substream *substream;
	struct snd_soc_codec *codec;
	uint32_t vol_step;
	struct gb_aud_devices enabled_devices;
	bool tx_active;
	bool rx_active;
};

/* declare 0 to -127.5 vol range with step 0.5 db */
static const DECLARE_TLV_DB_SCALE(mods_tlv_0_5, \
											MODS_MIN_VOL, MODS_VOL_STEP, 0);

static int mods_codec_set_usecase(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_get_usecase(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_get_vol(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_set_vol(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_get_sys_vol(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_set_sys_vol(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_get_out_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_set_out_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_get_in_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_set_in_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);

static const char *const mods_playback_use_case[] = {
	"None",
	"Music",
	"Voice",
	"Ringtone",
	"Sonifications"
};

static const char *const mods_capture_use_case[] = {
	"Default",
	"Voice",
	"Raw",
	"Camcorder",
	"Ambisonic",
	"Voice-Rec",
};

static const struct soc_enum mods_codec_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mods_playback_use_case),
						mods_playback_use_case),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mods_capture_use_case),
						mods_capture_use_case),
};

static const struct snd_kcontrol_new mods_codec_snd_controls[] = {
	SOC_SINGLE_EXT_TLV("Mods Codec Volume", SND_SOC_NOPM,
			0, 0xff, 0, mods_codec_get_vol, mods_codec_set_vol, mods_tlv_0_5),
	SOC_SINGLE_EXT("Mods Codec System Volume", SND_SOC_NOPM,
			0, 0x7fff, 0, mods_codec_get_sys_vol, mods_codec_set_sys_vol),
	SOC_ENUM_EXT("Mods Set Playback Use Case", mods_codec_enum[0],
				mods_codec_get_usecase,
				mods_codec_set_usecase),
	SOC_ENUM_EXT("Mods Set Capture Use Case", mods_codec_enum[1],
				mods_codec_get_usecase,
				mods_codec_set_usecase),
	SOC_SINGLE_EXT("Mods Enable Output Devices", SND_SOC_NOPM,
				0, 0x7fffffff, 0, mods_codec_get_out_enabled_devices,
				mods_codec_set_out_enabled_devices),
	SOC_SINGLE_EXT("Mods Enable Input Devices", SND_SOC_NOPM,
				0, 0x7fffffff, 0, mods_codec_get_in_enabled_devices,
				mods_codec_set_in_enabled_devices),
};

static inline int mods_codec_check_connection(struct gb_snd_codec *gb_codec)
{
	if (!gb_codec->mods_aud_connection)
		return 0;

	return 1;
}

static void mods_codec_work(struct work_struct *work)
{
	struct mods_codec_dai *priv =
			container_of(work, struct mods_codec_dai, work);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	int err;
	bool *port_active;
	int pcm_triggered;
	uint16_t port_type;

	if (!gb_codec) {
		priv->rx_active = false;
		priv->tx_active = false;
		priv->is_params_set = false;
		return;
	}

	mutex_lock(&gb_codec->lock);
	if (!priv->is_params_set) {
		pr_err("%s: params not set returning\n", __func__);
		mutex_unlock(&gb_codec->lock);
		return;
	}

	if (!gb_codec->mgmt_connection) {
		/* Always clear the rx/tx port active status
		 * when greybus connection down
		 */
		priv->rx_active = false;
		priv->tx_active = false;
		priv->is_params_set = false;
		mutex_unlock(&gb_codec->lock);
		return;
	}

	if (priv->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		port_active = &priv->rx_active;
		pcm_triggered = atomic_read(&priv->playback_pcm_triggered);
		port_type = GB_I2S_MGMT_PORT_TYPE_RECEIVER;
	} else if (priv->substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		port_active = &priv->tx_active;
		pcm_triggered = atomic_read(&priv->capture_pcm_triggered);
		port_type = GB_I2S_MGMT_PORT_TYPE_TRANSMITTER;
	} else {
		mutex_unlock(&gb_codec->lock);
		return;
	}

	gb_mods_i2s_get(gb_codec);
	mutex_unlock(&gb_codec->lock);


	if (!(*port_active) && pcm_triggered) {
		pr_debug("%s(): activate snd dev i2s port: %d\n",
				__func__, port_type);
		err = gb_i2s_mgmt_activate_port(gb_codec->mgmt_connection,
				port_type);
		if (err)
			pr_err("%s() failed to activate I2S port %d\n",
				__func__, port_type);
		else
			*port_active = true;
	} else if ((*port_active) && !pcm_triggered) {
		pr_debug("%s(): deactivate snd dev i2s port: %d\n",
				__func__, port_type);
		err = gb_i2s_mgmt_deactivate_port(gb_codec->mgmt_connection,
					port_type);
		if (err)
			pr_err("%s() failed to deactivate I2S port %d\n",
				__func__, port_type);

		*port_active = false;
	}

	mutex_lock(&gb_codec->lock);
	gb_mods_i2s_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

}

static int mods_codec_get_usecase(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;

	if (!strncmp(kcontrol->id.name, "Mods Set Playback Use Case",
				strlen(kcontrol->id.name)))
		ucontrol->value.integer.value[0] = gb_codec->playback_use_case;
	else
		ucontrol->value.integer.value[0] = gb_codec->capture_use_case;

	return 0;
}

static int mods_codec_set_usecase(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	uint32_t use_case = ucontrol->value.integer.value[0];
	int ret;

	mutex_lock(&gb_codec->lock);
	/* save the use case if mod is attached or not, saved
	 * use case will be set when mod is attached.
	 */
	if (!strncmp(kcontrol->id.name, "Mods Set Playback Use Case",
				strlen(kcontrol->id.name)))
		gb_codec->playback_use_case = use_case;
	else
		gb_codec->capture_use_case = use_case;

	if (!mods_codec_check_connection(gb_codec)) {
		pr_debug("%s: audio mods connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	gb_mods_audio_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	if (!strncmp(kcontrol->id.name, "Mods Set Playback Use Case",
				strlen(kcontrol->id.name))) {
		ret = gb_mods_aud_set_playback_usecase(
					gb_codec->mods_aud_connection,
					BIT(use_case));
		if (ret)
			pr_err("%s: failed to set mods codec use case\n", __func__);
	} else {
		ret = gb_mods_aud_set_capture_usecase(
				gb_codec->mods_aud_connection,
				BIT(use_case));
		if (ret)
			pr_err("%s: failed to set mods codec use case\n", __func__);
	}

	mutex_lock(&gb_codec->lock);
	gb_mods_audio_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return ret;
}

static int mods_codec_get_vol(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;

	/* should we read vol step from remote end instead of cached value ?*/
	ucontrol->value.integer.value[0] = gb_codec->mods_vol_step;

	return 0;
}

static int mods_codec_set_vol(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	uint32_t mods_vol_step;
	int ret;
	uint32_t vol_step = ucontrol->value.integer.value[0];

	mutex_lock(&gb_codec->lock);
	gb_codec->mods_vol_step = vol_step;
	if (!mods_codec_check_connection(gb_codec)) {
		pr_debug("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	if (gb_codec->vol_range->vol_range.step == 0) {
		pr_err("%s: audio mods vol_range step is zero\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	gb_mods_audio_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	/* calculate remote codec vol step */
	mods_vol_step = (vol_step*MODS_VOL_STEP)/(gb_codec->vol_range->vol_range.step);
	ret = gb_mods_aud_set_vol(gb_codec->mods_aud_connection, mods_vol_step);
	if (ret)
		pr_err("%s: failed to set mods codec volume\n", __func__);

	priv->vol_step = mods_vol_step;

	mutex_lock(&gb_codec->lock);
	gb_mods_audio_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return ret;
}

static int mods_codec_get_sys_vol(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;

	ucontrol->value.integer.value[0] = gb_codec->sys_vol_mb;

	return 0;
}

static int mods_codec_set_sys_vol(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	int ret;
	/* mods audio greybus calls out this value as attenuation,
	 * -ve value should be sent to mods.
	 */
	int sys_vol_mb = ucontrol->value.integer.value[0] * -1;

	mutex_lock(&gb_codec->lock);
	gb_codec->sys_vol_mb = sys_vol_mb;
	if (!mods_codec_check_connection(gb_codec)) {
		pr_debug("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	gb_mods_audio_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	ret = gb_mods_aud_set_sys_vol(gb_codec->mods_aud_connection,
			sys_vol_mb);
	if (ret)
		pr_err("%s: failed to set mods codec sys volume\n", __func__);

	mutex_lock(&gb_codec->lock);
	gb_mods_audio_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return ret;
}

static int mods_codec_get_out_enabled_devices(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = priv->enabled_devices.out_devices;

	return 0;
}

static int mods_codec_set_out_enabled_devices(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	int ret;
	uint32_t out_mods_devices = ucontrol->value.integer.value[0];

	mutex_lock(&gb_codec->lock);
	if (!mods_codec_check_connection(gb_codec)) {
		pr_debug("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	if ((gb_codec->aud_devices->devices.out_devices & out_mods_devices) !=
			out_mods_devices) {
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}

	gb_mods_audio_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	ret = gb_mods_aud_enable_devices(gb_codec->mods_aud_connection,
			priv->enabled_devices.in_devices, out_mods_devices);
	if (ret)
		pr_err("%s: failed to enable mods aud devices out\n", __func__);

	priv->enabled_devices.out_devices = out_mods_devices;

	mutex_lock(&gb_codec->lock);
	gb_mods_audio_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return 0;
}

static int mods_codec_get_in_enabled_devices(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = priv->enabled_devices.in_devices;

	return 0;
}

static int mods_codec_set_in_enabled_devices(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	int ret;
	uint32_t in_mods_devices = ucontrol->value.integer.value[0];

	mutex_lock(&gb_codec->lock);

	if (!mods_codec_check_connection(gb_codec)) {
		pr_debug("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}

	if ((gb_codec->aud_devices->devices.in_devices & in_mods_devices) !=
			in_mods_devices) {
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	gb_mods_audio_get(gb_codec);
	mutex_unlock(&gb_codec->lock);


	ret = gb_mods_aud_enable_devices(gb_codec->mods_aud_connection,
			in_mods_devices, priv->enabled_devices.out_devices);
	if (ret)
		pr_err("%s: failed to enable mods aud devices in\n", __func__);

	priv->enabled_devices.in_devices = in_mods_devices;
	mutex_lock(&gb_codec->lock);
	gb_mods_audio_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return 0;
}

static int mods_codec_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	uint32_t rate, format;
	uint8_t chans;
	int bytes_per_chan, is_le;
	int err = 0;

	if (priv->is_params_set == 1) {
		pr_debug("%s: params already set\n", __func__);
		return err;
	}

	mutex_lock(&gb_codec->lock);
	if (!gb_codec->mgmt_connection) {
		pr_err("%s: audio i2s mgmt connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	gb_mods_i2s_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	rate = params_rate(params);
	chans = params_channels(params);
	format = params_format(params);
	bytes_per_chan = snd_pcm_format_width(format) / 8;
	is_le = snd_pcm_format_little_endian(format);

	err = gb_i2s_mgmt_set_cfg(gb_codec, rate, chans, format,
						bytes_per_chan, is_le);
	if (!err)
		priv->is_params_set = true;
	else
		pr_err("%s: failed to set hw params\n",
			__func__);


	mutex_lock(&gb_codec->lock);
	gb_mods_i2s_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return err;
}

static int mods_codec_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);

	priv->substream = substream;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			atomic_set(&priv->playback_pcm_triggered, 1);
		else
			atomic_set(&priv->capture_pcm_triggered, 1);
		queue_work(priv->workqueue, &priv->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			atomic_set(&priv->playback_pcm_triggered, 0);
		else
			atomic_set(&priv->capture_pcm_triggered, 0);
		queue_work(priv->workqueue, &priv->work);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int mods_codec_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int mods_codec_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int mods_codec_start(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	int ret;

	mutex_lock(&gb_codec->lock);
	if (!gb_codec->mgmt_connection) {
		pr_err("%s: audio i2s mgmt connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	gb_mods_i2s_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = gb_i2s_mgmt_send_start(gb_codec,
				GB_I2S_MGMT_PORT_TYPE_RECEIVER, true);
	else
		ret = gb_i2s_mgmt_send_start(gb_codec,
				GB_I2S_MGMT_PORT_TYPE_TRANSMITTER, true);

	if (ret == -ENOTSUPP)
		ret = 0;
	mutex_lock(&gb_codec->lock);
	gb_mods_i2s_put(gb_codec);
	mutex_unlock(&gb_codec->lock);

	return ret;
}

static void mods_codec_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;

	mutex_lock(&gb_codec->lock);
	if (!gb_codec->mgmt_connection) {
		pr_err("%s: audio i2s mgmt connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&gb_codec->lock);
		return;
	}
	gb_mods_i2s_get(gb_codec);
	mutex_unlock(&gb_codec->lock);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		gb_i2s_mgmt_send_start(gb_codec,
				GB_I2S_MGMT_PORT_TYPE_RECEIVER, false);
		priv->rx_active = false;
	} else {
		gb_i2s_mgmt_send_start(gb_codec,
				GB_I2S_MGMT_PORT_TYPE_TRANSMITTER, false);
		priv->tx_active = false;
	}

	if (!priv->rx_active && !priv->tx_active)
		priv->is_params_set = false;

	mutex_lock(&gb_codec->lock);
	gb_mods_i2s_put(gb_codec);
	mutex_unlock(&gb_codec->lock);
}

static const struct snd_soc_dai_ops mods_codec_dai_ops = {
	.startup    = mods_codec_start,
	.hw_params = mods_codec_hw_params,
	.trigger	= mods_codec_dai_trigger,
	.set_fmt	= mods_codec_dai_set_fmt,
	.set_sysclk = mods_codec_set_dai_sysclk,
	.shutdown = mods_codec_shutdown,
};


static const struct snd_soc_dapm_widget mods_dai_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("MODS_DAI_RX", "Mods Dai Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MODS_DAI_TX", "Mods Dai Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_OUTPUT("MODS_DAI_RX Playback"),
	SND_SOC_DAPM_INPUT("MODS_DAI_TX Capture"),
};

static const struct snd_soc_dapm_route mods_codec_dapm_routes[] = {
	{"MODS_DAI_RX", NULL, "Mods Dai Playback"},
	{"Mods Dai Capture", NULL, "MODS_DAI_TX"},
#ifdef CONFIG_MODS_USE_EXTCODEC_MI2S
	{"MODS_DAI_RX Playback", NULL, "MODS_DAI_RX"},
	{"MODS_DAI_TX", NULL, "MODS_DAI_TX Capture"},
#endif
};

static int mods_codec_probe(struct snd_soc_codec *codec)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct snd_kcontrol *kcontrol;
	int i;
	int ret;

	priv->codec = codec;
	for (i = 0; i < ARRAY_SIZE(mods_codec_snd_controls); i++) {
		ret = snd_ctl_add(codec->component.card->snd_card,
			snd_ctl_new1(&mods_codec_snd_controls[i],
					&codec->component));
		if (ret) {
			pr_err("%s: failed to add codec control %s",
			__func__, mods_codec_snd_controls[i].name);
			goto cleanup;
		}
	}
	snd_soc_dapm_sync(snd_soc_codec_get_dapm(codec));

	pr_info("mods codec probed\n");

	return 0;
cleanup:
	for (--i; i >= 0; i--) {
		kcontrol = snd_soc_card_get_kcontrol(
					priv->codec->component.card,
					mods_codec_snd_controls[i].name);
		if (kcontrol)
			snd_ctl_remove(
				priv->codec->component.card->snd_card,
				kcontrol);
	}

	return ret;
}

static int mods_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static ssize_t mods_codec_devices_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t count;
	struct gb_snd_codec *codec;
	struct mods_codec_dai *priv = dev_get_drvdata(dev);

	codec = priv->snd_codec;
	mutex_lock(&codec->lock);
	if (!mods_codec_check_connection(codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&codec->lock);
		return 0;
	}
	pr_info("%s report change in mods audio devices out:0x%x in: 0x%x\n",
			__func__, codec->aud_devices->devices.out_devices,
			codec->aud_devices->devices.in_devices);
	count = scnprintf(buf, PAGE_SIZE,
				"mods_codec_in_devices=%d;mods_codec_out_devices=%d\n",
			le32_to_cpu(codec->aud_devices->devices.in_devices),
			le32_to_cpu(codec->aud_devices->devices.out_devices));
	mutex_unlock(&codec->lock);

	return count;
}

static DEVICE_ATTR_RO(mods_codec_devices);

static int mods_codec_convert_gb_rate(uint32_t gb_rate)
{
	switch (gb_rate) {
	case GB_I2S_MGMT_PCM_RATE_5512:
		return 5512;
	case GB_I2S_MGMT_PCM_RATE_8000:
		return 8000;
	case GB_I2S_MGMT_PCM_RATE_11025:
		return 11025;
	case GB_I2S_MGMT_PCM_RATE_16000:
		return 16000;
	case GB_I2S_MGMT_PCM_RATE_22050:
		return 22050;
	case GB_I2S_MGMT_PCM_RATE_32000:
		return 32000;
	case GB_I2S_MGMT_PCM_RATE_44100:
		return 44100;
	case GB_I2S_MGMT_PCM_RATE_48000:
		return 48000;
	case GB_I2S_MGMT_PCM_RATE_64000:
		return 64000;
	case GB_I2S_MGMT_PCM_RATE_88200:
		return 88200;
	case GB_I2S_MGMT_PCM_RATE_96000:
		return 96000;
	case GB_I2S_MGMT_PCM_RATE_176400:
		return 176400;
	case GB_I2S_MGMT_PCM_RATE_192000:
		return 192000;
	default:
		return -EINVAL;
	}
}

static uint8_t mods_codec_convert_gb_format(uint8_t gb_fmt)
{
	switch (gb_fmt) {
	case GB_I2S_MGMT_PCM_FMT_8:
		return 8;
	case GB_I2S_MGMT_PCM_FMT_16:
		return 16;
	case GB_I2S_MGMT_PCM_FMT_24:
		return 24;
	case GB_I2S_MGMT_PCM_FMT_32:
		return 32;
	case GB_I2S_MGMT_PCM_FMT_64:
		return 64;
	default:
		return -EINVAL;
	}
}

static ssize_t mods_codec_caps_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t count;
	struct gb_snd_codec *codec;
	struct mods_codec_dai *priv = dev_get_drvdata(dev);
	uint32_t rate_mask;
	uint8_t fmt_mask;
	int i = 0;
	int rate;
	int fmt;

	codec = priv->snd_codec;
	mutex_lock(&codec->lock);
	if (!mods_codec_check_connection(codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&codec->lock);
		return 0;
	}
	if (!codec->i2s_cfg_masks) {
		pr_err("%s: audio mods i2s cfg is not init'ed\n",
				__func__);
		mutex_unlock(&codec->lock);
		return 0;
	}
	count = scnprintf(buf, PAGE_SIZE, "mods_codec_rates=");
	rate_mask = le32_to_cpu(codec->i2s_cfg_masks->config.sample_frequency);
	while (rate_mask > 0) {
		if (rate_mask & 1) {
			rate = mods_codec_convert_gb_rate(BIT(i));
			if (rate > 0)
				count += scnprintf(buf + count, PAGE_SIZE, "%d,",
								rate);
		}
		rate_mask >>= 1;
		i++;
	}
	count += (scnprintf(buf + count -1, PAGE_SIZE,
					";mods_codec_sample_sizes=") -1);
	i = 0;
	fmt_mask = codec->i2s_cfg_masks->config.format;
	while (fmt_mask > 0) {
		if (fmt_mask & 1) {
			fmt = mods_codec_convert_gb_format(BIT(i));
			if (fmt > 0)
				count += scnprintf(buf + count, PAGE_SIZE, "%d,",
							fmt);
		}
		fmt_mask >>= 1;
		i++;
	}
	count += (scnprintf(buf + count -1, PAGE_SIZE,
				";mods_codec_max_channels=%d\n",
				codec->i2s_cfg_masks->config.num_channels) - 1);
	mutex_unlock(&codec->lock);

	return count;
}

static DEVICE_ATTR_RO(mods_codec_caps);

static ssize_t mods_codec_usecases_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t count;
	struct gb_snd_codec *codec;
	struct mods_codec_dai *priv = dev_get_drvdata(dev);

	codec = priv->snd_codec;
	mutex_lock(&codec->lock);
	if (!mods_codec_check_connection(codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&codec->lock);
		return 0;
	}
	count = scnprintf(buf, PAGE_SIZE,
			"mods_codec_playback_usecases=%d;mods_codec_capture_usecases=%d\n",
			le32_to_cpu(codec->use_cases->aud_use_cases.playback_usecases),
			le32_to_cpu(codec->use_cases->aud_use_cases.capture_usecases));
	mutex_unlock(&codec->lock);

	return count;
}

static DEVICE_ATTR_RO(mods_codec_usecases);

static ssize_t mods_codec_speaker_preset_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t count;
	struct gb_snd_codec *codec;
	struct mods_codec_dai *priv = dev_get_drvdata(dev);

	codec = priv->snd_codec;
	mutex_lock(&codec->lock);
	if (!mods_codec_check_connection(codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&codec->lock);
		return -ENODEV;
	}

	if (codec->spkr_preset)
		count = scnprintf(buf, PAGE_SIZE,
				"%d\n",
				le32_to_cpu(codec->spkr_preset->preset_eq));
	else
		count = scnprintf(buf, PAGE_SIZE,"%d\n",
					GB_AUDIO_SPEAKER_PRESET_EQ_NONE);
	mutex_unlock(&codec->lock);

	return count;
}

static DEVICE_ATTR_RO(mods_codec_speaker_preset);

static ssize_t mods_codec_mic_params_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t count;
	struct gb_snd_codec *codec;
	struct mods_codec_dai *priv = dev_get_drvdata(dev);
	struct gb_audio_get_mic_params_response *get_params;
	int ret;

	codec = priv->snd_codec;
	mutex_lock(&codec->lock);
	if (!mods_codec_check_connection(codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
				__func__);
		count = -ENODEV;
		goto out;
	}
	if (!gb_i2s_audio_is_ver_supported(codec->mods_aud_connection,
			GB_MODS_AUDIO_VERSION_BF_PARAMS_MAJOR,
			GB_MODS_AUDIO_VERSION_BF_PARAMS_MINOR)) {
		pr_warn("%s: audio mods does not support mic tuning params\n",
				__func__);
		count = -ENOTSUPP;
		goto out;
	}
	get_params = kzalloc(
		sizeof(struct gb_audio_get_mic_params_response),
		GFP_KERNEL);
	if (!get_params) {
		count = -ENOMEM;
		goto out;
	}

	ret = gb_mods_aud_get_mic_params(get_params,
			codec->mods_aud_connection);
	if (ret) {
		count = ret;
		goto params_free;
	}

	memcpy(buf, get_params->params, GB_AUDIO_MIC_PARAMS_SIZE);
	count = GB_AUDIO_MIC_PARAMS_SIZE;

params_free:
	kfree(get_params);
out:
	mutex_unlock(&codec->lock);
	return count;
}

static DEVICE_ATTR_RO(mods_codec_mic_params);

static struct attribute *mods_codec_attrs[] = {
	&dev_attr_mods_codec_devices.attr,
	&dev_attr_mods_codec_usecases.attr,
	&dev_attr_mods_codec_caps.attr,
	&dev_attr_mods_codec_speaker_preset.attr,
	&dev_attr_mods_codec_mic_params.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mods_codec);

int mods_codec_report_devices(struct gb_snd_codec *codec)
{

	pr_info("%s report change in mods audio devices out:0x%x in:0x%x\n",
			__func__, codec->aud_devices->devices.out_devices,
			codec->aud_devices->devices.in_devices);

	kobject_uevent(&codec->codec_dev.dev.kobj, KOBJ_CHANGE);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_mods = {
	.probe = mods_codec_probe,
	.remove = mods_codec_remove,
	.dapm_widgets = mods_dai_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mods_dai_dapm_widgets),
	.dapm_routes = mods_codec_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(mods_codec_dapm_routes),
};

static struct snd_soc_dai_driver mods_codec_codec_dai = {
	.name			= "mods-codec-dai",
	.playback = {
		.stream_name = "Mods Dai Playback",
		.rates		= GB_RATES,
		.formats	= GB_FMTS,
		.channels_min	= 1,
		.channels_max	= 4,
	},
	.capture = {
		.stream_name = "Mods Dai Capture",
		.rates		= GB_RATES,
		.formats	= GB_FMTS,
		.channels_min	= 1,
		.channels_max	= 4,
	},
	.ops = &mods_codec_dai_ops,
};

static const struct mods_codec_device_ops mods_dev_ops = {
	.dai_ops = &mods_codec_dai_ops,
	.probe = mods_codec_probe,
	.remove = mods_codec_remove,
};

static int mods_codec_dai_probe(struct platform_device *pdev)
{
	int ret;
	struct mods_codec_dai *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct mods_codec_dai),
				GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->snd_codec = (struct gb_snd_codec *)pdev->dev.platform_data;
	if (!priv->snd_codec)
		return -EINVAL;

	priv->workqueue = alloc_workqueue("mods-codec", WQ_HIGHPRI, 0);
	if (!priv->workqueue) {
		ret = -ENOMEM;
		pr_err("%s: failed to allocate work queue", __func__);
		goto wq_fail;
	}

	INIT_WORK(&priv->work, mods_codec_work);

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_mods,
						&mods_codec_codec_dai, 1);

	if (ret) {
		pr_err("%s: failed to register mods codec", __func__);
		goto codec_reg_fail;
	}

	dev_set_drvdata(&pdev->dev, priv);

	priv->m_dev = mods_codec_register_device(&pdev->dev, &mods_dev_ops,
					(void *)priv);
	if (!priv->m_dev) {
		pr_err("%s: failed to register mods codec bus dev", __func__);
		ret = -ENOMEM;
		goto codec_unreg;
	}

	ret = sysfs_create_groups(&pdev->dev.kobj, mods_codec_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs attr\n");
		goto mdev_unreg;
	}

	priv->snd_codec->report_devices = mods_codec_report_devices;

	return 0;

mdev_unreg:
	mods_codec_unregister_device(priv->m_dev);
codec_unreg:
	snd_soc_unregister_codec(&pdev->dev);
codec_reg_fail:
	destroy_workqueue(priv->workqueue);
wq_fail:
	return ret;
}

static int mods_codec_dai_remove(struct platform_device *pdev)
{
	int i;
	int ret;
	struct snd_kcontrol *kcontrol;
	struct mods_codec_dai *priv = dev_get_drvdata(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(mods_codec_snd_controls); i++) {
		kcontrol = snd_soc_card_get_kcontrol(
					priv->codec->component.card,
					mods_codec_snd_controls[i].name);
		if (kcontrol) {
			ret = snd_ctl_remove(
					priv->codec->component.card->snd_card,
					kcontrol);
			if (ret)
				pr_warn("%s: failed to remove codec control %s",
				__func__, mods_codec_snd_controls[i].name);
		}
	}
	sysfs_remove_groups(&pdev->dev.kobj, mods_codec_groups);
	mods_codec_unregister_device(priv->m_dev);
	snd_soc_unregister_codec(&pdev->dev);
	destroy_workqueue(priv->workqueue);
	return 0;
}

struct platform_driver gb_audio_mods_driver = {
	.driver = {
		.name = "mods_codec",
	},
	.probe = mods_codec_dai_probe,
	.remove  = mods_codec_dai_remove,
};

