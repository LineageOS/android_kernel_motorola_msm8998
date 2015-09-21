#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "audio.h"

#define MODS_VOL_STEP		50
#define MODS_MIN_VOL		-12750

struct mods_codec_dai {
	struct gb_snd_codec *snd_codec;
	atomic_t pcm_triggered;
	int is_params_set;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct snd_pcm_substream *substream;
	struct snd_soc_codec *codec;
	uint32_t vol_step;
	uint8_t use_case;
	int sys_vol_step;
};

/* declare 0 to -127.5 vol range with step 0.5 db */
static const DECLARE_TLV_DB_SCALE(mods_tlv_0_5, \
											MODS_MIN_VOL, MODS_VOL_STEP, 0);
static const DECLARE_TLV_DB_SCALE(mods_sys_tlv_0_5, \
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

static const char *const mods_use_case[] = {
	"None",
	"Music",
	"Voice",
	"Low Latency",
};

static const struct soc_enum mods_codec_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mods_use_case), mods_use_case),
};

static const struct snd_kcontrol_new mods_codec_snd_controls[] = {
	SOC_SINGLE_EXT_TLV("Mods Codec Volume", SND_SOC_NOPM,
			0, 0xff, 0, mods_codec_get_vol, mods_codec_set_vol, mods_tlv_0_5),
	SOC_SINGLE_EXT_TLV("Mods Codec System Volume", SND_SOC_NOPM,
			0, 0xff, 0, mods_codec_get_sys_vol, mods_codec_set_sys_vol,
				mods_sys_tlv_0_5),
	SOC_ENUM_EXT("Mods Set Use Case", mods_codec_enum[0],
				mods_codec_get_usecase,
				mods_codec_set_usecase),
};

static void mods_codec_work(struct work_struct *work)
{
	struct mods_codec_dai *priv =
			container_of(work, struct mods_codec_dai, work);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	struct gb_snd *snd_dev;
	int err;
	int port;

	if (!priv->is_params_set) {
		pr_err("%s: params not set returning\n", __func__);
		return;
	}

	list_for_each_entry(snd_dev, gb_codec->gb_snd_devs, list) {

		if (!snd_dev || !snd_dev->mgmt_connection)
			continue;
		if (!snd_dev->i2s_rx_connection && !snd_dev->i2s_tx_connection)
			continue;

		if (priv->substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			port = snd_dev->i2s_tx_connection->intf_cport_id;
		else
			port = snd_dev->i2s_rx_connection->intf_cport_id;

		if (!snd_dev->cport_active &&
			atomic_read(&priv->pcm_triggered)) {
			pr_debug("%s(): activate snd dev i2s port: %d\n",
					__func__, port);
			err = gb_i2s_mgmt_activate_cport(snd_dev->mgmt_connection,
					port);
			if (err)
				pr_err("%s() failed to activate I2S port %d\n",
					__func__, port);
			else
				snd_dev->cport_active = true;
		} else if (snd_dev->cport_active &&
				!atomic_read(&priv->pcm_triggered)) {
			pr_debug("%s(): deactivate snd dev i2s port: %d\n",
					__func__, port);
			err = gb_i2s_mgmt_deactivate_cport(snd_dev->mgmt_connection,
						port);
			if (err)
				pr_err("%s() failed to deactivate I2S port %d\n",
					__func__, port);
			else
				snd_dev->cport_active = false;
		}
	}
}

static int mods_codec_get_usecase(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = priv->use_case;

	return 0;
}

static int mods_codec_set_usecase(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	struct gb_snd *snd_dev;
	uint8_t use_case = ucontrol->value.integer.value[0];
	int ret;

	list_for_each_entry(snd_dev, gb_codec->gb_snd_devs, list) {

		if (!snd_dev || !snd_dev->mods_aud_connection || !use_case)
			continue;
		if (snd_dev->use_cases->use_cases & BIT(use_case)) {
			ret = gb_mods_aud_set_supported_usecase(
						snd_dev->mods_aud_connection,
						BIT(use_case));
			if (ret) {
				pr_err("%s: failed to set mods codec use case\n", __func__);
				return -EIO;
			}
			priv->use_case = use_case;
		}
	}

	return 0;
}

static int mods_codec_get_vol(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);

	/* should we read vol step from remote end instead of cached value ?*/
	ucontrol->value.integer.value[0] = priv->vol_step;

	return 0;
}

static int mods_codec_set_vol(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	struct gb_snd *snd_dev;
	uint32_t mods_vol_step;
	int ret;
	uint32_t vol_step = ucontrol->value.integer.value[0];

	list_for_each_entry(snd_dev, gb_codec->gb_snd_devs, list) {

		if (!snd_dev || !snd_dev->mods_aud_connection || !snd_dev->vol_range)
			continue;
		/* calculate remote codec vol step */
		mods_vol_step = (vol_step*MODS_VOL_STEP)/(snd_dev->vol_range->vol_range.step);
		ret = gb_mods_aud_set_vol(snd_dev->mods_aud_connection, mods_vol_step);
		if (ret) {
			pr_err("%s: failed to set mods codec volume\n", __func__);
			return -EIO;
		}
		priv->vol_step = mods_vol_step;
	}
	return 0;
}


static int mods_codec_get_sys_vol(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = priv->sys_vol_step;

	return 0;
}

static int mods_codec_set_sys_vol(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	struct gb_snd *snd_dev;
	int mods_vol_db;
	int ret;
	uint32_t vol_step = ucontrol->value.integer.value[0];

	list_for_each_entry(snd_dev, gb_codec->gb_snd_devs, list) {

		if (!snd_dev || !snd_dev->mods_aud_connection)
			continue;
		/* calculate remote codec vol db */
		mods_vol_db = (vol_step*MODS_VOL_STEP);
		ret = gb_mods_aud_set_sys_vol(snd_dev->mods_aud_connection,
				mods_vol_db);
		if (ret) {
			pr_err("%s: failed to set mods codec sys volume\n", __func__);
			return -EIO;
		}
		priv->sys_vol_step = vol_step;
	}
	return 0;
}

static int mods_codec_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	struct gb_snd *snd_dev;
	int rate, chans, bytes_per_chan, is_le;
	int err;

	rate = params_rate(params);
	chans = params_channels(params);
	bytes_per_chan = snd_pcm_format_width(params_format(params)) / 8;
	is_le = snd_pcm_format_little_endian(params_format(params));

	list_for_each_entry(snd_dev, gb_codec->gb_snd_devs, list) {
		if (snd_dev && snd_dev->mgmt_connection) {
			err = gb_i2s_mgmt_set_cfg(snd_dev, rate, chans,
							bytes_per_chan, is_le);
			if (!err)
				priv->is_params_set = true;
		}
	}

	return 0;
}

static int mods_codec_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);

	priv->substream = substream;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		atomic_set(&priv->pcm_triggered, 1);
		queue_work(priv->workqueue, &priv->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		atomic_set(&priv->pcm_triggered, 0);
		queue_work(priv->workqueue, &priv->work);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
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

static const struct snd_soc_dai_ops mods_codec_dai_ops = {
	.hw_params = mods_codec_hw_params,
	.trigger	= mods_codec_dai_trigger,
	.set_fmt	= mods_codec_dai_set_fmt,
	.set_sysclk = mods_codec_set_dai_sysclk,
};


static const struct snd_soc_dapm_widget mods_dai_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("MODS_DAI_RX", "Mods Dai Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MODS_DAI_TX", "Mods Dai Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_OUTPUT("MODS_DAI_RX Playback"),
};

static const struct snd_soc_dapm_route mods_codec_dapm_routes[] = {
	{"MODS_DAI_RX", NULL, "Mods Dai Playback"},
	{"Mods Dai Capture", NULL, "MODS_DAI_TX"},
};

static int mods_codec_probe(struct snd_soc_codec *codec)
{
	struct mods_codec_dai *priv = snd_soc_codec_get_drvdata(codec);

	priv->codec = codec;

	snd_soc_dapm_add_routes(&codec->dapm, mods_codec_dapm_routes,
			ARRAY_SIZE(mods_codec_dapm_routes));
	snd_soc_add_codec_controls(codec, mods_codec_snd_controls,
				 ARRAY_SIZE(mods_codec_snd_controls));
	snd_soc_dapm_sync(&codec->dapm);

	pr_info("mods codec probed\n");
	return 0;
}

static int mods_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_mods = {
	.dapm_widgets = mods_dai_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mods_dai_dapm_widgets),
	.probe = mods_codec_probe,
	.remove = mods_codec_remove,
};


static struct snd_soc_dai_driver mods_codec_codec_dai = {
	.name			= "mods-codec-dai",
	.playback = {
		.stream_name = "Mods Dai Playback",
		.rates		= GB_RATES,
		.formats	= GB_FMTS,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.capture = {
		.stream_name = "Mods Dai Capture",
		.rates		= GB_RATES,
		.formats	= GB_FMTS,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops = &mods_codec_dai_ops,

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

	return 0;
codec_reg_fail:
	destroy_workqueue(priv->workqueue);
wq_fail:
	return ret;
}

static int mods_codec_dai_remove(struct platform_device *pdev)
{
	struct mods_codec_dai *priv = dev_get_drvdata(&pdev->dev);

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

