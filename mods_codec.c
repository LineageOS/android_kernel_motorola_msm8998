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

#define MODS_VOL_STEP		50
#define MODS_MIN_VOL		-12750

struct mods_codec_dai {
	struct gb_snd_codec *snd_codec;
	struct mods_codec_device *m_dev;
	atomic_t pcm_triggered;
	int is_params_set;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct snd_pcm_substream *substream;
	struct snd_soc_codec *codec;
	uint32_t vol_step;
	uint8_t use_case;
	int sys_vol_step;
	struct gb_aud_devices enabled_devices;
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
static int mods_codec_get_out_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_set_out_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_get_in_enabled_devices(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int mods_codec_set_in_enabled_devices(struct snd_kcontrol *kcontrol,
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
	SOC_SINGLE_EXT("Mods Enable Output Devices", SND_SOC_NOPM,
				0, 0x80000000, 0, mods_codec_get_out_enabled_devices,
				mods_codec_set_out_enabled_devices),
	SOC_SINGLE_EXT("Mods Enable Input Devices", SND_SOC_NOPM,
				0, 0x80000000, 0, mods_codec_get_in_enabled_devices,
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
	uint8_t use_case = ucontrol->value.integer.value[0];
	int ret;

	mutex_lock(&gb_codec->lock);
	if (!mods_codec_check_connection(gb_codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
				__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	if (gb_codec->use_cases->use_cases & BIT(use_case)) {
		ret = gb_mods_aud_set_supported_usecase(
					gb_codec->mods_aud_connection,
					BIT(use_case));
		if (ret) {
			pr_err("%s: failed to set mods codec use case\n", __func__);
			mutex_unlock(&gb_codec->lock);
			return -EIO;
		}
		priv->use_case = use_case;
	}
	mutex_unlock(&gb_codec->lock);
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
	uint32_t mods_vol_step;
	int ret;
	uint32_t vol_step = ucontrol->value.integer.value[0];

	mutex_lock(&gb_codec->lock);
	if (!mods_codec_check_connection(gb_codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	/* calculate remote codec vol step */
	mods_vol_step = (vol_step*MODS_VOL_STEP)/(gb_codec->vol_range->vol_range.step);
	ret = gb_mods_aud_set_vol(gb_codec->mods_aud_connection, mods_vol_step);
	if (ret) {
		pr_err("%s: failed to set mods codec volume\n", __func__);
		mutex_unlock(&gb_codec->lock);
		return -EIO;
	}
	priv->vol_step = mods_vol_step;
	mutex_unlock(&gb_codec->lock);

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
	int mods_vol_db;
	int ret;
	uint32_t vol_step = ucontrol->value.integer.value[0];

	mutex_lock(&gb_codec->lock);
	if (!mods_codec_check_connection(gb_codec)) {
		pr_err("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	/* calculate remote codec vol db */
	mods_vol_db = (vol_step*MODS_VOL_STEP);
	ret = gb_mods_aud_set_sys_vol(gb_codec->mods_aud_connection,
			mods_vol_db);
	if (ret) {
		pr_err("%s: failed to set mods codec sys volume\n", __func__);
		mutex_unlock(&gb_codec->lock);
		return -EIO;
	}
	priv->sys_vol_step = vol_step;
	mutex_unlock(&gb_codec->lock);

	return 0;
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
		pr_err("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}
	if ((gb_codec->aud_devices->devices.out_devices & out_mods_devices) !=
			out_mods_devices) {
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}

	ret = gb_mods_aud_enable_devices(gb_codec->mods_aud_connection,
			priv->enabled_devices.in_devices, out_mods_devices);
	if (ret) {
		pr_err("%s: failed to enable mods aud devices out\n", __func__);
		mutex_unlock(&gb_codec->lock);
		return -EIO;
	}
	priv->enabled_devices.out_devices = out_mods_devices;
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
		pr_err("%s: audio mods connection is not init'ed yet\n",
			__func__);
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}

	if ((gb_codec->aud_devices->devices.in_devices & in_mods_devices) !=
			in_mods_devices) {
		mutex_unlock(&gb_codec->lock);
		return -EINVAL;
	}

	ret = gb_mods_aud_enable_devices(gb_codec->mods_aud_connection,
			in_mods_devices, priv->enabled_devices.out_devices);
	if (ret) {
		pr_err("%s: failed to enable mods aud devices in\n", __func__);
		mutex_unlock(&gb_codec->lock);
		return -EIO;
	}
	priv->enabled_devices.in_devices = in_mods_devices;
	mutex_unlock(&gb_codec->lock);

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
	SND_SOC_DAPM_INPUT("MODS_DAI_TX Capture"),
};

static const struct snd_soc_dapm_route mods_codec_dapm_routes[] = {
	{"MODS_DAI_RX", NULL, "Mods Dai Playback"},
	{"Mods Dai Capture", NULL, "MODS_DAI_TX"},
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
	snd_soc_dapm_sync(&codec->dapm);

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
	pr_info("%s report change in mods audio devices 0x%x\n",
			__func__, codec->aud_devices->devices.out_devices);
	count = scnprintf(buf, PAGE_SIZE,
				"mods_codec_in_devices=%d;mods_codec_out_devices=%d\n",
				codec->aud_devices->devices.in_devices,
				codec->aud_devices->devices.out_devices);
	mutex_unlock(&codec->lock);

	return count;
}

static DEVICE_ATTR_RO(mods_codec_devices);

static struct attribute *mods_codec_attrs[] = {
	&dev_attr_mods_codec_devices.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mods_codec);

int mods_codec_report_devices(struct gb_snd_codec *codec)
{

	pr_info("%s report change in mods audio devices 0x%x\n",
			__func__, codec->aud_devices->devices.out_devices);

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

