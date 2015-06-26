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

struct slice_codec_dai {
	struct gb_snd_codec *snd_codec;
	atomic_t pcm_triggered;
	int is_params_set;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct snd_pcm_substream *substream;
};

static void slice_codec_work(struct work_struct *work)
{
	struct slice_codec_dai *priv =
			container_of(work, struct slice_codec_dai, work);
	struct gb_snd_codec *gb_codec = priv->snd_codec;
	struct gb_snd *snd_dev;
	int err;
	int port;

	if (!priv->is_params_set) {
		pr_err("%s: params not set returning\n", __func__);
		return;
	}

	list_for_each_entry(snd_dev, gb_codec->gb_snd_devs, list) {

		if (snd_dev && snd_dev->mgmt_connection &&
					(snd_dev->i2s_rx_connection || snd_dev->i2s_tx_connection)) {
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
}

static int slice_codec_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct slice_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);
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
			err = gb_i2s_mgmt_set_cfg(snd_dev, rate, chans, bytes_per_chan, is_le);
			if (!err)
				priv->is_params_set = true;
		}
	}

	return 0;
}

static int slice_codec_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct slice_codec_dai *priv = snd_soc_codec_get_drvdata(dai->codec);
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


static int slice_codec_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int slice_codec_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static const struct snd_soc_dai_ops slice_codec_dai_ops = {
	.hw_params = slice_codec_hw_params,
	.trigger	= slice_codec_dai_trigger,
	.set_fmt	= slice_codec_dai_set_fmt,
	.set_sysclk = slice_codec_set_dai_sysclk,
};


static const struct snd_soc_dapm_widget slice_dai_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("SLICE_DAI_RX", "Slice Dai Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SLICE_DAI_TX", "Slice Dai Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_OUTPUT("SLICE_DAI_RX Playback"),
};

static const struct snd_soc_dapm_route slice_codec_dapm_routes[] = {
	{"SLICE_DAI_RX", NULL, "Slice Dai Playback"},
	{"Slice Dai Capture", NULL, "SLICE_DAI_TX"},
};

static int slice_codec_probe(struct snd_soc_codec *codec)
{

	snd_soc_dapm_new_widgets(&codec->dapm);
	snd_soc_dapm_add_routes(&codec->dapm, slice_codec_dapm_routes,
			ARRAY_SIZE(slice_codec_dapm_routes));
	snd_soc_dapm_sync(&codec->dapm);

	pr_info("slice codec probed\n");
	return 0;
}

static int slice_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_slice = {
	.probe = slice_codec_probe,
	.remove = slice_codec_remove,
};


static struct snd_soc_dai_driver slice_codec_codec_dai = {
	.name			= "slice-codec-dai",
	.playback = {
		.stream_name = "Slice Dai Playback",
		.rates		= GB_RATES,
		.formats	= GB_FMTS,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.capture = {
		.stream_name = "Slice Dai Capture",
		.rates		= GB_RATES,
		.formats	= GB_FMTS,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops = &slice_codec_dai_ops,

};

static int slice_codec_dai_probe(struct platform_device *pdev)
{
	int ret;
	struct slice_codec_dai *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct slice_codec_dai),
			       GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->snd_codec = (struct gb_snd_codec *)pdev->dev.platform_data;

	priv->workqueue = alloc_workqueue("slice-codec", WQ_HIGHPRI, 0);
	if (!priv->workqueue) {
		ret = -ENOMEM;
		pr_err("%s: failed to allocate work queue", __func__);
		goto wq_fail;
	}

	INIT_WORK(&priv->work, slice_codec_work);

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_slice,
						&slice_codec_codec_dai, 1);

	if (ret) {
		pr_err("%s: failed to register slice codec", __func__);
		goto codec_reg_fail;
	}

	dev_set_drvdata(&pdev->dev, priv);

	return 0;
codec_reg_fail:
	destroy_workqueue(priv->workqueue);
wq_fail:
	return ret;
}

static int slice_codec_dai_remove(struct platform_device *pdev)
{
	struct slice_codec_dai *priv = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);
	destroy_workqueue(priv->workqueue);
	return 0;
}

struct platform_driver gb_audio_slice_driver = {
	.driver = {
		.name = "slice_codec",
	},
	.probe = slice_codec_dai_probe,
	.remove  = slice_codec_dai_remove,
};

