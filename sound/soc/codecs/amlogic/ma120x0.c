#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/tas57xx.h>
#include <linux/amlogic/aml_gpio_consumer.h>

#include "ma120x0.h"

#define DEV_NAME	"ma120x0"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static void ma120x0_early_suspend(struct early_suspend *h);
static void ma120x0_late_resume(struct early_suspend *h);
#endif

#define MA120X0_RATES (SNDRV_PCM_RATE_44100 | \
		       SNDRV_PCM_RATE_48000 | \
				 	 SNDRV_PCM_RATE_88200 | \
			 	   SNDRV_PCM_RATE_96000)

#define MA120X0_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE )

/* Power-up register defaults */
struct reg_default ma120x0_reg_defaults[] = {
	{	0x01,	0x3c	},
};

struct ma120x0_platform_data {
	int (*init_func)(void);
	int (*early_suspend_func)(void);
	int (*suspend_func)(void);
	int (*resume_func)(void);
	int (*late_resume_func)(void);
	char *custom_init_value_table;
	int init_value_table_len;
	char *init_regs;
	int num_init_regs;
};
/* codec private data */
struct ma120x0_priv {
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct ma120x0_platform_data *pdata;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static const DECLARE_TLV_DB_SCALE(ma120x0_vol_tlv, -5000, 100,  0);

static const struct snd_kcontrol_new ma120x0_snd_controls[] = {
	SOC_SINGLE_RANGE_TLV("A.Mstr Vol Volume", MA_vol_db_master__a, 0, 0x18, 0x4a, 1, ma120x0_vol_tlv),
};

static int ma120x0_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	u16 blen = 0x00;

	struct snd_soc_codec *codec = dai->codec;
	//struct ma120x0_priv *ma120x0 = snd_soc_codec_get_drvdata(codec);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		blen = 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		blen = 0x08;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		blen = 0x00;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		blen = 0x00;
		break;
	default:
		dev_err(dai->dev, "Unsupported word length: %u\n",
			params_format(params));
		return -EINVAL;
	}

	// set word length
	snd_soc_update_bits(codec, MA_i2s_framesize__a, MA_i2s_framesize__mask, blen);

	return 0;
}

static int ma120x0_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	pr_debug("level = %d\n", level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* Full power on */
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		/* The chip runs through the power down sequence for us. */
		break;
	}
	codec->component.dapm.bias_level = level;

	return 0;
}

static const struct snd_soc_dai_ops ma120x0_dai_ops = {
	.hw_params = ma120x0_hw_params,
};

static struct snd_soc_dai_driver ma120x0_dai = {
	.name = DEV_NAME,
	.playback = {
		.stream_name = "HIFI Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MA120X0_RATES,
		.formats = MA120X0_FORMATS,
	},
	.ops = &ma120x0_dai_ops,
};

/*
static int reset_ma120x0_GPIO(struct snd_soc_codec *codec)
{
	struct ma120x0_priv *ma120x0 = snd_soc_codec_get_drvdata(codec);
	struct tas57xx_platform_data *pdata = ma120x0->pdata;
	int ret = 0;

	if (pdata->reset_pin < 0)
		return 0;

	ret = devm_gpio_request_one(codec->dev, pdata->reset_pin,
					    GPIOF_OUT_INIT_LOW,
					    "ma120x0-reset-pin");
	if (ret < 0)
		return -1;

	gpio_direction_output(pdata->reset_pin, GPIOF_OUT_INIT_LOW);
	udelay(1000);
	gpio_direction_output(pdata->reset_pin, GPIOF_OUT_INIT_HIGH);
	mdelay(15);

	return 0;
}
*/

static int ma120x0_clear_err(struct snd_soc_codec *codec)
{
 int ret = 0;

 //struct ma120x0_priv *ma120x0;
 //ma120x0 = snd_soc_component_get_drvdata(component);

 ret = snd_soc_update_bits(codec, MA_eh_clear__a, MA_eh_clear__mask, 0x00);
 if (ret < 0) return ret;

 ret = snd_soc_update_bits(codec, MA_eh_clear__a, MA_eh_clear__mask, 0x04);
 if (ret < 0) return ret;

 ret = snd_soc_update_bits(codec, MA_eh_clear__a, MA_eh_clear__mask, 0x00);
 if (ret < 0) return ret;

 return 0;
}


static int ma120x0_init(struct snd_soc_codec *codec)
{
	//struct ma120x0_priv *ma120x0 = snd_soc_codec_get_drvdata(codec);
  int ret = 0;
	//reset_ma120x0_GPIO(codec);

	dev_info(codec->dev, "ma120x0_init!\n");
	//snd_soc_write(codec, DDX_OSC_TRIM, 0x00);
	msleep(50);
	//snd_soc_write(codec, DDX_CLOCK_CTL, 0x6c);
	//snd_soc_write(codec, DDX_SYS_CTL_1, 0xa0);
	//snd_soc_write(codec, DDX_SERIAL_DATA_INTERFACE, 0x05);
	//snd_soc_write(codec, DDX_BKND_ERR, 0x02);

	//regmap_raw_write(ma120x0->regmap, DDX_INPUT_MUX, burst_data[0], 4);
	//regmap_raw_write(ma120x0->regmap, DDX_CH4_SOURCE_SELECT,
			 //burst_data[1], 4);
			 //Reset error
	ma120x0_clear_err(codec);
	if (ret < 0) return ret;

	// set serial audio format I2S and enable audio processor
	ret = snd_soc_write(codec, MA_i2s_format__a, 0x08);
	if (ret < 0) return ret;

	//Frame change on BCLK rising edge
	//ret = snd_soc_update_bits(codec, MA_i2s_sck_pol__a, MA_i2s_sck_pol__mask, 0x00);
	//if (ret < 0) return ret;

	// Enable audio limiter
	ret = snd_soc_update_bits(codec, MA_audio_proc_limiterEnable__a, MA_audio_proc_limiterEnable__mask, 0x40);
	if (ret < 0) return ret;

	// Set lim attack to fast
	ret = snd_soc_update_bits(codec, MA_audio_proc_attack__a, MA_audio_proc_attack__mask, 0x80);
	if (ret < 0) return ret;

	// Set lim attack to low
	ret = snd_soc_update_bits(codec, MA_audio_proc_release__a, MA_audio_proc_release__mask, 0x00);
	if (ret < 0) return ret;

	// set volume to xdB
	ret = snd_soc_write(codec, MA_vol_db_master__a, 0x33);
	if (ret < 0) return ret;

	// set ch0 lim tresh to 0dB
	ret = snd_soc_write(codec, MA_thr_db_ch0__a, 0x18);
	if (ret < 0) return ret;

	// set ch0 lim tresh to 0dB
	ret = snd_soc_write(codec, MA_thr_db_ch1__a, 0x18);
	if (ret < 0) return ret;

	//Check for errors
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x00, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x01, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x02, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x08, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x10, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x20, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x40, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, MA_error_acc__a, 0x80, 0);
	if (ret < 0) return ret;

	return 0;
}

static int ma120x0_probe(struct snd_soc_codec *codec)
{

#ifdef CONFIG_HAS_EARLYSUSPEND
	ma120x0->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ma120x0->early_suspend.suspend = ma120x0_early_suspend;
	ma120x0->early_suspend.resume = ma120x0_late_resume;
	ma120x0->early_suspend.param = codec;
	register_early_suspend(&(ma120x0->early_suspend));
#endif

ma120x0_init(codec);

	return 0;
}

static int ma120x0_remove(struct snd_soc_codec *codec)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct ma120x0_priv *ma120x0 = snd_soc_codec_get_drvdata(codec);

	unregister_early_suspend(&(ma120x0->early_suspend));
#endif

	return 0;
}

#ifdef CONFIG_PM
static int ma120x0_suspend(struct snd_soc_codec *codec)
{
	//struct ma120x0_priv *ma120x0 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "ma120x0_suspend!\n");

	/*save volume */
	//ma120x0->Ch1_vol = snd_soc_read(codec, DDX_CHANNEL1_VOL);
	//ma120x0->Ch2_vol = snd_soc_read(codec, DDX_CHANNEL2_VOL);
	//ma120x0->master_vol = snd_soc_read(codec, DDX_MASTER_VOLUME);
	ma120x0_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ma120x0_resume(struct snd_soc_codec *codec)
{
	//struct ma120x0_priv *ma120x0 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "ma120x0_resume!\n");

	ma120x0_init(codec);
	//snd_soc_write(codec, DDX_CHANNEL1_VOL, ma120x0->Ch1_vol);
	//snd_soc_write(codec, DDX_CHANNEL2_VOL, ma120x0->Ch2_vol);
	//snd_soc_write(codec, DDX_MASTER_VOLUME, ma120x0->master_vol);
	ma120x0_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
#else
#define ma120x0_suspend NULL
#define ma120x0_resume NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ma120x0_early_suspend(struct early_suspend *h)
{
}

static void ma120x0_late_resume(struct early_suspend *h)
{
}
#endif

/*
static const struct snd_soc_dapm_widget ma120x0_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "HIFI Playback", SND_SOC_NOPM, 0, 0),
};
*/

static const struct snd_soc_codec_driver soc_codec_dev_ma120x0 = {
	.probe = ma120x0_probe,
	.remove = ma120x0_remove,
	.suspend = ma120x0_suspend,
	.resume = ma120x0_resume,
	.set_bias_level = ma120x0_set_bias_level,
	.component_driver = {
		.controls = ma120x0_snd_controls,
		.num_controls = ARRAY_SIZE(ma120x0_snd_controls),
		//.dapm_widgets = ma120x0_dapm_widgets,
		//.num_dapm_widgets = ARRAY_SIZE(ma120x0_dapm_widgets),
	}
};

static const struct regmap_config ma120x0_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 255,
	.reg_defaults = ma120x0_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ma120x0_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

/*
static int ma120x0_parse_dt(
	struct ma120x0_priv *ma120x0,
	struct device_node *np)
{
	int ret = 0;
	int reset_pin = -1;

	reset_pin = of_get_named_gpio(np, "reset_pin", 0);
	if (reset_pin < 0) {
		pr_err("%s fail to get reset pin from dts!\n", __func__);
		ret = -1;
	} else {
		pr_info("%s pdata->reset_pin = %d!\n", __func__,
				ma120x0->pdata->reset_pin);
	}
	ma120x0->pdata->reset_pin = reset_pin;

	return ret;
}
*/

static int ma120x0_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct ma120x0_priv *ma120x0;
	struct ma120x0_platform_data *pdata;
	int ret;
	const char *codec_name;
	//int val = 1;

	ma120x0 = devm_kzalloc(&i2c->dev, sizeof(struct ma120x0_priv),
			       GFP_KERNEL);
	if (!ma120x0)
		return -ENOMEM;

	ma120x0->regmap = devm_regmap_init_i2c(i2c, &ma120x0_regmap);
	if (IS_ERR(ma120x0->regmap)) {
		ret = PTR_ERR(ma120x0->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	pdata = devm_kzalloc(&i2c->dev,
				sizeof(struct ma120x0_platform_data),
				GFP_KERNEL);
	if (!pdata) {
		pr_err("%s failed to kzalloc for ma120x0 pdata\n", __func__);
		return -ENOMEM;
	}
	ma120x0->pdata = pdata;

	//ma120x0_parse_dt(ma120x0, i2c->dev.of_node);

	if (of_property_read_string(i2c->dev.of_node,
			"codec_name",
				&codec_name)) {
		pr_info("no codec name\n");
		ret = -1;
	}
	pr_info("aux name = %s\n", codec_name);
	if (codec_name)
		dev_set_name(&i2c->dev, "%s", codec_name);

	i2c_set_clientdata(i2c, ma120x0);

	pr_info(KERN_INFO "registering codec\n" );

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ma120x0,
						 &ma120x0_dai, 1);

	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register codec (%d)\n", ret);
	} else {
		msleep(5000);
		ret = regmap_write(ma120x0->regmap, MA_vol_db_master__a, 0x33);
	}

	/*
	while (val != 0) {
		msleep(500);
		val = regmap_write(ma120x0->regmap, MA_vol_db_master__a, 0x33);
	}
	*/

	pr_info(KERN_INFO "register codec =(%d)\n",ret );

	return ret;

}

static int ma120x0_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id ma120x0_i2c_id[] = {
	{ "ma120x0", 0 },
	{}
};

static const struct of_device_id ma120x0_of_id[] = {
	{.compatible = "ma,ma120x0",},
	{ /* senitel */ }
};
MODULE_DEVICE_TABLE(of, ma120x0_of_id);

static struct i2c_driver ma120x0_i2c_driver = {
	.driver = {
		.name = DEV_NAME,
		.of_match_table = ma120x0_of_id,
		.owner = THIS_MODULE,
	},
	.probe = ma120x0_i2c_probe,
	.remove = ma120x0_i2c_remove,
	.id_table = ma120x0_i2c_id,
};
module_i2c_driver(ma120x0_i2c_driver);


MODULE_DESCRIPTION("ASoC MA120x0 driver");
MODULE_AUTHOR("MERUS AUDIO team");
MODULE_LICENSE("GPL");
