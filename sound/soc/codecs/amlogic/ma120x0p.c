#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
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
#include <linux/gpio/consumer.h>

#include "ma120x0p.h"

#define DEV_NAME	"ma120x0p"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static void ma120x0p_early_suspend(struct early_suspend *h);
static void ma120x0p_late_resume(struct early_suspend *h);
#endif

#define MA120X0P_RATES (SNDRV_PCM_RATE_44100 | \
		       SNDRV_PCM_RATE_48000 | \
				 	 SNDRV_PCM_RATE_88200 | \
			 	   SNDRV_PCM_RATE_96000)

#define MA120X0P_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE )

#define SOC_ENUM_ERR(xname, xenum)\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_READ,\
	.info = snd_soc_info_enum_double,\
	.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double,\
	.private_value = (unsigned long)&(xenum) }

/* Power-up register defaults */
struct reg_default ma120x0p_reg_defaults[] = {
	{	0x01,	0x3c	},
};

struct ma120x0p_platform_data {
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
struct ma120x0p_priv {
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct ma120x0p_platform_data *pdata;
	struct clk *clk_srcpll;
	struct clk *mclk_c;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *mute_gpio;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};
/*
 *    _   _    ___   _      ___         _           _
 *   /_\ | |  / __| /_\    / __|___ _ _| |_ _ _ ___| |___
 *  / _ \| |__\__ \/ _ \  | (__/ _ \ ' \  _| '_/ _ \ (_-<
 * /_/ \_\____|___/_/ \_\  \___\___/_||_\__|_| \___/_/__/
 *
 */
static const char * const limenable_text[] = {"Bypassed", "Enabled"};
static const char * const limatack_text[] = {"Slow", "Normal", "Fast"};
static const char * const limrelease_text[] = {"Slow", "Normal", "Fast"};
//static const char * const audioproc_mute_text[] = {"Play", "Mute"};

static const char * const err_flycap_text[] = {"Ok", "Error"};
static const char * const err_overcurr_text[] = {"Ok", "Error"};
static const char * const err_pllerr_text[] = {"Ok", "Error"};
static const char * const err_pvddunder_text[] = {"Ok", "Error"};
static const char * const err_overtempw_text[] = {"Ok", "Error"};
static const char * const err_overtempe_text[] = {"Ok", "Error"};
static const char * const err_pinlowimp_text[] = {"Ok", "Error"};
static const char * const err_dcprot_text[] = {"Ok", "Error"};

static const char * const pwr_mode_prof_text[] = {"PMF0", "PMF1", "PMF2",
"PMF3", "PMF4"};

static const struct soc_enum lim_enable_ctrl =
	SOC_ENUM_SINGLE(ma_audio_proc_limiterenable__a,
									ma_audio_proc_limiterenable__shift,
									ma_audio_proc_limiterenable__len + 1,
									limenable_text);
static const struct soc_enum limatack_ctrl =
	SOC_ENUM_SINGLE(ma_audio_proc_attack__a,
									ma_audio_proc_attack__shift,
									ma_audio_proc_attack__len + 1,
									limatack_text);
static const struct soc_enum limrelease_ctrl =
	SOC_ENUM_SINGLE(ma_audio_proc_release__a,
									ma_audio_proc_release__shift,
									ma_audio_proc_release__len + 1,
									limrelease_text);
static const struct soc_enum err_flycap_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 0, 3, err_flycap_text);
static const struct soc_enum err_overcurr_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 1, 3, err_overcurr_text);
static const struct soc_enum err_pllerr_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 2, 3, err_pllerr_text);
static const struct soc_enum err_pvddunder_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 3, 3, err_pvddunder_text);
static const struct soc_enum err_overtempw_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 4, 3, err_overtempw_text);
static const struct soc_enum err_overtempe_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 5, 3, err_overtempe_text);
static const struct soc_enum err_pinlowimp_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 6, 3, err_pinlowimp_text);
static const struct soc_enum err_dcprot_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 7, 3, err_dcprot_text);
static const struct soc_enum pwr_mode_prof_ctrl =
	SOC_ENUM_SINGLE(ma_pmprofile__a, ma_pmprofile__shift, 5,
									pwr_mode_prof_text);

static const char * const pwr_mode_texts[] = {
		"Dynamic power mode",
		"Power mode 1",
		"Power mode 2",
		"Power mode 3",
	};

static const int pwr_mode_values[] = {
		0x10,
		0x50,
		0x60,
		0x70,
	};

static SOC_VALUE_ENUM_SINGLE_DECL(pwr_mode_ctrl,
					  ma_pm_man__a, 0, 0x70,
					  pwr_mode_texts,
					  pwr_mode_values);

static const DECLARE_TLV_DB_SCALE(ma120x0p_vol_tlv, -5000, 100,  0);
static const DECLARE_TLV_DB_SCALE(ma120x0p_lim_tlv, -5000, 100,  0);
static const DECLARE_TLV_DB_SCALE(ma120x0p_lr_tlv, -5000, 100,  0);

static const struct snd_kcontrol_new ma120x0p_snd_controls[] = {
	//Master Volume
	SOC_SINGLE_RANGE_TLV("A.Mstr Vol Volume",
												ma_vol_db_master__a, 0, 0x18, 0x4a, 1,
												ma120x0p_vol_tlv),

	//L-R Volume ch0
	SOC_SINGLE_RANGE_TLV("B.L Vol Volume",
												ma_vol_db_ch0__a, 0, 0x18, 0x4a, 1,
												ma120x0p_lr_tlv),
	SOC_SINGLE_RANGE_TLV("C.R Vol Volume",
												ma_vol_db_ch1__a, 0, 0x18, 0x4a, 1,
												ma120x0p_lr_tlv),

	//L-R Limiter Threshold ch0-ch1
	SOC_DOUBLE_R_RANGE_TLV("D.Lim thresh Volume",
												 ma_thr_db_ch0__a, ma_thr_db_ch1__a,
												 0, 0x0e, 0x4a, 1, ma120x0p_lim_tlv),

	//Enum Switches/Selectors
	//SOC_ENUM("E.AudioProc Mute", audioproc_mute_ctrl),
	SOC_ENUM("F.Limiter Enable", lim_enable_ctrl),
	SOC_ENUM("G.Limiter Attck", limatack_ctrl),
	SOC_ENUM("H.Limiter Rls", limrelease_ctrl),

	//Enum Error Monitor (read-only)
	SOC_ENUM_ERR("I.Err flycap", err_flycap_ctrl),
	SOC_ENUM_ERR("J.Err overcurr", err_overcurr_ctrl),
	SOC_ENUM_ERR("K.Err pllerr", err_pllerr_ctrl),
	SOC_ENUM_ERR("L.Err pvddunder", err_pvddunder_ctrl),
	SOC_ENUM_ERR("M.Err overtempw", err_overtempw_ctrl),
	SOC_ENUM_ERR("N.Err overtempe", err_overtempe_ctrl),
	SOC_ENUM_ERR("O.Err pinlowimp", err_pinlowimp_ctrl),
	SOC_ENUM_ERR("P.Err dcprot", err_dcprot_ctrl),

	//Power modes profiles
	SOC_ENUM("Q.PM Prof", pwr_mode_prof_ctrl),

	// Power mode selection (Dynamic,1,2,3)
	SOC_ENUM("R.Power Mode", pwr_mode_ctrl),
};

static int ma120x0p_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	u16 blen = 0x00;

	struct snd_soc_codec *codec = dai->codec;
	//struct ma120x0_priv *ma120x0p = snd_soc_codec_get_drvdata(codec);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		blen = 0x00;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		blen = 0x00;
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
	snd_soc_update_bits(codec, ma_i2s_framesize__a, ma_i2s_framesize__mask, blen);

	return 0;
}

static int ma120x0p_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	int val = 0;

	struct ma120x0p_priv *ma120x0p;

	struct snd_soc_codec *codec = dai->codec;

	ma120x0p = snd_soc_codec_get_drvdata(codec);

	if (mute)
		val = 1;
	else
		val = 0;

	snd_soc_update_bits(codec, ma_audio_proc_mute__a, ma_audio_proc_mute__mask, val);

	return 0;
}

static int ma120x0p_set_bias_level(struct snd_soc_codec *codec,
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

static const struct snd_soc_dai_ops ma120x0p_dai_ops = {
	.hw_params = ma120x0p_hw_params,
	.mute_stream = ma120x0p_mute_stream,
};

static struct snd_soc_dai_driver ma120x0p_dai = {
	.name = DEV_NAME,
	.playback = {
		.stream_name = "HIFI Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MA120X0P_RATES,
		.formats = MA120X0P_FORMATS,
	},
	.ops = &ma120x0p_dai_ops,
};

/*
static int reset_ma120x0_GPIO(struct snd_soc_codec *codec)
{
	struct ma120x0_priv *ma120x0p = snd_soc_codec_get_drvdata(codec);
	struct tas57xx_platform_data *pdata = ma120x0p->pdata;
	int ret = 0;

	if (pdata->reset_pin < 0)
		return 0;

	ret = devm_gpio_request_one(codec->dev, pdata->reset_pin,
					    GPIOF_OUT_INIT_LOW,
					    "ma120x0p-reset-pin");
	if (ret < 0)
		return -1;

	gpio_direction_output(pdata->reset_pin, GPIOF_OUT_INIT_LOW);
	udelay(1000);
	gpio_direction_output(pdata->reset_pin, GPIOF_OUT_INIT_HIGH);
	mdelay(15);

	return 0;
}
*/

static int ma120x0p_clear_err(struct snd_soc_codec *codec)
{
 int ret = 0;

 //struct ma120x0_priv *ma120x0p;
 //ma120x0p = snd_soc_component_get_drvdata(component);

 ret = snd_soc_update_bits(codec, ma_eh_clear__a, ma_eh_clear__mask, 0x00);
 if (ret < 0) return ret;

 ret = snd_soc_update_bits(codec, ma_eh_clear__a, ma_eh_clear__mask, 0x04);
 if (ret < 0) return ret;

 ret = snd_soc_update_bits(codec, ma_eh_clear__a, ma_eh_clear__mask, 0x00);
 if (ret < 0) return ret;

 return 0;
}

static int ma120x0p_init(struct snd_soc_codec *codec)
{
	//struct ma120x0_priv *ma120x0p = snd_soc_codec_get_drvdata(codec);
  int ret = 0;

	dev_info(codec->dev, "ma120x0p_init!\n");
	//snd_soc_write(codec, DDX_OSC_TRIM, 0x00);
	msleep(50);

	ma120x0p_clear_err(codec);
	if (ret < 0) return ret;

	// set serial audio format I2S and enable audio processor
	ret = snd_soc_write(codec, ma_i2s_format__a, 0x08);
	if (ret < 0) return ret;

	// Enable audio limiter
	ret = snd_soc_update_bits(codec, ma_audio_proc_limiterenable__a, ma_audio_proc_limiterenable__mask, 0x40);
	if (ret < 0) return ret;

	// Set lim attack to fast
	ret = snd_soc_update_bits(codec, ma_audio_proc_attack__a, ma_audio_proc_attack__mask, 0x80);
	if (ret < 0) return ret;

	// Set lim attack to low
	ret = snd_soc_update_bits(codec, ma_audio_proc_release__a, ma_audio_proc_release__mask, 0x00);
	if (ret < 0) return ret;

	// set volume to xdB
	ret = snd_soc_write(codec, ma_vol_db_master__a, 0x33);
	if (ret < 0) return ret;

	// set ch0 lim tresh to 0dB
	ret = snd_soc_write(codec, ma_thr_db_ch0__a, 0x18);
	if (ret < 0) return ret;

	// set ch0 lim tresh to 0dB
	ret = snd_soc_write(codec, ma_thr_db_ch1__a, 0x18);
	if (ret < 0) return ret;

	//Check for errors
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x00, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x01, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x02, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x08, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x10, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x20, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x40, 0);
	if (ret < 0) return ret;
	ret = snd_soc_test_bits(codec, ma_error_acc__a, 0x80, 0);
	if (ret < 0) return ret;

	return 0;
}

static int ma120x0p_probe(struct snd_soc_codec *codec)
{

#ifdef CONFIG_HAS_EARLYSUSPEND
	ma120x0p->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ma120x0p->early_suspend.suspend = ma120x0_early_suspend;
	ma120x0p->early_suspend.resume = ma120x0_late_resume;
	ma120x0p->early_suspend.param = codec;
	register_early_suspend(&(ma120x0p->early_suspend));
#endif

ma120x0p_init(codec);

	return 0;
}

static int ma120x0p_remove(struct snd_soc_codec *codec)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct ma120x0p_priv *ma120x0p = snd_soc_codec_get_drvdata(codec);

	unregister_early_suspend(&(ma120x0p->early_suspend));
#endif

	return 0;
}

#ifdef CONFIG_PM
static int ma120x0p_suspend(struct snd_soc_codec *codec)
{
	//struct ma120x0_priv *ma120x0p = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "ma120x0p_suspend!\n");

	/*save volume */
	//ma120x0p->Ch1_vol = snd_soc_read(codec, DDX_CHANNEL1_VOL);
	//ma120x0p->Ch2_vol = snd_soc_read(codec, DDX_CHANNEL2_VOL);
	//ma120x0p->master_vol = snd_soc_read(codec, DDX_MASTER_VOLUME);
	ma120x0p_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ma120x0p_resume(struct snd_soc_codec *codec)
{
	//struct ma120x0_priv *ma120x0p = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "ma120x0p_resume!\n");

	ma120x0p_init(codec);
	//snd_soc_write(codec, DDX_CHANNEL1_VOL, ma120x0p->Ch1_vol);
	//snd_soc_write(codec, DDX_CHANNEL2_VOL, ma120x0p->Ch2_vol);
	//snd_soc_write(codec, DDX_MASTER_VOLUME, ma120x0p->master_vol);
	ma120x0p_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
#else
#define ma120x0p_suspend NULL
#define ma120x0p_resume NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ma120x0p_early_suspend(struct early_suspend *h)
{
}

static void ma120x0p_late_resume(struct early_suspend *h)
{
}
#endif

/*
static const struct snd_soc_dapm_widget ma120x0_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "HIFI Playback", SND_SOC_NOPM, 0, 0),
};
*/

static const struct snd_soc_codec_driver soc_codec_dev_ma120x0p = {
	.probe = ma120x0p_probe,
	.remove = ma120x0p_remove,
	.suspend = ma120x0p_suspend,
	.resume = ma120x0p_resume,
	.set_bias_level = ma120x0p_set_bias_level,
	.component_driver = {
		.controls = ma120x0p_snd_controls,
		.num_controls = ARRAY_SIZE(ma120x0p_snd_controls),
		//.dapm_widgets = ma120x0_dapm_widgets,
		//.num_dapm_widgets = ARRAY_SIZE(ma120x0_dapm_widgets),
	}
};

static const struct regmap_config ma120x0p_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 255,
	.reg_defaults = ma120x0p_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ma120x0p_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

/*
static int ma120x0_parse_dt(
	struct ma120x0_priv *ma120x0p,
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
				ma120x0p->pdata->reset_pin);
	}
	ma120x0p->pdata->reset_pin = reset_pin;

	return ret;
}
*/

static int ma120x0p_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct ma120x0p_priv *ma120x0p;
	struct ma120x0p_platform_data *pdata;
	int ret;
	const char *codec_name;
	int mclk_rate_c;
	int clk_srcpll_rate;

	//int val = 1;

	ma120x0p = devm_kzalloc(&i2c->dev, sizeof(struct ma120x0p_priv),
			       GFP_KERNEL);
	if (!ma120x0p)
		return -ENOMEM;

	//Mute ma120x0p
	ma120x0p->mute_gpio = devm_gpiod_get(&i2c->dev, "mute",
																							GPIOD_OUT_LOW);
	if (IS_ERR(ma120x0p->mute_gpio)) {
		ret = PTR_ERR(ma120x0p->mute_gpio);
		dev_err(&i2c->dev, "Failed to get ma120x0p mute gpio line: %d\n", ret);
		return ret;
	}
	msleep(500);

	//Disable ma120x0p
	ma120x0p->enable_gpio = devm_gpiod_get(&i2c->dev, "enable",
																						GPIOD_OUT_HIGH);
	if (IS_ERR(ma120x0p->enable_gpio)) {
		ret = PTR_ERR(ma120x0p->enable_gpio);
		dev_err(&i2c->dev, "Failed to get ma120x0p enable gpio line: %d\n", ret);
		return ret;
	}
	msleep(500);

	ma120x0p->regmap = devm_regmap_init_i2c(i2c, &ma120x0p_regmap);
	if (IS_ERR(ma120x0p->regmap)) {
		ret = PTR_ERR(ma120x0p->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	pdata = devm_kzalloc(&i2c->dev,
				sizeof(struct ma120x0p_platform_data),
				GFP_KERNEL);
	if (!pdata) {
		pr_err("%s failed to kzalloc for ma120x0p pdata\n", __func__);
		return -ENOMEM;
	}
	ma120x0p->pdata = pdata;

	//ma120x0_parse_dt(ma120x0p, i2c->dev.of_node);

	if (of_property_read_string(i2c->dev.of_node,
			"codec_name",
				&codec_name)) {
		pr_info("no codec name\n");
		ret = -1;
	}
	pr_info("aux name = %s\n", codec_name);
	if (codec_name)
		dev_set_name(&i2c->dev, "%s", codec_name);

	i2c_set_clientdata(i2c, ma120x0p);

 pr_info(KERN_INFO "geting clock......\n" );

 ma120x0p->clk_srcpll = devm_clk_get(&i2c->dev, "clk_srcpll");
 if (IS_ERR(ma120x0p->clk_srcpll)) {
 	dev_err(&i2c->dev, "Can't retrieve mpll2 clock\n");
 	return PTR_ERR(ma120x0p->clk_srcpll);
 }

 ma120x0p->mclk_c = devm_clk_get(&i2c->dev, "mclk_c");
 if (IS_ERR(ma120x0p->mclk_c)) {
	 dev_err(&i2c->dev, "Can't retrieve mclk\n");
	 return PTR_ERR(ma120x0p->mclk_c);
 }

 //clk_set_parent(ma120x0p->mclk, ma120x0p->clk_srcpll);

 clk_set_rate(ma120x0p->clk_srcpll, 24576000 * 20);
 clk_set_rate(ma120x0p->mclk_c, 24576000);

 ret = clk_prepare_enable(ma120x0p->mclk_c);
 if (ret) {
 	dev_err(&i2c->dev, "Error enabling master clock c %d\n", ret);
 	return ret;
 }

	ret = clk_prepare_enable(ma120x0p->clk_srcpll);
	if (ret) {
		dev_err(&i2c->dev, "Error enabling master clock %d\n", ret);
		return ret;
	}
	msleep(300);

	pr_info(KERN_INFO " clk prepare enable = %d\n",ret );

	mclk_rate_c = clk_get_rate(ma120x0p->mclk_c);
	clk_srcpll_rate = clk_get_rate(ma120x0p->clk_srcpll);

	pr_info(KERN_INFO "mclk rate is (%d)\n", mclk_rate_c);
	pr_info(KERN_INFO "clk_srcpll rate (%d)\n", clk_srcpll_rate);

	//Enable ma120x0p
	gpiod_set_value_cansleep(ma120x0p->enable_gpio, 0);
	msleep(300);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ma120x0p,
						 &ma120x0p_dai, 1);
	if (ret != 0)
		dev_err(&i2c->dev, "Failed to register codec (%d)\n", ret);

	msleep(5000);
	ret = regmap_write(ma120x0p->regmap, ma_vol_db_master__a, 0x33);

	if (ret != 0)
		return -EPROBE_DEFER;

	pr_info(KERN_INFO "register codec =(%d)\n",ret );

	//Unmute ma120x0p
	gpiod_set_value_cansleep(ma120x0p->mute_gpio, 1);

	return ret;
}

static int ma120x0p_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id ma120x0p_i2c_id[] = {
	{ "ma120x0p", 0 },
	{}
};

static const struct of_device_id ma120x0p_of_id[] = {
	{.compatible = "ma,ma120x0p",},
	{ /* senitel */ }
};
MODULE_DEVICE_TABLE(of, ma120x0p_of_id);

static struct i2c_driver ma120x0p_i2c_driver = {
	.driver = {
		.name = DEV_NAME,
		.of_match_table = ma120x0p_of_id,
		.owner = THIS_MODULE,
	},
	.probe = ma120x0p_i2c_probe,
	.remove = ma120x0p_i2c_remove,
	.id_table = ma120x0p_i2c_id,
};
module_i2c_driver(ma120x0p_i2c_driver);

MODULE_DESCRIPTION("ASoC MA120x0P driver");
MODULE_AUTHOR("MERUS AUDIO team");
MODULE_LICENSE("GPL");
