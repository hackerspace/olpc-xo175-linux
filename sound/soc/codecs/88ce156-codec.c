#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <sound/soc.h>


struct ce156_private {
	struct regmap *regmap;
};





static inline int
meh_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	//struct ce156_private *ce156_priv = context;

	*val = 0x666;
	printk("XXX READ 0x%08x -> 0x%08x\n", reg, *val);
	return 0;
}

static inline int
meh_reg_write(void *context, unsigned int reg, unsigned int val)
{
	//struct ce156_private *ce156_priv = context;

	printk("XXX WRITE 0x%08x <- 0x%08x\n", reg, val);
	return 0;
}


static const struct regmap_config ce156_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x9f, // XXX
	//.cache_type = REGCACHE_RBTREE,
	//.reg_defaults = ce156_reg_defaults,
	//.num_reg_defaults = ARRAY_SIZE(ce156_reg_defaults),

        .reg_read = meh_reg_read,
        .reg_write = meh_reg_write
};




static const struct snd_kcontrol_new ce156_snd_controls[] = {
	// #define SOC_SINGLE(xname, reg, shift, max, invert)
	SOC_SINGLE("Some Switch 0", 0x01, 1, 1, 0),
	SOC_SINGLE("Some Switch 1", 0x02, 1, 1, 0),
	SOC_SINGLE("Master Capture Switch", 0x03, 1, 1, 0),
};

static const struct snd_soc_dapm_widget ce156_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1N"),
	SND_SOC_DAPM_INPUT("MIC1P"),

	SND_SOC_DAPM_MIXER("Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("SPKLN"),
	SND_SOC_DAPM_OUTPUT("SPKLP"),
	SND_SOC_DAPM_OUTPUT("SPKRN"),
	SND_SOC_DAPM_OUTPUT("SPKRP"),
};

static const struct snd_soc_dapm_route ce156_dapm_routes[] = {
	{"Mixer", NULL, "MIC1N"},
	{"Mixer", NULL, "MIC1P"},

	{"HPL", NULL, "Mixer"},
	{"HPR", NULL, "Mixer"},
	{"SPKLN", NULL, "Mixer"},
	{"SPKLP", NULL, "Mixer"},
	{"SPKRN", NULL, "Mixer"},
	{"SPKRP", NULL, "Mixer"},
};






static int ce156_probe(struct snd_soc_component *component)
{
	printk("XXX F %s\n", __func__);
	return 0;
}




static void ce156_remove(struct snd_soc_component *component)
{
	//set_bias_level(component, SND_SOC_BIAS_OFF);
	printk("XXX F %s\n", __func__);
}




static int ce156_suspend(struct snd_soc_component *component)
{
	//set_bias_level(component, SND_SOC_BIAS_OFF);

	printk("XXX F %s\n", __func__);
	return 0;
}




static int ce156_resume(struct snd_soc_component *component)
{
	
	//set_bias_level(component, SND_SOC_BIAS_STANDBY);

	printk("XXX F %s\n", __func__);
	return 0;
}



static int set_bias_level(struct snd_soc_component *component, enum snd_soc_bias_level level)
{
	printk("XXX F %s\n", __func__);
	return 0;
}





struct snd_soc_component_driver soc_component_dev_ce156 = {
	.probe   = ce156_probe,
	.remove  = ce156_remove,
	.suspend = ce156_suspend,
	.resume  = ce156_resume,
	.set_bias_level = set_bias_level,

	.controls               = ce156_snd_controls,
        .num_controls           = ARRAY_SIZE(ce156_snd_controls),
        .dapm_widgets           = ce156_dapm_widgets,
        .num_dapm_widgets       = ARRAY_SIZE(ce156_dapm_widgets),
        .dapm_routes            = ce156_dapm_routes,
        .num_dapm_routes        = ARRAY_SIZE(ce156_dapm_routes),
};









#define CE156_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define CE156_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)



static int ce156_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	printk("XXX F %s\n", __func__);
	return 0;
}

static int ce156_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	printk("XXX F %s\n", __func__);
	return 0;
}

static int ce156_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	printk("XXX F %s\n", __func__);
	return 0;
}


static int ce156_set_dai_sysclk(struct snd_soc_dai * codec_dai, int clk_id, unsigned int freq, int dir)
{
	printk("XXX F %s\n", __func__);
	return 0;
}


static int ce156_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
{
	printk("XXX F %s\n", __func__);
	return 0;
}


static struct snd_soc_dai_ops ce156_dai_ops= {
	.hw_params	= ce156_hw_params,
	.mute_stream	= ce156_mute,
	.set_fmt	= ce156_set_dai_fmt,
	.set_sysclk	= ce156_set_dai_sysclk,
	.set_pll	= ce156_set_dai_pll,
	.no_capture_mute = 1,
};


struct snd_soc_dai_driver ce156_dai = {
	/* hifi codec dai */
	.name = "CE156",
	.id = 1,
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates	      = CE156_RATES,
		.formats      = CE156_FORMATS,
	},
	.capture = {
		.stream_name  = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = CE156_RATES,
		.formats      = CE156_FORMATS,
	},
	.ops = &ce156_dai_ops,
	.symmetric_rates = 1,
};



static int ce156_i2c_probe(struct i2c_client *i2c)
{
	struct ce156_private *ce156_priv;
	int ret;
	
	printk(KERN_INFO "CE156: register i2c driver successfully\n");
	
	ce156_priv = devm_kzalloc(&i2c->dev, sizeof(struct ce156_private), GFP_KERNEL);
	if (ce156_priv == NULL)
		return -ENOMEM;

#if 0
	ce156_priv->regmap = devm_regmap_init_i2c(i2c, &ce156_regmap);
#else
	ce156_priv->regmap = devm_regmap_init(&i2c->dev, NULL, ce156_priv, &ce156_regmap);
#endif
	if (IS_ERR(ce156_priv->regmap)) {
		ret = PTR_ERR(ce156_priv->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap (%d)\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_component(&i2c->dev, &soc_component_dev_ce156, &ce156_dai, 1);
		
	if (ret) {
		printk(KERN_INFO "CE156: Failed to register codec\n");
		goto out;
	}
		
	return ret;
	
out:
	kfree(ce156_priv);
	return -EINVAL;
}






static const struct of_device_id ce156_i2c_dt_ids[] = {
	{ .compatible = "marvell,88ce156"},
	{ }
};
MODULE_DEVICE_TABLE(of, ce156_i2c_dt_ids);

static struct i2c_driver ce156_i2c_driver = {
	.driver = {
		.name  = "88ce156",
		.of_match_table = of_match_ptr(ce156_i2c_dt_ids),
	},
	.probe_new = ce156_i2c_probe,
};
module_i2c_driver(ce156_i2c_driver);



MODULE_LICENSE("GPL");
