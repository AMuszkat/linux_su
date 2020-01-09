#ifndef __AML_AUDIO_CLOCKS_H_
#define __AML_AUDIO_CLOCKS_H_

unsigned int aml_mpll_mclk_ratio(unsigned int freq);
unsigned int aml_hifipll_mclk_ratio(unsigned int freq);

unsigned int aml_get_mclk_rate(unsigned int rate);

#endif /* __AML_AUDIO_CLOCKS_H_ */
