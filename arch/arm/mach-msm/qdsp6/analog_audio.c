/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/gpio.h>
#include <mach/pmic.h>
#include <mach/msm_qdsp6_audio.h>
#ifdef CONFIG_AUDIO_TPA2018
#include <mach/tpa2018.h>
#endif
#ifdef CONFIG_ACER_HEADSET
#include <mach/acer_headset.h>
#endif
#ifdef CONFIG_AUDIO_FM2018
#include <mach/fm2018.h>
#endif

void analog_init(void)
{
	/* stereo pmic init */
	pmic_spkr_set_gain(LEFT_SPKR, SPKR_GAIN_PLUS12DB);
	pmic_spkr_set_gain(RIGHT_SPKR, SPKR_GAIN_PLUS12DB);
#ifdef CONFIG_MACH_ACER_A3
	pmic_mic_set_volt(MIC_VOLT_2_00V);
#else
	pmic_mic_set_volt(MIC_VOLT_1_80V);
#endif
}

void analog_headset_enable(int en)
{
	/* enable audio amp */
	if(en) {
		hs_amp(true);
		set_adie_flag(1);
		pr_info("[Audio] Enable HS \n");
	} else {
		hs_amp(false);
		set_adie_flag(0);
		pr_info("[Audio] Disable HS \n");
	}
}

void analog_speaker_enable(int en)
{
	struct spkr_config_mode scm;
	memset(&scm, 0, sizeof(scm));

	if (en) {
		scm.is_right_chan_en = 1;
		scm.is_left_chan_en = 1;
		scm.is_stereo_en = 1;
		scm.is_hpf_en = 1;
		pmic_spkr_en_mute(LEFT_SPKR, 0);
		pmic_spkr_en_mute(RIGHT_SPKR, 0);
		pmic_set_spkr_configuration(&scm);
		pmic_spkr_en(LEFT_SPKR, 1);
		pmic_spkr_en(RIGHT_SPKR, 1);

		set_adie_flag(1);
		spkr_amp(1);
		pr_info("[Audio] Enable Speaker AMP \n");
		/* unmute */
		pmic_spkr_en_mute(LEFT_SPKR, 1);
		pmic_spkr_en_mute(RIGHT_SPKR, 1);
	} else {
		pmic_spkr_en_mute(LEFT_SPKR, 0);
		pmic_spkr_en_mute(RIGHT_SPKR, 0);

		set_adie_flag(0);
		spkr_amp(0);
		pr_info("[Audio] Disable Speaker AMP \n");
		pmic_spkr_en(LEFT_SPKR, 0);
		pmic_spkr_en(RIGHT_SPKR, 0);

		pmic_set_spkr_configuration(&scm);
	}
}

void analog_mic_enable(int en)
{
	pmic_mic_en(en);
#ifdef CONFIG_AUDIO_FM2018
	if (hw_version <= 3) {
		pr_debug("### open fm2018 !!\n");
		fm2018_set_pwd(en);
		fm2018_set_procedure(1);
	}
#endif
}

void analog_back_mic_enable(int en)
{
	hs_mic_en(en);
}

void analog_ext_mic_enable(int en)
{
	/* hs_mic_en(en); */
}

void analog_amp_mute(int en)
{
	tpa2018_mute(en);
}

static struct q6audio_analog_ops ops = {
	.init = analog_init,
	.speaker_enable = analog_speaker_enable,
	.headset_enable = analog_headset_enable,
	.int_mic_enable = analog_mic_enable,
	.ext_mic_enable = analog_ext_mic_enable,
	.back_mic_enable = analog_back_mic_enable,
	.amp_mute = analog_amp_mute,
};

static int __init init(void)
{
	q6audio_register_analog_ops(&ops);
	return 0;
}

device_initcall(init);
