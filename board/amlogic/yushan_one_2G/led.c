/*
 * AMLOGIC LCD panel parameter.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Evoke Zhang <evoke.zhang@amlogic.com>
 * Modify:  Chris Chen <chris_chen@jabil.com>
 *
 */

#include <jabil/ledoutc.h>

//**********************************************//
// keypad led control
//*********************************************//
#define KPLED_LEVEL_DEFAULT		128	/** default brightness level */
#define KPLED_LEVEL_MID			102	/** brightness middle level*/
#define KPLED_LEVEL_MID_MAPPING	70	/** brightness middle level mapping to a new level*/
#define KPLED_LEVEL_MAX			255	/** brightness level max, must match the rootfs setting*/
#define KPLED_LEVEL_MIN			10	/** brightness level min, must match the rootfs setting*/

//**** define keypad led control method ***//
#define KPLED_POWER_ON_DELAY	200	/** delay time before led power on(unit: ms) */
#define KPLED_CTL				KPLED_CTL_PWM_POSITIVE	/** led control method(KPLED_CTL_GPIO, KPLED_CTL_PWM_NEGATIVE, KPLED_CTL_PWM_POSITIVE, KPLED_CTL_EXTERN) */
#define KPLED_GPIO_KEY1			GPIOX_16	/** key1 LED control gpio port */
#define KPLED_GPIO_KEY2			GPIOX_17	/** key2 LED control gpio port */
#define KPLED_GPIO_KEY3			GPIOX_10	/** key3 LED control gpio port */
#define KPLED_GPIO_KEY4			GPIOX_19	/** key4 LED control gpio port */
#define KPLED_GPIO_KEY5			GPIOX_11	/** key5 LED control gpio port */

//**** define keypad led GPIO control ***//
#define	KPLED_DIM_MAX			0x0	/** brightness diming level_max, negative logic */
#define	KPLED_DIM_MIN			0xf	/** brightness diming level_min, negative logic */

//**** define keypad led PWM control ***//
#define KPLED_PWM_PORT			KPLED_PWM_C	/** pwm port name(KPLED_PWM_A, KPLED_PWM_B, KPLED_PWM_C, KPLED_PWM_D) */
#define KPLED_PWM_USE_GPIO		1			/** pwm gpio used(0=use pwm_port only, 1=use bl_gpio_port to control on/off) */

#define	KPLED_PWM_FREQ			300000	/** led control pwm frequency(unit: Hz) */
#define KPLED_PWM_MAX			100		/** brightness diminig duty_max(unit: %, positive logic) */
#define KPLED_PWM_MIN			20		/** brightness diminig duty_min(unit: %, positive logic) */

//**** keypad led PWM pinmux setting ***//
const static unsigned kpled_pwm_pinmux_set[][2] = {{3, 0x2000000},};
const static unsigned kpled_pwm_pinmux_clr[][2] = {};
//*********************************************//

//**********************************************//

//*********************************************//

//*********************************************//
// led parameter API struct, DO NOT modify them!!
//*********************************************//
Keypad_Led_Config_t kpled_config_dft = {
    .level_default = KPLED_LEVEL_DEFAULT,
    .level_mid = KPLED_LEVEL_MID,
    .level_mid_mapping = KPLED_LEVEL_MID_MAPPING,
    .level_min = KPLED_LEVEL_MIN,
    .level_max = KPLED_LEVEL_MAX,
    .power_on_delay = KPLED_POWER_ON_DELAY,
    .method = KPLED_CTL,
    .gpio[0] = KPLED_GPIO_KEY1,
    .gpio[1] = KPLED_GPIO_KEY2,
    .gpio[2] = KPLED_GPIO_KEY3,
    .gpio[3] = KPLED_GPIO_KEY4,
    .gpio[4] = KPLED_GPIO_KEY5,
    .dim_max = KPLED_DIM_MAX,
    .dim_min = KPLED_DIM_MIN,
    .pwm_port = KPLED_PWM_PORT,
    .pwm_gpio_used = KPLED_PWM_USE_GPIO,
};

void kpled_default_config_init(Keypad_Led_Config_t *kpled_config)
{
    int i;
    unsigned pwm_freq, pwm_cnt, pwm_pre_div;

    pwm_freq = ((KPLED_PWM_FREQ >= (KPLED_FIN_FREQ * 500)) ? (KPLED_FIN_FREQ * 500) : KPLED_PWM_FREQ);
    
    for (i=0; i<0x7f; i++) {
        pwm_pre_div = i;
        pwm_cnt = KPLED_FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
        if (pwm_cnt <= 0xffff)
            break;
    }

    kpled_config->pwm_cnt = pwm_cnt;
    kpled_config->pwm_pre_div = pwm_pre_div;
    kpled_config->pwm_max = pwm_cnt * KPLED_PWM_MAX / 100;
    kpled_config->pwm_min = pwm_cnt * KPLED_PWM_MIN / 100;

    kpled_config->pinmux_set_num = ARRAY_SIZE(kpled_pwm_pinmux_set);
    kpled_config->pinmux_clr_num = ARRAY_SIZE(kpled_pwm_pinmux_clr);

    for (i=0; i<kpled_config->pinmux_set_num; i++) {
        kpled_config->pinmux_set[i][0] = kpled_pwm_pinmux_set[i][0];
        kpled_config->pinmux_set[i][1] = kpled_pwm_pinmux_set[i][1];
    }

    for (i=0; i<kpled_config->pinmux_clr_num; i++) {
        kpled_config->pinmux_clr[i][0] = kpled_pwm_pinmux_clr[i][0];
        kpled_config->pinmux_clr[i][1] = kpled_pwm_pinmux_clr[i][1];
    }
}
