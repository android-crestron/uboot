/*
 * AMLOGIC lcd controller driver.
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
 * Author:  Tim Yao <timyao@amlogic.com>
 * Modify:  Evoke Zhang <evoke.zhang@amlogic.com>
 * Modify:  Chris Chen <chris_chen@jabil.com>
 */

#ifndef LEDOUTC_H
#define LEDOUTC_H

#include <common.h>
#include <linux/list.h>
#include <jabil/jbl_kpled.h>
#include <asm/arch/aml_lcd_gpio.h>

//**********************************************
//debug print define
//***********************************************
//#define LED_DEBUG_INFO

extern unsigned int led_print_flag;
extern void led_print(const char *fmt, ...);
//**********************************************
//global define
//***********************************************
#define KPLED_FIN_FREQ				(24 * 1000)
//***************************************

//****************************************//
// keypad led control
//****************************************//
#define KPLED_LEVEL_MAX_DFT			255
#define KPLED_LEVEL_MIN_DFT			10
#define KPLED_LEVEL_OFF				1

#define KPLED_LEVEL_MID_DFT			128
#define KPLED_LEVEL_MID_MAPPED_DFT	102

#define KPLED_LEVEL_DFT				128

typedef enum {
    KPLED_CTL_GPIO = 0,
    KPLED_CTL_PWM_NEGATIVE = 1,
    KPLED_CTL_PWM_POSITIVE = 2,
    KPLED_CTL_PWM_COMBO = 3,
    KPLED_CTL_EXTERN = 4,
    KPLED_CTL_MAX = 5,
} KPLED_Ctrl_Method_t;

typedef enum {
    KPLED_PWM_A = 0,
    KPLED_PWM_B,
    KPLED_PWM_C,
    KPLED_PWM_D,
    KPLED_PWM_MAX,
} KPLED_PWM_t;

#define LED_NUM_MAX		5
typedef struct {
    unsigned level_default;
    unsigned level_mid;
    unsigned level_mid_mapping;
    unsigned level_min;
    unsigned level_max;
    unsigned short power_on_delay;
    unsigned char method;
    int gpio[LED_NUM_MAX];
    unsigned dim_max;
    unsigned dim_min;
    unsigned char pwm_port;
    unsigned char pwm_gpio_used;
    unsigned pwm_cnt;
    unsigned pwm_pre_div;
    unsigned pwm_max;
    unsigned pwm_min;

    unsigned combo_level_switch;
    unsigned char combo_high_port;
    unsigned char combo_high_method;
    unsigned char combo_low_port;
    unsigned char combo_low_method;
    unsigned combo_high_cnt;
    unsigned combo_high_pre_div;
    unsigned combo_high_duty_max;
    unsigned combo_high_duty_min;
    unsigned combo_low_cnt;
    unsigned combo_low_pre_div;
    unsigned combo_low_duty_max;
    unsigned combo_low_duty_min;

    unsigned pinmux_set_num;
    unsigned pinmux_set[5][2];
    unsigned pinmux_clr_num;
    unsigned pinmux_clr[5][2];
} Keypad_Led_Config_t;

Keypad_Led_Config_t kpled_config_dft;
//*************************************//

extern void mdelay(unsigned long msec);

#endif /* LEDOUTC_H */
