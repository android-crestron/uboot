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
 * Author:  Elvis Yu <elvis.yu@amlogic.com>
 *
 * Modify:  Evoke Zhang <evoke.zhang@amlogic.com>
 * Modify:  Chris Chen <chris_chen@jabil.com>
 * compatible dts
 *
 */
#include <common.h>
#include <malloc.h>
#include <asm/arch/io.h>
#include <asm/arch/vinfo.h>
#include <asm/arch/lcdoutc.h>
#ifdef CONFIG_LCD_TYPE_MID_VALID
#include <amlogic/aml_lcd.h>
#include <amlogic/vinfo.h>
#include <amlogic/lcdoutc.h>
#include <jabil/ledoutc.h>
#include <asm/arch/clock.h>
#include <asm/arch/timing.h>
#include <asm/arch/lcd_reg.h>
#ifdef CONFIG_OF_LIBFDT
#include <libfdt.h>
#endif
#ifdef CONFIG_AML_LCD_EXTERN
#include <amlogic/aml_lcd_extern.h>
#endif
#ifdef CONFIG_PLATFORM_HAS_PMU
#include <amlogic/aml_pmu_common.h>
#define BATTERY_LOW_THRESHOLD       20
#endif

extern void kpled_default_config_init(Keypad_Led_Config_t *pConf);

unsigned int led_print_flag = 0;
unsigned int led_debug_flag = 0;
void led_print(const char *fmt, ...)
{
	va_list args;
	char printbuffer[CONFIG_SYS_PBSIZE];

	if (led_print_flag == 0)
		return;

	va_start(args, fmt);
	vsprintf(printbuffer, fmt, args);
	va_end(args);

	puts(printbuffer);
}

#ifdef CONFIG_OF_LIBFDT
static const char* led_ctrl_method_table[]={
    "gpio",
    "pwm_negative",
    "pwm_positive",
    "pwm_combo",
    "extern",
    "null",
};
#endif

typedef struct {
    Keypad_Led_Config_t *led_config;
} led_dev_t;

static led_dev_t *pDev = NULL;
#ifdef CONFIG_OF_LIBFDT
static char * dt_addr;
#endif
static int dts_ready = 0;

static unsigned led_level;

static Keypad_Led_Config_t led_config = {
    .level_default = KPLED_LEVEL_DFT,
    .level_mid = KPLED_LEVEL_MID_DFT,
    .level_mid_mapping = KPLED_LEVEL_MID_MAPPED_DFT,
    .level_min = KPLED_LEVEL_MIN_DFT,
    .level_max = KPLED_LEVEL_MAX_DFT,
};

static void keypad_led_power_ctrl(Bool_t status)
{
    int i;

    if( status == ON ) {
        mdelay(pDev->led_config->power_on_delay);

        switch (pDev->led_config->method) {
            case KPLED_CTL_GPIO:
                //TBD
                break;
            case KPLED_CTL_PWM_NEGATIVE:
            case KPLED_CTL_PWM_POSITIVE:
                switch (pDev->led_config->pwm_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, (READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~(0x7f<<8)) | ((1 << 15) | (pDev->led_config->pwm_pre_div<<8) | (1<<0)));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, (READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~(0x7f<<16)) | ((1 << 23) | (pDev->led_config->pwm_pre_div<<16) | (1<<1)));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, (READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~(0x7f<<8)) | ((1 << 15) | (pDev->led_config->pwm_pre_div<<8) | (1<<0)));  //enable pwm clk & pwm output
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, (READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~(0x7f<<16)) | ((1 << 23) | (pDev->led_config->pwm_pre_div<<16) | (1<<1)));  //enable pwm clk & pwm output
                        break;
                    default:
                        break;
                }
                for (i=0; i<pDev->led_config->pinmux_clr_num; i++) {
                    aml_lcd_pinmux_clr(pDev->led_config->pinmux_clr[i][0], pDev->led_config->pinmux_clr[i][1]);
                }
                for (i=0; i<pDev->led_config->pinmux_set_num; i++) {
                    aml_lcd_pinmux_set(pDev->led_config->pinmux_set[i][0], pDev->led_config->pinmux_set[i][1]);
                }
                mdelay(20);
                if (pDev->led_config->pwm_gpio_used)
                {
                    for (i=0; i<LED_NUM_MAX; i++) {
                        aml_lcd_gpio_set(pDev->led_config->gpio[i], LCD_POWER_GPIO_OUTPUT_LOW);
                        led_print("led gpio=%d on\n", pDev->led_config->gpio[i]);
                    }
                }
                break;
            case KPLED_CTL_PWM_COMBO:
                switch (pDev->led_config->combo_high_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, (READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~(0x7f<<8)) | ((1 << 15) | (pDev->led_config->combo_high_pre_div<<8) | (1<<0)));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, (READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~(0x7f<<16)) | ((1 << 23) | (pDev->led_config->combo_high_pre_div<<16) | (1<<1)));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, (READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~(0x7f<<8)) | ((1 << 15) | (pDev->led_config->combo_high_pre_div<<8) | (1<<0)));  //enable pwm clk & pwm output
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, (READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~(0x7f<<16)) | ((1 << 23) | (pDev->led_config->combo_high_pre_div<<16) | (1<<1)));  //enable pwm clk & pwm output
                        break;
                    default:
                        break;
                }
                switch (pDev->led_config->combo_low_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, (READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~(0x7f<<8)) | ((1 << 15) | (pDev->led_config->combo_low_pre_div<<8) | (1<<0)));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, (READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~(0x7f<<16)) | ((1 << 23) | (pDev->led_config->combo_low_pre_div<<16) | (1<<1)));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, (READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~(0x7f<<8)) | ((1 << 15) | (pDev->led_config->combo_low_pre_div<<8) | (1<<0)));  //enable pwm clk & pwm output
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, (READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~(0x7f<<16)) | ((1 << 23) | (pDev->led_config->combo_low_pre_div<<16) | (1<<1)));  //enable pwm clk & pwm output
                        break;
                    default:
                        break;
                }
                for (i=0; i<pDev->led_config->pinmux_clr_num; i++) {
                    aml_lcd_pinmux_clr(pDev->led_config->pinmux_clr[i][0], pDev->led_config->pinmux_clr[i][1]);
                }
                for (i=0; i<pDev->led_config->pinmux_set_num; i++) {
                    aml_lcd_pinmux_set(pDev->led_config->pinmux_set[i][0], pDev->led_config->pinmux_set[i][1]);
                }
                break;
            case KPLED_CTL_EXTERN:
                //TBD
                break;
            default:
                printf("Wrong led control method\n");
                break;
        }
    }
    else {
        switch (pDev->led_config->method) {
            case KPLED_CTL_GPIO:
                //TBD
                break;
            case KPLED_CTL_PWM_NEGATIVE:
            case KPLED_CTL_PWM_POSITIVE:
                if (pDev->led_config->pwm_gpio_used) {
                    for (i=0; i<LED_NUM_MAX; i++)
                    {
                        if (pDev->led_config->gpio[i]) {
                            aml_lcd_gpio_set(pDev->led_config->gpio[i], LCD_POWER_GPIO_OUTPUT_HIGH);
                            led_print("led gpio=%d off\n", pDev->led_config->gpio[i]);
                        }
                    }
                }
                switch (pDev->led_config->pwm_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~((1 << 15) | (1<<0)));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~((1 << 23) | (1<<1)));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~((1 << 15) | (1<<0)));  //disable pwm_clk & pwm port
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~((1 << 23) | (1<<1)));  //disable pwm_clk & pwm port
                        break;
                    default:
                        break;
                }
                break;
            case KPLED_CTL_PWM_COMBO:
                switch (pDev->led_config->combo_high_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~((1 << 15) | (1<<0)));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~((1 << 23) | (1<<1)));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~((1 << 15) | (1<<0)));  //disable pwm_clk & pwm port
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~((1 << 23) | (1<<1)));  //disable pwm_clk & pwm port
                        break;
                    default:
                        break;
                }
                switch (pDev->led_config->combo_low_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~((1 << 15) | (1<<0)));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_AB, READ_LCD_CBUS_REG(PWM_MISC_REG_AB) & ~((1 << 23) | (1<<1)));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~((1 << 15) | (1<<0)));  //disable pwm_clk & pwm port
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_MISC_REG_CD, READ_LCD_CBUS_REG(PWM_MISC_REG_CD) & ~((1 << 23) | (1<<1)));  //disable pwm_clk & pwm port
                        break;
                    default:
                        break;
                }
                break;
            case KPLED_CTL_EXTERN:
                //TBD
                break;
            default:
                printf("Wrong backlight control method\n");
                break;
        }
    }
    printf("%s(): %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
}

static void set_keypad_led_level(unsigned level)
{
    unsigned pwm_hi = 0, pwm_lo = 0;

    printf("set_led_level: %u, last level: %u\n", level, led_level);
    level = (level > pDev->led_config->level_max ? pDev->led_config->level_max : (level < pDev->led_config->level_min ? pDev->led_config->level_min : level));
    led_level = level;

    //mapping
    if (level > pDev->led_config->level_mid)
        level = ((level - pDev->led_config->level_mid) * (pDev->led_config->level_max - pDev->led_config->level_mid_mapping)) / (pDev->led_config->level_max - pDev->led_config->level_mid) + pDev->led_config->level_mid_mapping;
    else
        level = ((level - pDev->led_config->level_min) * (pDev->led_config->level_mid_mapping - pDev->led_config->level_min)) / (pDev->led_config->level_mid - pDev->led_config->level_min) + pDev->led_config->level_min;

    switch (pDev->led_config->method) {
        case KPLED_CTL_GPIO:
            level = pDev->led_config->dim_min - ((level - pDev->led_config->level_min) * (pDev->led_config->dim_min - pDev->led_config->dim_max)) / (pDev->led_config->level_max - pDev->led_config->level_min);
            WRITE_LCD_CBUS_REG_BITS(LED_PWM_REG0, level, 0, 4);
            break;
        case KPLED_CTL_PWM_NEGATIVE:
        case KPLED_CTL_PWM_POSITIVE:
            level = (pDev->led_config->pwm_max - pDev->led_config->pwm_min) * (level - pDev->led_config->level_min) / (pDev->led_config->level_max - pDev->led_config->level_min) + pDev->led_config->pwm_min;
            if (pDev->led_config->method == KPLED_CTL_PWM_POSITIVE) {
                pwm_hi = level;
                pwm_lo = pDev->led_config->pwm_cnt - level;
            }
            else if (pDev->led_config->method == KPLED_CTL_PWM_NEGATIVE) {
                pwm_hi = pDev->led_config->pwm_cnt - level;
                pwm_lo = level;
            }
            led_print("led pwm_hi=%d\n", pwm_hi);
            led_print("led pwm_lo=%d\n", pwm_lo);

            switch (pDev->led_config->pwm_port) {
                case KPLED_PWM_A:
                    WRITE_LCD_CBUS_REG(PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                    break;
                case KPLED_PWM_B:
                    WRITE_LCD_CBUS_REG(PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                    break;
                case KPLED_PWM_C:
                    WRITE_LCD_CBUS_REG(PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                    break;
                case KPLED_PWM_D:
                    WRITE_LCD_CBUS_REG(PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                    break;
                default:
                    break;
            }
            break;
        case KPLED_CTL_PWM_COMBO:
            if (level >= pDev->led_config->combo_level_switch) {
                //pre_set combo_low duty max
                if (pDev->led_config->combo_low_method == BL_CTL_PWM_NEGATIVE) {
                    pwm_hi = pDev->led_config->combo_low_cnt - pDev->led_config->combo_low_duty_max;
                    pwm_lo = pDev->led_config->combo_low_duty_max;
                }
                else {
                    pwm_hi = pDev->led_config->combo_low_duty_max;
                    pwm_lo = pDev->led_config->combo_low_cnt - pDev->led_config->combo_low_duty_max;
                }
                switch (pDev->led_config->combo_low_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                        break;
                    default:
                        break;
                }

                //set combo_high duty
                level = (pDev->led_config->combo_high_duty_max - pDev->led_config->combo_high_duty_min) * (level - pDev->led_config->combo_level_switch) / (pDev->led_config->level_max - pDev->led_config->combo_level_switch) + pDev->led_config->combo_high_duty_min;
                if (pDev->led_config->combo_high_method == KPLED_CTL_PWM_NEGATIVE) {
                    pwm_hi = pDev->led_config->combo_high_cnt - level;
                    pwm_lo = level;
                }
                else {
                    pwm_hi = level;
                    pwm_lo = pDev->led_config->combo_high_cnt - level;
                }
                switch (pDev->led_config->combo_high_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                        break;
                    default:
                        break;
                }
            }
            else {
                //pre_set combo_high duty min
                if (pDev->led_config->combo_high_method == KPLED_CTL_PWM_NEGATIVE) {
                    pwm_hi = pDev->led_config->combo_high_cnt - pDev->led_config->combo_high_duty_min;
                    pwm_lo = pDev->led_config->combo_high_duty_min;
                }
                else {
                    pwm_hi = pDev->led_config->combo_high_duty_min;;
                    pwm_lo = pDev->led_config->combo_high_cnt - pDev->led_config->combo_high_duty_min;
                }
                switch (pDev->led_config->combo_high_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                        break;
                    default:
                        break;
                }

                //set combo_low duty
                level = (pDev->led_config->combo_low_duty_max - pDev->led_config->combo_low_duty_min) * (level - pDev->led_config->level_min) / (pDev->led_config->combo_level_switch - pDev->led_config->level_min) + pDev->led_config->combo_low_duty_min;
                if (pDev->led_config->combo_low_method == KPLED_CTL_PWM_NEGATIVE) {
                    pwm_hi = pDev->led_config->combo_low_cnt - level;
                    pwm_lo = level;
                }
                else {
                    pwm_hi = level;
                    pwm_lo = pDev->led_config->combo_low_cnt - level;
                }
                switch (pDev->led_config->combo_low_port) {
                    case KPLED_PWM_A:
                        WRITE_LCD_CBUS_REG(PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_B:
                        WRITE_LCD_CBUS_REG(PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_C:
                        WRITE_LCD_CBUS_REG(PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_D:
                        WRITE_LCD_CBUS_REG(PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                        break;
                    default:
                        break;
                }
            }
            break;
        case KPLED_CTL_EXTERN:
            //TBD
            break;
        default:
            break;
    }
}

static unsigned get_keypad_led_level(void)
{
    led_print("%s :%d\n", __FUNCTION__, led_level);
    return led_level;
}

static void led_test(unsigned num)
{
    int i, loop;
    unsigned level;
    Keypad_Led_Config_t *pConf = pDev->led_config;

    switch (num) {
        case 1:
            //Turn all off
            for (i=0; i<LED_NUM_MAX; i++)
            {
                if (pConf->gpio[i]) {
                    aml_lcd_gpio_set(pConf->gpio[i], LCD_POWER_GPIO_OUTPUT_HIGH);
                }
            }
            //Turn on/off one by one
            for (loop=1; loop<LED_NUM_MAX+1; loop++)
            {
                if ((loop-2) >= 0) {
                        aml_lcd_gpio_set(pConf->gpio[loop-2], LCD_POWER_GPIO_OUTPUT_HIGH);
                }
                if (loop <= LED_NUM_MAX) {
                    aml_lcd_gpio_set(pConf->gpio[loop-1], LCD_POWER_GPIO_OUTPUT_LOW);
                }
                mdelay(1000);
            }
            //Turn all on/off 5 times
            for (loop=0; loop<LED_NUM_MAX; loop++)
            {
                for (i=0; i<LED_NUM_MAX; i++)
                {
                    if (pConf->gpio[i]) {
                        aml_lcd_gpio_set(pConf->gpio[i], LCD_POWER_GPIO_OUTPUT_LOW);
                    }
                }
                mdelay(1000);
                for (i=0; i<LED_NUM_MAX; i++)
                {
                    if (pConf->gpio[i]) {
                        aml_lcd_gpio_set(pConf->gpio[i], LCD_POWER_GPIO_OUTPUT_HIGH);
                    }
                }
                mdelay(1000);
            }
            //Turn all on
            for (i=0; i<LED_NUM_MAX; i++)
            {
                if (pConf->gpio[i]) {
                    aml_lcd_gpio_set(pConf->gpio[i], LCD_POWER_GPIO_OUTPUT_LOW);
                }
            }
            break;
        case 2:
            level = pConf->level_min;
            for (loop=0; loop<13; loop++)
            {
                set_keypad_led_level(level);
                level = level + 20;
                mdelay(2000);
            }
            break;
        default:
            printf("un-support number\n");
            printf("video dev test 1~2: show different test flash\n");
            break;
    }
}

static void print_led_info(void)
{
    Keypad_Led_Config_t *pConf = pDev->led_config;

    printf("led control_method: %s(%u)\n", led_ctrl_method_table[pConf->method], pConf->method);

    switch (pConf->pwm_port) {
        case 0:
            printf("led pwm_port: PWM_A\n");
            break;
        case 1:
            printf("led pwm_port: PWM_B\n");
            break;
        case 2:
            printf("led pwm_port: PWM_C\n");
            break;
        case 3:
            printf("led pwm_port: PWM_D\n");
            break;
    }
}

static void _kpled_init(Keypad_Led_Config_t *pConf)
{
#ifdef CONFIG_PLATFORM_HAS_PMU
    struct aml_pmu_driver *pmu_driver;
    int battery_percent;
#endif

    if (pDev->led_config->level_default == pDev->led_config->level_min) {
        set_keypad_led_level(pDev->led_config->level_min);
    }
    else {
#ifdef CONFIG_PLATFORM_HAS_PMU
        /* if battery percentage is very low, set backlight level as low as possible  */
        pmu_driver = aml_pmu_get_driver();
        if (pmu_driver && pmu_driver->pmu_get_battery_capacity) {
            battery_percent = pmu_driver->pmu_get_battery_capacity();
            if (battery_percent <= BATTERY_LOW_THRESHOLD) {
                set_keypad_led_level(pDev->led_config->level_min + battery_percent + 10);
            } else {
                set_keypad_led_level(pDev->led_config->level_default);
            }
        } else {
            set_keypad_led_level(pDev->led_config->level_default);
        }
#else
        set_keypad_led_level(pDev->led_config->level_default);
#endif
    }

    keypad_led_power_ctrl(ON);
}

#ifdef CONFIG_OF_LIBFDT
static int _get_kpled_config(Keypad_Led_Config_t *led_conf)
{
    int ret=0;
    int nodeoffset;
    char * propdata;
    unsigned int led_para[3];
    int i;
    struct fdt_property *prop;
    char *p;
    const char * str;
    unsigned pwm_freq, pwm_cnt, pwm_pre_div;
    int len;

    nodeoffset = fdt_path_offset(dt_addr, "/keypad_led");
    if(nodeoffset < 0) {
        printf("keypad_led init: not find /keypad_led node %s.\n",fdt_strerror(nodeoffset));
        return ret;
    }

    propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_level_default_uboot_kernel", NULL);
    if(propdata == NULL){
        printf("faild to get kpled_level_default_uboot_kernel\n");
        led_conf->level_default = KPLED_LEVEL_DFT;
    }
    else {
        led_conf->level_default = (be32_to_cpup((u32*)propdata));
    }
    led_print("kpled level default uboot=%u\n", led_conf->level_default);

    propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_level_middle_mapping", NULL);
    if(propdata == NULL){
        printf("faild to get kpled_level_middle_mapping\n");
        led_conf->level_mid = KPLED_LEVEL_MID_DFT;
        led_conf->level_mid_mapping = KPLED_LEVEL_MID_MAPPED_DFT;
    }
    else {
        led_conf->level_mid = (be32_to_cpup((u32*)propdata));
        led_conf->level_mid_mapping = (be32_to_cpup((((u32*)propdata)+1)));
    }
    led_print("kpled level mid=%u, mid_mapping=%u\n", led_conf->level_mid, led_conf->level_mid_mapping);
    
    propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_level_max_min", NULL);
    if(propdata == NULL){
        printf("faild to get kpled_level_max_min\n");
        led_conf->level_min = KPLED_LEVEL_MIN_DFT;
        led_conf->level_max = KPLED_LEVEL_MAX_DFT;
    }
    else {
        led_conf->level_max = (be32_to_cpup((u32*)propdata));
        led_conf->level_min = (be32_to_cpup((((u32*)propdata)+1)));
    }
    led_print("kpled level max=%u, min=%u\n", led_conf->level_max, led_conf->level_min);

    propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_power_on_delay", NULL);
    if(propdata == NULL){
        printf("faild to get kpled_power_on_delay\n");
        led_conf->power_on_delay = 100;
    }
	else {
        led_conf->power_on_delay = (unsigned short)(be32_to_cpup((u32*)propdata));
    }
    led_print("kpled power_on_delay: %ums\n", led_conf->power_on_delay);

    propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_ctrl_method", NULL);
    if(propdata == NULL){
        printf("faild to get led_ctrl_method\n");
        led_conf->method = KPLED_CTL_PWM_POSITIVE;
    }
    else {
        led_conf->method = (be32_to_cpup((u32*)propdata) > KPLED_CTL_MAX) ? (KPLED_CTL_MAX-1) : (unsigned char)(be32_to_cpup((u32*)propdata));
    }
    led_print("kpled control_method: %s(%u)\n", led_ctrl_method_table[led_conf->method], led_conf->method);

    if (led_conf->method == KPLED_CTL_GPIO) {
        //TBD
    }
    else if ((led_conf->method == KPLED_CTL_PWM_NEGATIVE) || (led_conf->method == KPLED_CTL_PWM_POSITIVE)) {
        propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_pwm_port_gpio_used", NULL);
        if(propdata == NULL){
            printf("faild to get kpled_pwm_port_gpio_used\n");
            str = "PWM_C";
            led_conf->pwm_port = KPLED_PWM_C;
            led_conf->pwm_gpio_used = 0;
        }
        else {
            prop = container_of(propdata, struct fdt_property, data);
            p = prop->data;
            str = p;
            if (strcmp(str, "PWM_A") == 0)
                led_conf->pwm_port = KPLED_PWM_A;
            else if (strcmp(str, "PWM_B") == 0)
                led_conf->pwm_port = KPLED_PWM_B;
            else if (strcmp(str, "PWM_C") == 0)
                led_conf->pwm_port = KPLED_PWM_C;
            else if (strcmp(str, "PWM_D") == 0)
                led_conf->pwm_port = KPLED_PWM_D;

            p += strlen(p) + 1;
            str = p;
            if (strncmp(str, "1", 1) == 0)
                led_conf->pwm_gpio_used = 1;
            else
                led_conf->pwm_gpio_used = 0;
        }
        led_print("led pwm_port: %s(%u)\n", propdata, led_conf->pwm_port);
        led_print("led pwm gpio_used: %u\n", led_conf->pwm_gpio_used);

        if (led_conf->pwm_gpio_used == 1) {
            propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_gpio_port_on_off", NULL);
            if (propdata == NULL) {
                printf("faild to get kpled_gpio_port_on_off\n");
                led_conf->gpio[0] = GPIOX_16;   /* KEY1_LED */
                led_conf->gpio[1] = GPIOX_17;   /* KEY2_LED */
                led_conf->gpio[2] = GPIOX_10;   /* KEY3_LED */
                led_conf->gpio[3] = GPIOX_19;   /* KEY4_LED */
                led_conf->gpio[4] = GPIOX_11;   /* KEY5_LED */
            }
            else {
                prop = container_of(propdata, struct fdt_property, data);
                p = prop->data;
                str = p;
                for(i=0; i<LED_NUM_MAX; i++) {
                    led_conf->gpio[i] = aml_lcd_gpio_name_map_num(str);
                    led_print("kpled gpio[%d] = %s(%d)\n", i, str, led_conf->gpio[i]);
                    p += strlen(p) + 1;
                    str = p;
                }
            }
        }

        propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_pwm_freq", NULL);
        if(propdata == NULL){
            pwm_freq = 300000;
            printf("faild to get kpled_pwm_freq, default set to %uHz\n", pwm_freq);
        }
        else {
            pwm_freq = be32_to_cpup((u32*)propdata);
            pwm_freq = ((pwm_freq >= (FIN_FREQ * 500)) ? (FIN_FREQ * 500) : pwm_freq);
        }
        for (i=0; i<0x7f; i++) {
            pwm_pre_div = i;
            pwm_cnt = FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
            if (pwm_cnt <= 0xffff)
                break;
        }
        led_conf->pwm_cnt = pwm_cnt;
        led_conf->pwm_pre_div = pwm_pre_div;
        led_print("kpled pwm_frequency = %uHz, pwm_cnt = %u, pre_div = %u\n", pwm_freq, led_conf->pwm_cnt, led_conf->pwm_pre_div);

        propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "kpled_pwm_duty_max_min", NULL);
        if(propdata == NULL){
            printf("faild to get kpled_pwm_duty_max_min\n");
            led_para[0] = 100;
            led_para[1] = 20;
        }
        else {
            led_para[0] = be32_to_cpup((u32*)propdata);
            led_para[1] = be32_to_cpup((((u32*)propdata)+1));
        }
        led_conf->pwm_max = (led_conf->pwm_cnt * led_para[0] / 100);
        led_conf->pwm_min = (led_conf->pwm_cnt * led_para[1] / 100);
        led_print("led kppwm_duty max = %u\%, min = %u\%\n", led_para[0], led_para[1]);
    }
    else if (led_conf->method == KPLED_CTL_PWM_COMBO) {
        //TBD
    }

    //get led pinmux for pwm
    len = 0;
    switch (led_conf->method) {
        case KPLED_CTL_PWM_NEGATIVE:
        case KPLED_CTL_PWM_POSITIVE:
            nodeoffset = fdt_path_offset(dt_addr, "/pinmux/keypad_led");
            if(nodeoffset < 0)
                printf("led init: not find /pinmux/keypad_led node %s.\n",fdt_strerror(nodeoffset));
            else
                len = 1;
            break;
        case BL_CTL_PWM_COMBO:
            //TBD
            break;
        default:
            break;
    }
    if (len > 0) {
        propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "amlogic,setmask", &len);
        if(propdata == NULL){
            printf("faild to get amlogic,setmask\n");
            led_conf->pinmux_set_num = 0;
        }
        else {
            led_conf->pinmux_set_num = len / 8;
            for (i=0; i<led_conf->pinmux_set_num; i++) {
                led_conf->pinmux_set[i][0] = be32_to_cpup((((u32*)propdata)+2*i));
                led_conf->pinmux_set[i][1] = be32_to_cpup((((u32*)propdata)+2*i+1));
            }
        }
        propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "amlogic,clrmask", &len);
        if(propdata == NULL){
            printf("faild to get amlogic,clrmask\n");
            led_conf->pinmux_clr_num = 0;
        }
        else {
            led_conf->pinmux_clr_num = len / 8;
            for (i=0; i<led_conf->pinmux_clr_num; i++) {
                led_conf->pinmux_clr[i][0] = be32_to_cpup((((u32*)propdata)+2*i));
                led_conf->pinmux_clr[i][1] = be32_to_cpup((((u32*)propdata)+2*i+1));
            }
        }
        
        for (i=0; i<led_conf->pinmux_set_num; i++) {
            led_print("kpled pinmux set %d: mux_num = %d, mux_mask = 0x%x\n", i+1, led_conf->pinmux_set[i][0], led_conf->pinmux_set[i][1]);
        }
        for (i=0; i<led_conf->pinmux_clr_num; i++) {
            led_print("kpled pinmux clr %d: mux_num = %d, mux_mask = 0x%x\n", i+1, led_conf->pinmux_clr[i][0], led_conf->pinmux_clr[i][1]);
        }
    }
    return ret;
}
#endif

static void prepare_led_debug(void)
{
#ifdef LED_DEBUG_INFO
    led_print_flag = 1;
#else
    if (getenv("led_print_flag") == NULL)
        led_print_flag = 0;
    else
        led_print_flag = simple_strtoul(getenv("led_print_flag"), NULL, 10);
#endif //LED_DEBUG_INFO

    led_print("led print flag: %u\n", led_print_flag);

    if (getenv("led_debug_flag") == NULL)
        led_debug_flag = 0;
    else
        led_debug_flag = simple_strtoul(getenv("led_debug_flag"), NULL, 10);
}

int led_probe(void)
{
    pDev = (led_dev_t *)malloc(sizeof(led_dev_t));
    if (!pDev) {
        printf("[lcd]: Not enough memory.\n");
        return -1;
    }

    prepare_led_debug();

    dts_ready = 0;	//prepare dts_ready flag, default no dts

#ifdef CONFIG_OF_LIBFDT
#ifdef CONFIG_DT_PRELOAD
    int ret;

#ifdef CONFIG_DTB_LOAD_ADDR
    dt_addr = (char *)CONFIG_DTB_LOAD_ADDR;
#else
    dt_addr = (char *)0x0f000000;
#endif //CONFIG_DTB_LOAD_ADDR

    ret = fdt_check_header(dt_addr);
    if(ret < 0) {
        dts_ready = 0;
        printf("check dts: %s, load default lcd parameters\n", fdt_strerror(ret));
    }
    else {
    	dts_ready = 1;
    }
#endif //CONFIG_DT_PRELOAD
#endif //CONFIG_OF_LIBFDT

    if (led_debug_flag > 0)
        dts_ready = 0;

    if(dts_ready == 0) {
        pDev->led_config = &kpled_config_dft;
        kpled_default_config_init(pDev->led_config);
        printf("load default led model\n");
    }
    else {
#ifdef CONFIG_OF_LIBFDT
        pDev->led_config = &led_config;
        _get_kpled_config(pDev->led_config);
#endif
    }

    _kpled_init(pDev->led_config);
    return 0;
}

int led_remove(void)
{
    keypad_led_power_ctrl(OFF);

    if (pDev)
        free(pDev);
    pDev = NULL;

    return 0;
}

//***************************************************//
//for led_function_call by other module, compatible dts
//***************************************************//
static void _led_enable(void)
{
    if (pDev == NULL)
        led_probe();
    else
        printf("led is already ON\n");
}

static void _led_disable(void)
{
    if (pDev != NULL)
        led_remove();
    else
        printf("led is already OFF\n");
}

static void _enable_led(void)
{
    if (pDev != NULL)
        keypad_led_power_ctrl(ON);
}
static void _disable_led(void)
{
    if (pDev != NULL)
        keypad_led_power_ctrl(OFF);
}

static void _set_led_level(unsigned level)
{
    if (pDev != NULL)
        set_keypad_led_level(level);
}

static unsigned _get_led_level(void)
{
    if (pDev != NULL)
        return get_keypad_led_level();
    else
        return 0;
}

static void _led_test(unsigned num)
{
    if (pDev != NULL)
        led_test(num);
    else
        printf("led is OFF, can't display test pattern\n");
}

static void _led_info(void)
{
    if (pDev != NULL) {
        print_led_info();
    }
    else {
        printf("led is OFF\n");
    }
}

struct kpled_operations kpled_oper =
{
    .enable        = _led_enable,
    .disable       = _led_disable,
    .led_on        = _enable_led,
    .led_off       = _disable_led,
    .set_led_level = _set_led_level,
    .get_led_level = _get_led_level,
    .test          = _led_test,
    .info          = _led_info,
};
//****************************************

#endif
