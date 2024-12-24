/*
 * AMLOGIC backlight external driver.
 *
 * Communication protocol:
 * I2C 
 *
 */

#include <common.h>
#include <asm/arch/io.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/clock.h>
#include <asm/arch/timing.h>
#include <aml_i2c.h>
#include <amlogic/aml_bl_extern.h>
#include <amlogic/lcdoutc.h>
#ifdef CONFIG_OF_LIBFDT
#include <libfdt.h>
#endif

#ifdef CONFIG_AML_BL_EXTERN
//#define BL_EXT_DEBUG_INFO

#define BL_EXTERN_NAME			"bl_i2c_mp3309c"
#define BL_EXTERN_TYPE			BL_EXTERN_I2C

#define BL_EXTERN_I2C_ADDR		(0x2e >> 1) //7bit address
#define BL_EXTERN_I2C_BUS		AML_I2C_MASTER_B

static unsigned int bl_status = 0;
static unsigned int bl_level = 0;
static unsigned aml_i2c_bus_tmp;
static struct aml_bl_extern_driver_t bl_ext_driver;

extern int aml_i2c_xfer_slow(struct i2c_msg *msgs, int num);

static struct bl_extern_config_t bl_ext_config = {
    .name = BL_EXTERN_NAME,
    .type = BL_EXTERN_TYPE,
    .i2c_addr = BL_EXTERN_I2C_ADDR,
    .i2c_bus = BL_EXTERN_I2C_BUS,
    .gpio_used = 1,
    .gpio = GPIODV_28,
    .gpio_on = 1,
    .gpio_off = 0,
    .dim_min = 1,
    .dim_max = 31,
};

static struct aml_bl_extern_pinmux_t aml_bl_extern_pinmux_set[] = {
    {.reg = 5, .mux = ((1 << 26) | (1 << 27)),},
};

static struct aml_bl_extern_pinmux_t aml_bl_extern_pinmux_clr[] = {
    {.reg = 6, .mux = ((1 << 2) | (1 << 3)),},
};

static unsigned char i2c_init_table[][2] = {
    {0x00, 0xfc}, //bit(7):EN; bit(6~2):D4~D0 (0~0x1F set backlight); bit(1~0):don't care
    {0x01, 0x28}, //bit(7):don't care; bit(6):DIMS; bit(5):SYNC; bit(4~3):OVP1~OVP0; bit(2):VOS; bit(1):LEDO; bit(0):OTP;
    {0xff, 0xff}, //ending flag
};

static int aml_bl_i2c_write(unsigned i2caddr, unsigned char *buff, unsigned len)
{
    int res = 0;
#ifdef BL_EXT_DEBUG_INFO
    int i;
#endif
    struct i2c_msg msg[] = {
        {
        .addr = i2caddr,
        .flags = 0,
        .len = len,
        .buf = buff,
        }
    };
#ifdef BL_EXT_DEBUG_INFO
    printf("%s:", __FUNCTION__);
    for (i=0; i<len; i++) {
        printf(" 0x%02x", buff[i]);
    }
    printf(" [addr 0x%02x]\n", i2caddr);
#endif
    /* i2c_transfer-> Returns negative errno, else the number of messages executed. */
    //res = aml_i2c_xfer(msg, 1);
    res = aml_i2c_xfer_slow(msg, 1);
    if (res < 0) {
        printf("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2caddr);
    }
    else if (res == 1)
        res = 0;

    return res;
}

#if 0
static int aml_bl_i2c_read(unsigned i2caddr, unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msg[] = {
        {
            .addr  = i2caddr,
            .flags = 0,
            .len   = 1,
            .buf   = buff,
        },
        {
            .addr  = i2caddr,
            .flags = I2C_M_RD,
            .len   = len,
            .buf   = buff,
        }
    };
    /* i2c_transfer-> Returns negative errno, else the number of messages executed. */
    //res = aml_i2c_xfer(msg, 2);
    res = aml_i2c_xfer_slow(msg, 2);
    if (res < 0) {
        printf("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2caddr);
    }
    else if (res == 2)
        res = 0;

    return res;
}
#endif

static int bl_extern_i2c_init(void)
{
    unsigned char tData[3];
    int i=0, end_mark=0;
    int ret=0;

    while (end_mark == 0) {
        if (i2c_init_table[i][0] == 0xff) {    //special mark
            if (i2c_init_table[i][1] == 0xff) { //end mark
                end_mark = 1;
            }
            else { //delay mark
                mdelay(i2c_init_table[i][1]);
            }
        }
        else {
            tData[0]=i2c_init_table[i][0];
            tData[1]=i2c_init_table[i][1];
            ret = aml_bl_i2c_write(BL_EXTERN_I2C_ADDR, tData, 2);
        }
        i++;
    }

    return ret;
}

static int aml_bl_extern_port_init(void)
{
    int i;
    unsigned pinmux_reg, pinmux_data;
    int ret=0;

    for (i=0; i<ARRAY_SIZE(aml_bl_extern_pinmux_set); i++) {
        pinmux_reg = PERIPHS_PIN_MUX_0+aml_bl_extern_pinmux_set[i].reg;
        pinmux_data = aml_bl_extern_pinmux_set[i].mux;
        WRITE_CBUS_REG(pinmux_reg, READ_CBUS_REG(pinmux_reg) | pinmux_data);
    }
    for (i=0; i<ARRAY_SIZE(aml_bl_extern_pinmux_clr); i++) {
        pinmux_reg = PERIPHS_PIN_MUX_0+aml_bl_extern_pinmux_clr[i].reg;
        pinmux_data = ~(aml_bl_extern_pinmux_clr[i].mux);
        WRITE_CBUS_REG(pinmux_reg, READ_CBUS_REG(pinmux_reg) & pinmux_data);
    }

    return ret;
}

static int aml_bl_extern_change_i2c_bus(unsigned aml_i2c_bus)
{
    int ret=0;
    extern struct aml_i2c_platform g_aml_i2c_plat;

    g_aml_i2c_plat.master_no = aml_i2c_bus;
    ret = aml_i2c_init();

    return ret;
}

static int bl_extern_set_level(unsigned int level)
{
    unsigned char tData[3];
    int ret = 0;
    extern struct aml_i2c_platform g_aml_i2c_plat;

    bl_level = level;
    if (bl_status) {
        get_bl_level(&bl_ext_config);
#ifdef BL_EXT_DEBUG_INFO
        printf("level=%d, dim_min=%d, dim_max=%d, level_min=%d, level_max=%d\n", level, bl_ext_config.dim_min, bl_ext_config.dim_max, bl_ext_config.level_min, bl_ext_config.level_max);
#endif

#if 0
    level = bl_ext_config->dim_min + ((level - bl_ext_config.level_min) * (bl_ext_config.dim_max - bl_ext_config.dim_min)) / (bl_ext_config.level_max - bl_ext_config.level_min);
#else
    level = (bl_ext_config.dim_min * 10) + ((level - bl_ext_config.level_min) * (bl_ext_config.dim_max - bl_ext_config.dim_min) * 10) / (bl_ext_config.level_max - bl_ext_config.level_min);
    /* Round off the original decimal point, then remove it by divide 10. */
#ifdef BL_EXT_DEBUG_INFO
    printf("decimal point=%d\n", (level%10));
#endif
    if ((level%10) > 4)
        level = (level / 10) + 1;
    else
        level = level / 10;
#endif  
#ifdef BL_EXT_DEBUG_INFO
    printf("convert level to dim step=%d\n", level);
#endif

        /* Swap bits, from D4~D0 to D0~D4 */
        level = ((level&0x01)<<4)+((level&0x02)<<2)+(level&0x04)+((level&0x08)>>2)+((level&0x10)>>4);
#ifdef BL_EXT_DEBUG_INFO
        printf("bits swap=%d\n", level);
#endif

        /* OR enable bit for register 0x00 */
        level = (level << 2) | 0x80;
#ifdef BL_EXT_DEBUG_INFO
        printf("add enable bit=0x%x\n\n", level);
#endif

        aml_i2c_bus_tmp = g_aml_i2c_plat.master_no;
        aml_bl_extern_port_init();
        aml_bl_extern_change_i2c_bus(BL_EXTERN_I2C_BUS);
        tData[0] = 0x0;
        tData[1] = level;
        ret = aml_bl_i2c_write(BL_EXTERN_I2C_ADDR, tData, 2);
        aml_bl_extern_change_i2c_bus(aml_i2c_bus_tmp);
    }

    return ret;
}

static int bl_extern_power_on(void)
{
    int ret=0;
    extern struct aml_i2c_platform g_aml_i2c_plat;

    aml_i2c_bus_tmp = g_aml_i2c_plat.master_no;

    aml_bl_extern_port_init();
    aml_bl_extern_change_i2c_bus(BL_EXTERN_I2C_BUS);

    if (bl_ext_config.gpio_used > 0) {
        if(bl_ext_config.gpio_on == 2)
            bl_extern_gpio_direction_input(bl_ext_config.gpio);
        else
            bl_extern_gpio_direction_output(bl_ext_config.gpio, bl_ext_config.gpio_on);
    }
    ret = bl_extern_i2c_init();
    bl_status = 1;
    aml_bl_extern_change_i2c_bus(aml_i2c_bus_tmp);

    if (bl_level > 0) {
        bl_extern_set_level(bl_level);
    }

    printf("%s\n", __FUNCTION__);
    return ret;
}

static int bl_extern_power_off(void)
{
    int ret=0;
    extern struct aml_i2c_platform g_aml_i2c_plat;

    bl_status = 0;
    aml_i2c_bus_tmp = g_aml_i2c_plat.master_no;

    aml_bl_extern_port_init();
    aml_bl_extern_change_i2c_bus(BL_EXTERN_I2C_BUS);
    //ret = bl_extern_i2c_off();
    if (bl_ext_config.gpio_used > 0) {
        if(bl_ext_config.gpio_off == 2)
            bl_extern_gpio_direction_input(bl_ext_config.gpio);
        else
            bl_extern_gpio_direction_output(bl_ext_config.gpio, bl_ext_config.gpio_off);
    }
    aml_bl_extern_change_i2c_bus(aml_i2c_bus_tmp);

    printf("%s\n", __FUNCTION__);
    return ret;
}

#ifdef CONFIG_OF_LIBFDT
static int get_bl_extern_dt_data(char *dt_addr)
{
	int ret=0;
	int nodeoffset;
	char * propdata;
	struct fdt_property *prop;
	char *p;
	const char * str;
	struct bl_extern_config_t *bl_extern = &bl_ext_config;

	nodeoffset = fdt_path_offset(dt_addr, "/bl_extern_i2c_mp3309c");
	if(nodeoffset < 0) {
		printf("dts: not find /bl_extern_i2c_mp3309c node %s.\n",fdt_strerror(nodeoffset));
		return ret;
	}
	
	propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "gpio_enable_on_off", NULL);
	if (propdata == NULL) {
		printf("faild to get gpio_enable_on_off\n");
		bl_extern->gpio_used = 1;
#ifdef GPIODV_28
		bl_extern->gpio = GPIODV_28;
#endif
#ifdef GPIOD_1
		bl_extern->gpio = GPIOD_1;
#endif
		bl_extern->gpio_on = LCD_POWER_GPIO_OUTPUT_HIGH;
		bl_extern->gpio_off = LCD_POWER_GPIO_OUTPUT_LOW;
	}
	else {
		prop = container_of(propdata, struct fdt_property, data);
		p = prop->data;
		str = p;
		bl_extern->gpio_used = 1;
		bl_extern->gpio = aml_lcd_gpio_name_map_num(p);
		p += strlen(p) + 1;
		str = p;
		if (strncmp(str, "2", 1) == 0)
			bl_extern->gpio_on = LCD_POWER_GPIO_INPUT;
		else if(strncmp(str, "0", 1) == 0)
			bl_extern->gpio_on = LCD_POWER_GPIO_OUTPUT_LOW;
		else
			bl_extern->gpio_on = LCD_POWER_GPIO_OUTPUT_HIGH;
		p += strlen(p) + 1;
		str = p;
		if (strncmp(str, "2", 1) == 0)
			bl_extern->gpio_off = LCD_POWER_GPIO_INPUT;
		else if(strncmp(str, "1", 1) == 0)
			bl_extern->gpio_off = LCD_POWER_GPIO_OUTPUT_HIGH;
		else
			bl_extern->gpio_off = LCD_POWER_GPIO_OUTPUT_LOW;
	}
	printf("bl_extern_gpio = %d, gpio_on = %d, gpio_off = %d\n", bl_extern->gpio,bl_extern->gpio_on,bl_extern->gpio_off);
	propdata = (char *)fdt_getprop(dt_addr, nodeoffset, "dim_max_min", NULL);
	if (propdata == NULL) {
		printf("faild to get dim_max_min\n");
		bl_extern->dim_min = 10;
		bl_extern->dim_max = 255;
	}
	else {
		bl_extern->dim_max = (be32_to_cpup((u32*)propdata));
		bl_extern->dim_min = (be32_to_cpup((((u32*)propdata)+1)));
	}
	printf("bl_extern_dim_min =%x, bl_extern_dim_max =%x\n", bl_extern->dim_min,bl_extern->dim_max);
	return ret;
}
#endif

static int get_bl_extern_config(void)
{
    int ret = 0;

#ifdef CONFIG_OF_LIBFDT
    if (bl_ext_driver.dt_addr) {
        ret = fdt_check_header(bl_ext_driver.dt_addr);
        if(ret < 0) {
            printf("check dts: %s, load bl_ext_config failed\n", fdt_strerror(ret));
        }
        else {
            get_bl_extern_dt_data(bl_ext_driver.dt_addr);
        }
    }
#endif

    if (bl_ext_config.dim_min > 0xff)
        bl_ext_config.dim_min = 0xff;
    if (bl_ext_config.dim_max > 0xff)
        bl_ext_config.dim_max = 0xff;

    return ret;
}

static struct aml_bl_extern_driver_t bl_ext_driver = {
    .name = BL_EXTERN_NAME,
    .type = BL_EXTERN_TYPE,
    .power_on = bl_extern_power_on,
    .power_off = bl_extern_power_off,
    .set_level = bl_extern_set_level,
    .dt_addr = NULL,
    .get_bl_ext_config = get_bl_extern_config,
};

struct aml_bl_extern_driver_t* aml_bl_extern_get_driver(void)
{
    return &bl_ext_driver;
}

#endif
