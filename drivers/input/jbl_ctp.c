#include <common.h>
#if defined(CONFIG_AML_I2C)
#include <aml_i2c.h>
#include <asm/arch/io.h>
#endif /*CONFIG_AML_I2C*/
#include <amlogic/gpio.h>
#include <spi_flash.h>
#include <jabil/jbl_ctp.h>

struct jbl_lcm g_jbl_lcm_judge[] = {
    {.id=0x00,     .supplier="Truly",     .dimension="5inch",    .type="CPT"},
    {.id=0x12,     .supplier="Truly",     .dimension="7inch",    .type="HSD"},  //HannStar
    {.id=0x21,     .supplier="Truly",     .dimension="10inch",   .type="AUO"}
};

#if defined(CONFIG_AML_I2C)
#define CTP_I2C_ADDR    (0x70 >> 1) //7bit address
#define CTP_I2C_BUS     AML_I2C_MASTER_B

static unsigned aml_i2c_bus_tmp;
extern int aml_i2c_xfer_slow(struct i2c_msg *msgs, int num);

static unsigned char sGpioLcdSize = 0, sSpiLcdSize = 0, sTouchLcdSize = 0, sGovernment = 0, sOrientation = 0;
static int sLcmSupplier = 0;
static unsigned char sTouchLcmInfo = 0;

struct aml_ctp_pinmux_t {
	unsigned reg;
	unsigned mux;
};

static struct aml_ctp_pinmux_t aml_ctp_pinmux_set[] = {
    {.reg = 5, .mux = ((1 << 26) | (1 << 27)),},
};

static struct aml_ctp_pinmux_t aml_ctp_pinmux_clr[] = {
    {.reg = 6, .mux = ((1 << 2) | (1 << 3)),},
};

static int aml_ctp_i2c_read(unsigned i2caddr, unsigned char *buff, unsigned len)
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
			.buf   = buff+1,
		}
	};
	/* i2c_transfer-> Returns negative errno, else the number of messages executed. */
	res = aml_i2c_xfer_slow(msg, 2);
	if (res < 0) {
		printf("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2caddr);
	}
	else if (res == 2)
		res = 0;

	return res;
}
#endif


#ifdef CONFIG_AML_I2C
static int aml_ctp_port_init(void)
{
    int i;
    unsigned pinmux_reg, pinmux_data;
    int ret=0;

    for (i=0; i<ARRAY_SIZE(aml_ctp_pinmux_set); i++) {	/* Enable I2C fucntion for GPIOZ_2 & GPIOZ_3 pins */
        pinmux_reg = PERIPHS_PIN_MUX_0+aml_ctp_pinmux_set[i].reg;
        pinmux_data = aml_ctp_pinmux_set[i].mux;
        WRITE_CBUS_REG(pinmux_reg, READ_CBUS_REG(pinmux_reg) | pinmux_data);
    }
    for (i=0; i<ARRAY_SIZE(aml_ctp_pinmux_clr); i++) {	/* Disable ETH fucntion for GPIOZ_2 & GPIOZ_3 pins */
        pinmux_reg = PERIPHS_PIN_MUX_0+aml_ctp_pinmux_clr[i].reg;
        pinmux_data = ~(aml_ctp_pinmux_clr[i].mux);
        WRITE_CBUS_REG(pinmux_reg, READ_CBUS_REG(pinmux_reg) & pinmux_data);
    }

    return ret;
}

static int aml_ctp_change_i2c_bus(unsigned aml_i2c_bus)
{
    int ret=0;
    extern struct aml_i2c_platform g_aml_i2c_plat;

    g_aml_i2c_plat.master_no = aml_i2c_bus;
    ret = aml_i2c_init();

    return ret;
}
#endif

int jbl_ctp_read_register(unsigned char *reg_id, unsigned char *reg_val)
{
	int res = 0;
	extern struct aml_i2c_platform g_aml_i2c_plat;
	unsigned char tData[2]={0};

	aml_i2c_bus_tmp = g_aml_i2c_plat.master_no;
	aml_ctp_port_init();
	aml_ctp_change_i2c_bus(CTP_I2C_BUS);

	tData[0] = *reg_id;
	res = aml_ctp_i2c_read(CTP_I2C_ADDR, tData, 1);
	if (res < 0)
		return res;
	//printf("Focaltech Reg(0x%x)=0x%x\n", tData[0], tData[1]);

	aml_ctp_change_i2c_bus(aml_i2c_bus_tmp);

	*reg_val = tData[1];

	return res;
}

#ifndef CONFIG_SF_DEFAULT_SPEED
# define CONFIG_SF_DEFAULT_SPEED	1000000
#endif
#ifndef CONFIG_SF_DEFAULT_MODE
# define CONFIG_SF_DEFAULT_MODE		SPI_MODE_3
#endif

#define SPI_FLASH_CRC_OFFSET			0x0FFC
#define SPI_FLASH_LCD_SIZE_OFFSET		0x0000
#define SPI_FLASH_GOVERNMENT_OFFSET		0x0001

static unsigned int const Polynomial = 0x04C11DB7;
static unsigned int LookupTable[256] = {0};

void InitCRCLookupTable(void)
{
    int i, j;

    for (i = 0; i < (sizeof(LookupTable)/sizeof(*(LookupTable))); ++i)
    {
        int Divisor = i << 8;
        for (j = 0; j < 8; ++j)
        {
            if (Divisor & 0x8000)
            {
                Divisor <<= 1;
                LookupTable[i] <<= 1;
                LookupTable[i] ^= Polynomial;
                Divisor ^= (Polynomial >> 16);
            }
            else
            {
                LookupTable[i] <<= 1;
                Divisor <<= 1;
            }
        }
    }
}

unsigned int CalculateRunningCRC(unsigned char const *const data, unsigned int const datasize, unsigned char initialRun)
{
	unsigned int remainder = 0;
	static unsigned int prevRemainder = 0;
	unsigned int i, k;

    // Verify parameter(s).
    if (NULL != data)
    {
        //8 bytes at a time crc
        if(initialRun)
		{
			prevRemainder = 0;
			for (k = 0; k < 4; ++k)
			{
				prevRemainder <<= 8;
				prevRemainder |= data[k];
			}
        }
        else
        {
			for (k = 0; k < 4; ++k)
			{
				prevRemainder = LookupTable[prevRemainder >> 24] ^ ((prevRemainder << 8) | data[k]);
			}
        }

        for (i = 0; i < datasize - 4; ++i)
        {
            prevRemainder = LookupTable[prevRemainder >> 24] ^ ((prevRemainder << 8) | data[i + 4]);
        }
        //last 4 bytes need to shift in 0's for remainder
        remainder = prevRemainder;
        for (i = 0; i < 4; ++i)
        {
            remainder = LookupTable[remainder >> 24] ^ ((remainder << 8) | 0);
        }
    }
    else
    {
        printf("%s: BAD parameter: NULL pointer to data.\r\n", __FUNCTION__);
    }
    return remainder;
}

int check_hw_id(void)
{
	unsigned int bus = 0;
	unsigned int cs = 0;
	unsigned int speed = CONFIG_SF_DEFAULT_SPEED;
	unsigned int mode = CONFIG_SF_DEFAULT_MODE;
	struct spi_flash *flash;
	
	unsigned long addr = 0x12000000;
	unsigned long offset = 0x1F000;
	unsigned long len = 0x1000;
	void *buf;
	int ret;
	
	unsigned int crc = 0;
	
	unsigned char reg_id = 0;
	unsigned char lcd_id = 0;
	unsigned char lcm_id = 0;
	
	amlogic_gpio_direction_input(GPIODV_1);
	amlogic_gpio_direction_input(GPIODV_2);
	sGpioLcdSize = ((amlogic_get_value(GPIODV_2) << 1) | (amlogic_get_value(GPIODV_1)));
	
	if (sGpioLcdSize == 3)
	{
		sGpioLcdSize = 0xFF;
	}
	
	flash = spi_flash_probe(bus, cs, speed, mode);
	if (!flash) {
		printf("Failed to initialize SPI flash at %u:%u\n", bus, cs);
		return -1;
	}
	
	buf = map_physmem(addr, len, MAP_WRBACK);
	if (!buf) {
		puts("Failed to map physical memory\n");
		return -1;
	}
	
	ret = spi_flash_read(flash, offset, len, buf);
	if (ret) {
		unmap_physmem(buf, len);
		printf("SPI flash read failed\n");
		return -1;
	}
	
	reg_id = 0xA8;	/* Read FT5X0X_REG_FT5201ID register*/
	if (jbl_ctp_read_register(&reg_id, &lcd_id) == 0)
	{
		printf("LCD ID: 0x%x\n", lcd_id);
		lcm_id = lcd_id;

		// judge which LCM supplier 
		// if ( (( lcm_id >> 6 )&0x01 ) || (( lcm_id >> 6 )&0x10 ) )
		if (((lcm_id & 0xC0) == 0x40) || ((lcm_id & 0xC0) == 0x80))
		{
			sLcmSupplier = 2;	// EDT
		}
		else
		{
			sLcmSupplier = 1;	// Truly
		}
		printf("[check_hw_id] LCM supplier(%d) is %s\n", sLcmSupplier, (sLcmSupplier == 1)?"Truly":"EDT");
		
		sTouchLcmInfo = lcd_id;
		sTouchLcdSize = (lcd_id&0x30) >> 4;
	}
	else
	{
		sTouchLcdSize = 0xFF;
	}
	
	InitCRCLookupTable();
	crc = CalculateRunningCRC((unsigned char*)buf, len-4, 1);
	if (crc == *(int*)((char*)buf+SPI_FLASH_CRC_OFFSET))
	{
		sSpiLcdSize = *(char*)buf & 0x0F; // high nibble means supplier, low nibble means size
		if (sSpiLcdSize > 2)
		{
			sSpiLcdSize = 0xFF;
		}

		sGovernment = *((char*)buf+1);
		sOrientation = *((char*)buf+2);
	}
	else
	{
		sSpiLcdSize = 0xFF;
		sGovernment = 0xFF;
		sOrientation = 0xFF;
	}
	unmap_physmem(buf, len);
	
	//printf("lcd_id=0x%x gpio_lcd_size=0x%x touch_lcd_size=0x%x spi_lcd_size=0x%x\n", lcd_id, sGpioLcdSize, sTouchLcdSize, sSpiLcdSize);
	//printf("spi_government=0x%x\n", sGovernment);
	//printf("spi_portrait=0x%x\n", sOrientation);
	return 0;
}

void get_lcm_supplier( int *lcm_supplier )
{
	*lcm_supplier = sLcmSupplier;
}

void get_lcd_size(int *lcd_size)
{
	if (sGpioLcdSize != 0xFF)
	{
		*lcd_size = sGpioLcdSize;
	}
	else if ((sTouchLcdSize == 0xFF) && (sSpiLcdSize == 0xFF))
	{
		*lcd_size = 0; // default 5inch LCD
	}
	else if (sTouchLcdSize != 0xFF)
	{
		*lcd_size = sTouchLcdSize;
	}
	else
	{
		*lcd_size = sSpiLcdSize;
	}
}

void get_government(char *government)
{
	if (sGovernment == 0)
	{
		strncpy(government, "0", 1);
	}
	else if (sGovernment == 1)
	{
		strncpy(government, "1", 1);
	}
}

void get_orientation(char *orientation)
{
	if (sOrientation == 1)
	{
		strncpy(orientation, "1", 1);
	}
	else
	{
		strncpy(orientation, "0", 1);
	}
}

void check_lcm_type(char *lcm_type)
{
	if (sTouchLcmInfo == 0x5A || sTouchLcmInfo == 0x5B)
	{
		*lcm_type = 0;
	}
	else
	{
		*lcm_type = 1;
	}
}
