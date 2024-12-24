#include <common.h>
#include <command.h>
#include <amlogic/aml_lcd.h>
#include <jabil/jbl_ctp.h>

#define LCD_5INCH_NAME             "5inch"
#define LCD_7INCH_NAME             "7inch"
#define LCD_10INCH_NAME            "10inch"

#define LCD_5INCH_HW_ID            0
#define LCD_7INCH_HW_ID            1
#define LCD_10INCH_HW_ID           2

#define LCD_5INCH_DISPLAY_WIDTH    512
#define LCD_5INCH_DISPLAY_HEIGHT   960
#define LCD_5INCH_FB_WIDTH         540
#define LCD_5INCH_FB_HEIGHT        960

#define LCD_7INCH_DISPLAY_WIDTH    1024
#define LCD_7INCH_DISPLAY_HEIGHT   600
#define LCD_7INCH_FB_WIDTH         1024
#define LCD_7INCH_FB_HEIGHT        600

#define LCD_10INCH_DISPLAY_WIDTH   1280
#define LCD_10INCH_DISPLAY_HEIGHT  800
#define LCD_10INCH_FB_WIDTH        1280
#define LCD_10INCH_FB_HEIGHT       800

static int do_lcd_enable(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	panel_oper.enable();
	return 0;
}

static int do_lcd_disable(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	panel_oper.disable();
	return 0;
}

static int do_lcd_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	panel_oper.test(simple_strtoul(argv[1], NULL, 10));
	return 0;
}

static int do_lcd_info(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	panel_oper.info();
	return 0;
}

static int do_lcd_bl(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int level;

	if (argc == 1) {
		return -1;
	}
	if (strcmp(argv[1], "on") == 0) {
		panel_oper.bl_on();
	} else if (strcmp(argv[1], "off") == 0) {
		panel_oper.bl_off();
	} else if (strcmp(argv[1], "level") == 0) {
		if (argc == 3) {
			level = (unsigned int)simple_strtoul(argv[2], NULL, 10);
			panel_oper.set_bl_level(level);
		} else {
			return 1;
		}
	} else {
		return 1;
	}
	return 0;
}

static int do_lcd_dpck(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int lcd_size_config;
	char *s;
	int dpw, dph, fbw, fbh;
	char buf[8];
	int save_flag = 0;

	get_lcd_size(&lcd_size_config);
	memset(buf, 0 , 8);
	get_government(buf);

	/* YUSHAN Government (Microphone+Camera) */
	setenv("government", buf);

	memset(buf, 0 , 8);
	get_orientation(buf);

	/* YUSHAN Orientation (Portrait Display) */
	if (strncmp(buf, "1", 1) == 0)
		setenv("orientation", "portrait");

	/* YUSHAN only LCD output */
	setenv("outputmode", "panel");

	/* get display_width and display_height */
	s = getenv("display_width");
	dpw = (int)simple_strtoull (s, NULL, 10);
	s = getenv("display_height");
	dph = (int)simple_strtoull (s, NULL, 10);
	/* get fb_width and fb_height */
	s = getenv("fb_width");
	fbw = (int)simple_strtoull (s, NULL, 10);
	s = getenv("fb_height");
	fbh = (int)simple_strtoull (s, NULL, 10);

	memset(buf, 0 , 8);

	if (lcd_size_config==LCD_5INCH_HW_ID) {
		/* check display_width and display_height setting, if not match then reconfig them */
		if (dpw!=LCD_5INCH_DISPLAY_WIDTH || dph!=LCD_5INCH_DISPLAY_HEIGHT) {
			sprintf(buf, "%d", LCD_5INCH_DISPLAY_WIDTH);
			setenv("display_width", buf);
			sprintf(buf, "%d", LCD_5INCH_DISPLAY_HEIGHT);
			setenv("display_height", buf);
			save_flag = 1;
		}

		/* check fb_width and fb_height setting, if not match then reconfig them */
		if (fbw!=LCD_5INCH_FB_WIDTH || fbh!=LCD_5INCH_FB_HEIGHT) {
			sprintf(buf, "%d", LCD_5INCH_FB_WIDTH);
			setenv("fb_width", buf);
			sprintf(buf, "%d", LCD_5INCH_FB_HEIGHT);
			setenv("fb_height", buf);
			save_flag = 1;
		}

		s = getenv("lcdsize");
		if((s == NULL) || (strcmp(s, LCD_5INCH_NAME) != 0)) {
			setenv("lcdsize", LCD_5INCH_NAME);
			save_flag = 1;
		}
	}
	else if (lcd_size_config==LCD_7INCH_HW_ID) {
		/* check display_width and display_height setting, if not match then reconfig them */
		if (dpw!=LCD_7INCH_DISPLAY_WIDTH || dph!=LCD_7INCH_DISPLAY_HEIGHT) {
			sprintf(buf, "%d", LCD_7INCH_DISPLAY_WIDTH);
			setenv("display_width", buf);
			sprintf(buf, "%d", LCD_7INCH_DISPLAY_HEIGHT);
			setenv("display_height", buf);
			save_flag = 1;
		}

		/* check fb_width and fb_height setting, if not match then reconfig them */
		if (fbw!=LCD_7INCH_FB_WIDTH || fbh!=LCD_7INCH_FB_HEIGHT) {
			sprintf(buf, "%d", LCD_7INCH_FB_WIDTH);
			setenv("fb_width", buf);
			sprintf(buf, "%d", LCD_7INCH_FB_HEIGHT);
			setenv("fb_height", buf);
			save_flag = 1;
		}

		s = getenv("lcdsize");
		if((s == NULL) || (strcmp(s, LCD_7INCH_NAME) != 0)) {
			setenv("lcdsize", LCD_7INCH_NAME);
			save_flag = 1;
		}
	}
	else if (lcd_size_config==LCD_10INCH_HW_ID) {
		/* check display_width and display_height setting, if not match then reconfig them */
		if (dpw!=LCD_10INCH_DISPLAY_WIDTH || dph!=LCD_10INCH_DISPLAY_HEIGHT) {
			sprintf(buf, "%d", LCD_10INCH_DISPLAY_WIDTH);
			setenv("display_width", buf);
			sprintf(buf, "%d", LCD_10INCH_DISPLAY_HEIGHT);
			setenv("display_height", buf);
			save_flag = 1;
		}

		/* check fb_width and fb_height setting, if not match then reconfig them */
		if (fbw!=LCD_10INCH_FB_WIDTH || fbh!=LCD_10INCH_FB_HEIGHT) {
			sprintf(buf, "%d", LCD_10INCH_FB_WIDTH);
			setenv("fb_width", buf);
			sprintf(buf, "%d", LCD_10INCH_FB_HEIGHT);
			setenv("fb_height", buf);
			save_flag = 1;
		}

		s = getenv("lcdsize");
		if((s == NULL) || (strcmp(s, LCD_10INCH_NAME) != 0)) {
			setenv("lcdsize", LCD_10INCH_NAME);
			save_flag = 1;
		}
	}
	else {
		printf("No defined\n");
		return -1;
	}

	if (save_flag == 1)
		saveenv();

	return 0;
}

static cmd_tbl_t cmd_lcd_sub[] = {
	U_BOOT_CMD_MKENT(enable, 2, 0,  do_lcd_enable, "", ""),
	U_BOOT_CMD_MKENT(disable, 2, 0, do_lcd_disable, "", ""),
	U_BOOT_CMD_MKENT(test, 2, 0,    do_lcd_test, "", ""),
	U_BOOT_CMD_MKENT(info, 2, 0,    do_lcd_info, "", ""),
	U_BOOT_CMD_MKENT(bl, 3, 0,      do_lcd_bl, "", ""),
	U_BOOT_CMD_MKENT(dpck, 2, 0,    do_lcd_dpck, "", ""),
};

static int do_lcd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *c;

	/* Strip off leading 'bmp' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], &cmd_lcd_sub[0], ARRAY_SIZE(cmd_lcd_sub));

	if (c) {
		return c->cmd(cmdtp, flag, argc, argv);
	} else {
		cmd_usage(cmdtp);
		return 1;
	}
}

U_BOOT_CMD(
	lcd,	8,	0,	do_lcd,
	"lcd sub-system",
	"lcd enable   - enable lcd\n"
	"lcd disable  -disable lcd\n"
	"lcd test     -test lcd display\n"
	"lcd info     -print lcd driver info\n"
	"lcd bl on    -lcd backlight on\n"
	"lcd bl off   -lcd backlight off\n"
	"lcd bl level <level>  -set backlight level\n"
	"lcd dpck     -check HW config then set lcd display width and height\n"
);
