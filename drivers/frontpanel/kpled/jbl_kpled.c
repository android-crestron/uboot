#include <common.h>
#include <stdio_dev.h>
#include <amlogic/aml_lcd.h>
#include <jabil/jbl_kpled.h>

char kpled_is_enabled = 0;

static void opt_cmd_help(void)
{
	printf("Help:\n");
	printf("enable              -enable keypad led\n");
	printf("disable             -disable keypad led\n");
	printf("kpled_on            -keypad led on\n");
	printf("kpled_off           -keypad led off\n");
	printf("brightness <level>  -set keypad led brightness level\n");
	printf("test                -test keypad led display\n");
	printf("info                -print keypad led driver info\n");
}

int kpled_opt_cmd(int argc, char * const argv[])
{
	if(strcmp(argv[0], "enable") == 0)
	{
		kpled_is_enabled = 1;
		kpled_oper.enable();
	}
	else if(strcmp(argv[0], "disable") == 0)
	{
		kpled_is_enabled = 0;
		kpled_oper.disable();
	}
	else if(strcmp(argv[0], "kpled_on") == 0)
	{
		kpled_oper.led_on();
	}
	else if(strcmp(argv[0], "kpled_off") == 0)
	{
		kpled_oper.led_off();
	}
	else if(strcmp(argv[0], "brightness") == 0)
	{
		kpled_oper.set_led_level(simple_strtoul(argv[1], NULL, 10));
	}
	else if(strcmp(argv[0], "test") == 0)
	{
		kpled_oper.test(simple_strtoul(argv[1], NULL, 10));
	}
	else if (strcmp(argv[0], "info") == 0)
	{
		kpled_oper.info();
	}
	else
	{
		printf("Current device is frontpanel keypad LED.\n");
		opt_cmd_help();
		return 1;
	}
	return 0;
}

/*=================================================================================
// Keypad LED initialize
=================================================================================*/
int jbl_kpled_init(void)
{	
	kpled_oper.enable();
	kpled_is_enabled = 1;		

	printf("Keypad LED enabled:	%d\n", kpled_is_enabled);
	return 0;
}

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

