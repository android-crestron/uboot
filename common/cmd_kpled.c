/*
 * Command for keypad led.
 *
 * Copyright (C) 2015 Jabil.
 * Chris Chen
 */
#include <common.h>
#include <video_fb.h>
#include <jabil/jbl_kpled.h>

/********************************************************************************************
*
*										Keypad LED
*
*********************************************************************************************/
static int do_kpled_dev(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	//int i;
	char *kpled_dev;

	kpled_dev = getenv ("kpled_dev");
#ifdef CONFIG_FRONTPANEL_JBLKPLED
	if(strcmp(kpled_dev, "frontpanel") == 0)
	{
		return kpled_opt_cmd(--argc, ++argv);
	}
#endif
	printf("ERROR:env kpled_dev invalid! kpled_dev is %s\n", kpled_dev);
	return 1;
}


static cmd_tbl_t cmd_kpled_sub[] = {
	U_BOOT_CMD_MKENT(dev, 3, 0, do_kpled_dev, "", ""),
};


static int do_kpled(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *c;

	/* Strip off leading 'bmp' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], &cmd_kpled_sub[0], ARRAY_SIZE(cmd_kpled_sub));

	if (c) {
		return	c->cmd(cmdtp, flag, argc, argv);
	} else {
		cmd_usage(cmdtp);
		return 1;
	}
}

U_BOOT_CMD(
	kpled,	8,	0,	do_kpled,
	"kpled sub-system",
	"kpled dev <opt>	- operate on the frontpanel keypad led device, opt=? for help\n"
);
