/*
 * (C) Copyright 2000-2010
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2001 Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Andreas Heppel <aheppel@sysgo.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <search.h>
#include <errno.h>
#include <malloc.h>
#include <version.h>
#include <timestamp.h>
#ifdef CONFIG_ENV_IS_IN_AML_NAND
#include <asm/arch/nand.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

extern env_t *env_ptr;

extern void env_relocate_spec (void);
extern uchar env_get_char_spec(int);

static uchar env_get_char_init (int index);

/************************************************************************
 * Default settings to be used when no valid environment is found
 */
#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)

uchar default_environment[] = {
#ifdef	CONFIG_BOOTARGS
	"bootargs="	CONFIG_BOOTARGS			"\0"
#endif
#ifdef	CONFIG_BOOTCOMMAND
	"bootcmd="	CONFIG_BOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_RAMBOOTCOMMAND
	"ramboot="	CONFIG_RAMBOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_NFSBOOTCOMMAND
	"nfsboot="	CONFIG_NFSBOOTCOMMAND		"\0"
#endif
#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	"bootdelay="	MK_STR(CONFIG_BOOTDELAY)	"\0"
#endif
#if defined(CONFIG_BAUDRATE) && (CONFIG_BAUDRATE >= 0)
	"baudrate="	MK_STR(CONFIG_BAUDRATE)		"\0"
#endif
#ifdef	CONFIG_LOADS_ECHO
	"loads_echo="	MK_STR(CONFIG_LOADS_ECHO)	"\0"
#endif
#ifdef	CONFIG_ETHADDR
	"ethaddr="	MK_STR(CONFIG_ETHADDR)		"\0"
#endif
#ifdef	CONFIG_ETH1ADDR
	"eth1addr="	MK_STR(CONFIG_ETH1ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH2ADDR
	"eth2addr="	MK_STR(CONFIG_ETH2ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH3ADDR
	"eth3addr="	MK_STR(CONFIG_ETH3ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH4ADDR
	"eth4addr="	MK_STR(CONFIG_ETH4ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH5ADDR
	"eth5addr="	MK_STR(CONFIG_ETH5ADDR)		"\0"
#endif
#ifdef	CONFIG_IPADDR
	"ipaddr="	MK_STR(CONFIG_IPADDR)		"\0"
#endif
#ifdef	CONFIG_SERVERIP
	"serverip="	MK_STR(CONFIG_SERVERIP)		"\0"
#endif
#ifdef	CONFIG_SYS_AUTOLOAD
	"autoload="	CONFIG_SYS_AUTOLOAD			"\0"
#endif
#ifdef	CONFIG_PREBOOT
	"preboot="	CONFIG_PREBOOT			"\0"
#endif
#ifdef	CONFIG_ROOTPATH
	"rootpath="	MK_STR(CONFIG_ROOTPATH)		"\0"
#endif
#ifdef	CONFIG_GATEWAYIP
	"gatewayip="	MK_STR(CONFIG_GATEWAYIP)	"\0"
#endif
#ifdef	CONFIG_NETMASK
	"netmask="	MK_STR(CONFIG_NETMASK)		"\0"
#endif
#ifdef	CONFIG_HOSTNAME
	"hostname="	MK_STR(CONFIG_HOSTNAME)		"\0"
#endif
#ifdef	CONFIG_BOOTFILE
	"bootfile="	MK_STR(CONFIG_BOOTFILE)		"\0"
#endif
#ifdef	CONFIG_LOADADDR
	"loadaddr="	MK_STR(CONFIG_LOADADDR)		"\0"
#endif
#ifdef  CONFIG_CLOCKS_IN_MHZ
	"clocks_in_mhz=1\0"
#endif
#if defined(CONFIG_PCI_BOOTDELAY) && (CONFIG_PCI_BOOTDELAY > 0)
	"pcidelay="	MK_STR(CONFIG_PCI_BOOTDELAY)	"\0"
#endif
#ifdef  CONFIG_EXTRA_ENV_SETTINGS
	CONFIG_EXTRA_ENV_SETTINGS
#endif
	"\0"
};

struct hsearch_data env_htab={NULL,0,0};

void env_crc_update (void)
{
	env_ptr->crc = crc32(0, env_ptr->data, ENV_SIZE);
}

static uchar env_get_char_init (int index)
{
	uchar c;

	/* if crc was bad, use the default environment */
	if (gd->env_valid)
		c = env_get_char_spec(index);
	else
		c = default_environment[index];

	return (c);
}

uchar env_get_char_memory (int index)
{
	return *env_get_addr(index);
}

uchar env_get_char (int index)
{
	uchar c;

	/* if relocated to RAM */
	if (gd->flags & GD_FLG_RELOC)
		c = env_get_char_memory(index);
	else
		c = env_get_char_init(index);

	return (c);
}

uchar *env_get_addr (int index)
{
	if (gd->env_valid)
		return (uchar *)(gd->env_addr + index);
	else
		return &default_environment[index];
}


#define MAX_REBOOTS	9
#define FIRMWARE_UPDATE_MAX_REBOOTS	6
#define MAX_GOLDEN_REBOOTS	5
void checkBootRetry ( void )
{
	int retryValue = 0;
	int goldenRetryValue = 0;
	int fwUpdate = 0;
	char* testPrnt = NULL;
	char retryStr[20];

	testPrnt=getenv("crestron_uboot_version");
	if ( testPrnt == NULL )
	{
		// ENV never set - update it
		// Perhaps this is a virgin board from   JABIL
		set_default_env("NEW BOARD - NO VERSION");
		saveenv();
	}

	testPrnt=getenv("boot_retry");
	if ( testPrnt != NULL )
	{
		retryValue=testPrnt[0] - '0';
		if ( testPrnt[1] != '\0')
		{
			retryValue = retryValue * 10;
			retryValue = retryValue + ( testPrnt[1] - '0' );
		}
		testPrnt=getenv("fwUpgrade");
		fwUpdate = testPrnt[0] - '0';

		if ( ( retryValue < 0 ) ||( retryValue > MAX_REBOOTS+2 ) )
		{
			retryValue = 0;
		}
		else if ( ( retryValue >= FIRMWARE_UPDATE_MAX_REBOOTS )&& ( fwUpdate > 0 ) )
		{
			retryValue = MAX_REBOOTS; // needed to force update from older versions
		}
		testPrnt=getenv("boot_retry");
		printf ("Checking reboot value %s == %d\n", testPrnt, retryValue );
	}
	else
	{
		printf ("Defaulting reboot value boot_retry == %d\n", retryValue );
	}

	retryValue++;

	testPrnt=getenv("golden_boot_retry");
	if ( testPrnt != NULL )
	{
		goldenRetryValue=testPrnt[0] - '0';
		if ( ( goldenRetryValue < 0 ) ||( goldenRetryValue > MAX_GOLDEN_REBOOTS+2 ) )
		{
			goldenRetryValue = 0;
		}
		printf ("Checking golden reboot value %s == %d\n", testPrnt, goldenRetryValue );
	}
	else
	{
		printf ("Defaulting golden reboot value boot_retry == %d\n", goldenRetryValue );
	}
	goldenRetryValue++;


	if ( retryValue > MAX_REBOOTS )
	{
		// Boot the golden image
		if ( goldenRetryValue > MAX_GOLDEN_REBOOTS )
		{
			setenv ( "bootcmd", "mmcinfo;imgread kernel boot ${loadaddr};bootm;" );
			setenv ( "golden", "0" );
			retryValue = MAX_REBOOTS - 1; // allow us to go to default at least once
			goldenRetryValue = 0;
		}
		else if ( fwUpdate == 0 ) // only tell the kernel to display golden splash in not an upgrade && not a reboot into golden
		{
			setenv ( "bootcmd", "mmcinfo;fatload mmc 0 ${loadaddr} boot.img;bootm;" );
			setenv ( "golden", "1" );
		}
		else // probably a fw upgrade into golden so don't display system maintenance splash
		{
			setenv ( "bootcmd", "mmcinfo;fatload mmc 0 ${loadaddr} boot.img;bootm;" );
			setenv ( "golden", "0" );
		}
	}
	else
	{
		setenv ( "bootcmd", "mmcinfo;imgread kernel boot ${loadaddr};bootm;" );
		setenv ( "golden", "0" );
		goldenRetryValue = 0;
	}

	if ( retryValue <= (MAX_REBOOTS + 1) )
	{
		sprintf ( retryStr, "%d", retryValue );
		setenv ( "boot_retry", retryStr );
	}

	sprintf ( retryStr, "%d", goldenRetryValue );
	setenv ( "golden_boot_retry", retryStr );

	testPrnt=getenv("boot_retry");
	printf ("Reboot value set to %s\n", testPrnt );

	testPrnt=getenv("golden_boot_retry");
	printf ("Golden Reboot value set to %s\n", testPrnt );
}


void set_default_env(const char *s)
{
	if (sizeof(default_environment) > ENV_SIZE) {
		puts("*** Error - default environment is too large\n\n");
		return;
	}

	if (s) {
		if (*s == '!') {
			printf("*** Warning - %s, "
				"using default environment\n\n",
				s+1);
		} else {
			puts(s);
		}
	} else {
		puts("Using default environment\n\n");
	}

	if (himport_r(&env_htab, (char *)default_environment,
		    sizeof(default_environment), '\0', 0) == 0) {
		error("Environment import failed: errno = %d\n", errno);
	}
	gd->flags |= GD_FLG_ENV_READY;
}

/*
 * Check if CRC is valid and (if yes) import the environment.
 * Note that "buf" may or may not be aligned.
 */
int env_import(const char *buf, int check)
{
	env_t *ep = (env_t *)buf;

	if (check) {
		uint32_t crc;

		memcpy(&crc, &ep->crc, sizeof(crc));

		if (crc32(0, ep->data, ENV_SIZE) != crc) {
			set_default_env("!bad CRC");
			return 0;
        }else if(ep->data[0] == 0xff){
			printf("check env data = 0xff, set default env\n");
			set_default_env("!check env data is 0xff");
			saveenv();
			return 0;
 		}
	}

	if (himport_r(&env_htab, (char *)ep->data, ENV_SIZE, '\0', 0)) {
		gd->flags |= GD_FLG_ENV_READY;
		return 1;
	}

	error("Cannot import environment: errno = %d\n", errno);

	set_default_env("!import failed");

	return 0;
}

void env_relocate (void)
{

#if defined (CONFIG_VLSI_EMULATOR)
    set_default_env("!For emulator speed up");
#else
#if defined(CONFIG_NEEDS_MANUAL_RELOC)
	extern void env_reloc(void);

	env_reloc();
#endif
	if (gd->env_valid == 0) {
#if defined(CONFIG_ENV_IS_NOWHERE)	/* Environment not changable */
		set_default_env(NULL);
#else
		show_boot_progress (-60);
		set_default_env("!bad CRC");
#endif
	} else {
		printf("%s [%d]\n", __func__, __LINE__);
		env_relocate_spec ();
	}
#endif //#if !defined (CONFIG_VLSI_EMULATOR)

#if defined(CONFIG_SILENT_CONSOLE) && \
	defined(CONFIG_SILENT_CONSOLE_UPDATE_ON_RELOC)
	if (getenv("silent") != NULL) {
		puts("silenced by env\n");
		gd->flags |= GD_FLG_SILENT;
	} else {
		gd->flags &= ~GD_FLG_SILENT;
	}
#endif
}

#ifdef CONFIG_AUTO_COMPLETE
int env_complete(char *var, int maxv, char *cmdv[], int bufsz, char *buf)
{
	ENTRY *match;
	int found, idx;

	idx = 0;
	found = 0;
	cmdv[0] = NULL;

	while ((idx = hmatch_r(var, idx, &match, &env_htab))) {
		int vallen = strlen(match->key) + 1;

		if (found >= maxv - 2 || bufsz < vallen)
			break;

		cmdv[found++] = buf;
		memcpy(buf, match->key, vallen);
		buf += vallen;
		bufsz -= vallen;
	}

	qsort(cmdv, found, sizeof(cmdv[0]), strcmp_compar);

	if (idx)
		cmdv[found++] = "...";
	cmdv[found] = NULL;
	return found;
}
#endif
