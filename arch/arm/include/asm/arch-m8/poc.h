#ifndef MESON_POC_H
#define MESON_POC_H

#define R_BOOT_DEVICE_FLAG  READ_CBUS_REG(ASSIST_POR_CONFIG)
#define POR_BOOT_VALUE 	((((R_BOOT_DEVICE_FLAG>>9)&1)<<2)|((R_BOOT_DEVICE_FLAG>>6)&3))

#define POR_NAND_BOOT() ((POR_BOOT_VALUE == 7) || (POR_BOOT_VALUE == 6))
#define POR_SPI_BOOT()  ((POR_BOOT_VALUE == 5) || (POR_BOOT_VALUE == 4))
#define POR_EMMC_BOOT() ((IS_MESON_M8M2_CPU)?((POR_BOOT_VALUE == 3) || (POR_BOOT_VALUE == 1)):(POR_BOOT_VALUE == 3))
#define POR_CARD_BOOT() (POR_BOOT_VALUE == 0)

#endif
