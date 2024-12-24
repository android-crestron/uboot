#ifndef __JABIL_CTP_H_
#define __JABIL_CTP_H_
#include <common.h>
#include <linux/list.h>

#define LCM_NUM 3

/* 
id ==    QQ             PP              LLLL       
      Supplier       Dimension        LCD type     
      00 -> Truly    00 -> 5"       0000 -> CPT    
      01 -> EDT      01 -> 7"       0001 -> AUO    
                     10 -> 10"      0010 -> Hanster
*/
struct jbl_lcm {
	unsigned id;
	char *supplier;
	char *dimension;
	char *type;
};

int check_hw_id(void);
void get_lcm_supplier( int *lcm_supplier );
void get_lcd_size(int *lcd_size);
void get_government(char *government);
void get_orientation(char *orientation);
void check_lcm_type(char *lcm_type);

#endif
