
#ifndef __JABIL_KPLED_H_
#define __JABIL_KPLED_H_
#include <common.h>
#include <linux/list.h>

extern int jbl_kpled_init(void);
extern int kpled_opt_cmd(int argc, char * const argv[]);

typedef struct kpled_operations {
	void  (*enable)(void);
	void  (*disable)(void);
	void  (*led_on)(void);
	void  (*led_off)(void);
	void  (*set_led_level)(unsigned level);
	unsigned (*get_led_level)(void);
	void  (*power_on)(void);
	void  (*power_off)(void);
	void  (*test)(unsigned num);
	void  (*info)(void);
} kpled_operations_t;

extern kpled_operations_t kpled_oper;


#endif


