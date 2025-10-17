#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#ifdef CONFIG_MYFUNCTION
#include "myfunction.h"
#endif

int main(void)
{

	while (1) {
#ifdef CONFIG_MYFUNCTION
		int a = 3, b = 4;
		printk("The sum of %d and %d is %d\n", a, b, sum(a, b));
#else
		printk("MYFUNCTION not enabled\n");
		return 0;
#endif
		k_msleep(1000);
	}
	return 0;
}