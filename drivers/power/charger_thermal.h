#ifndef CHARGER_THERMAL_H
#define CHARGER_THERMAL_H

#define CHARGER_TAG "CHARGER_THERMAL"

#define CHARGE_MAX_LEVEL 10
#define MAX_BUF_SIZE 1024

#define INVALID_NUM -100

struct vadc_map_pt {
	int voltage;
	int temp;
};

struct temp_map_level {
	int temp_trg;
	int temp_clr;
	int level;
};

int voltage_to_temp(int voltage);
int update_t2l_table(char const *src);
int temp_to_level(int temp);

int v2t_sample(int voltage);
void t2l_table_update_sample(char *src);
ssize_t t2l_table_update_sample2(const char __user *src, size_t len);
int t2l_sample(int temp);

#endif
