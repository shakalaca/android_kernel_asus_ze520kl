#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include "charger_thermal.h"

/*************************************
 *                                   *
 *          Global variable          *
 *                                   *
 *************************************/
static int charger_level_size = 0;

/***********************************
 *                                 *
 *          Mapping table          *
 *                                 *
 ***********************************/
/* voltage to temp mapping table */
static const struct vadc_map_pt v2t[] = {
	{2940, -40},
	{2935, -39},
	{2930, -38},
	{2925, -37},
	{2920, -36},
	{2914, -35},
	{2908, -34},
	{2902, -33},
	{2895, -32},
	{2888, -31},
	{2880, -30},
	{2872, -29},
	{2863, -28},
	{2854, -27},
	{2844, -26},
	{2834, -25},
	{2823, -24},
	{2812, -23},
	{2800, -22},
	{2788, -21},
	{2774, -20},
	{2761, -19},
	{2746, -18},
	{2731, -17},
	{2715, -16},
	{2698, -15},
	{2681, -14},
	{2663, -13},
	{2644, -12},
	{2625, -11},
	{2605, -10},
	{2583, -9},
	{2562, -8},
	{2539, -7},
	{2516, -6},
	{2492, -5},
	{2467, -4},
	{2441, -3},
	{2415, -2},
	{2388, -1},
	{2360, 0},
	{2331, 1},
	{2302, 2},
	{2272, 3},
	{2241, 4},
	{2210, 5},
	{2178, 6},
	{2146, 7},
	{2113, 8},
	{2079, 9},
	{2046, 10},
	{2011, 11},
	{1976, 12},
	{1941, 13},
	{1906, 14},
	{1870, 15},
	{1834, 16},
	{1798, 17},
	{1762, 18},
	{1869, 19},
	{1689, 20},
	{1652, 21},
	{1616, 22},
	{1579, 23},
	{1543, 24},
	{1507, 25},
	{1472, 26},
	{1437, 27},
	{1402, 28},
	{1368, 29},
	{1334, 30},
	{1300, 31},
	{1267, 32},
	{1234, 33},
	{1202, 34},
	{1170, 35},
	{1138, 36},
	{1108, 37},
	{1077, 38},
	{1047, 39},
	{1018, 40},
	{990, 41},
	{962, 42},
	{934, 43},
	{907, 44},
	{881, 45},
	{855, 46},
	{830, 47},
	{806, 48},
	{782, 49},
	{759, 50},
	{736, 51},
	{714, 52},
	{693, 53},
	{672, 54},
	{652, 55},
	{632, 56},
	{613, 57},
	{595, 58},
	{576, 59},
	{559, 60},
	{542, 61},
	{525, 62},
	{509, 63},
	{494, 64},
	{479, 65},
	{464, 66},
	{450, 67},
	{437, 68},
	{423, 69},
	{410, 70},
	{398, 71},
	{386, 72},
	{374, 73},
	{363, 74},
	{352, 75},
	{341, 76},
	{331, 77},
	{321, 78},
	{311, 79},
	{302, 80},
	{293, 81},
	{284, 82},
	{276, 83},
	{268, 84},
	{260, 85},
	{252, 86},
	{245, 87},
	{237, 88},
	{230, 89},
	{224, 90},
	{217, 91},
	{211, 92},
	{205, 93},
	{199, 94},
	{193, 95},
	{187, 96},
	{182, 97},
	{177, 98},
	{172, 99},
	{167, 100},
	{162, 101},
	{158, 102},
	{153, 103},
	{149, 104},
	{145, 105},
	{141, 106},
	{137, 107},
	{133, 108},
	{129, 109},
	{126, 110},
	{122, 111},
	{119, 112},
	{115, 113},
	{112, 114},
	{109, 115},
	{106, 116},
	{103, 117},
	{101, 118},
	{98, 119},
	{95, 120},
	{93, 121},
	{90, 122},
	{88, 123},
	{86, 124},
	{83, 125}
};

/* temp to charger level mapping table */
static struct temp_map_level charger_t2l[CHARGE_MAX_LEVEL];
static struct temp_map_level cur_charger_status = {0,0,0};

/**************************************
 *                                    *
 *          General function          *
 *                                    *
 **************************************/
/* Change string to int */
static long str_to_int(char *str, int base){
        long value, rc;

		if(str[0] < '0' || str[0] > '9'){
			return INVALID_NUM;
		}

        rc = kstrtol(str, base, &value);
        if(rc != 0){
			pr_err("[%s] %s: kstrtol fail!!\n", CHARGER_TAG, __func__);
			return INVALID_NUM;
        }
        
        return value;
}
/* Swap value */
static void swap_value(int *v1, int *v2){
	int temp;
	
	temp = *v2;
	*v2 = *v1;
	*v1 = temp;
}

/****************************************
 *                                      *
 *          Show mapping table          *
 *                                      *
 ****************************************/
/* Show voltage to temp mapping table */
void show_v2t_table(void){
	int i = 0;
	
	for(i = 0; i < sizeof(v2t) / sizeof(v2t[0]); i++){
		printk("[%s] voltage(%d) mapping temp is %d\n", CHARGER_TAG, v2t[i].voltage, v2t[i].temp);
	}
}

/* Show temp to charger level mapping table */
void show_t2l_table(void){
	int i = 0;
	
	for(i = 0; i < charger_level_size; i++){
		printk("[%s](%d) charger level[%d] : trg temp is %d, clr temp is %d\n", 
			CHARGER_TAG, i, charger_t2l[i].level, charger_t2l[i].temp_trg, charger_t2l[i].temp_clr);
	}
}

/*********************************************
 *                                           *
 *          Mapping voltage to temp          *
 *                                           *
 *********************************************/
/* Search mapping temp */
static int mapping_charger_temp(int voltage){
	/* Use binary search */
	int low = 0, high = sizeof(v2t) / sizeof(v2t[0]), mid = 0;
	int temp = v2t[0].temp;	

	if(voltage >= v2t[low].voltage){
		temp = v2t[low].temp;
	}else if(voltage <= v2t[high-1].voltage){
		temp = v2t[high-1].temp;
	}else{
		while(low <= high){
			mid = (low + high) / 2;
			if((voltage > v2t[mid].voltage && voltage < v2t[mid-1].voltage) || (voltage == v2t[mid].voltage)){
				temp = v2t[mid].temp;
				break;
			}if(voltage > v2t[mid].voltage){
				high = mid - 1;	
			}else{
				low = mid + 1;
			}
		}
	}

	return temp;
}

/* Change voltage to temp */
int voltage_to_temp(int voltage){
	int temp = v2t[0].temp;

	temp = mapping_charger_temp(voltage);

	return temp;
}

/*****************************************************************
 *                                                               *
 *          Update temp and charger level mapping talbe          *
 *                                                               *
 *****************************************************************/
/* Init temp to charger level mapping table */
static void init_t2l_table(void){
	int i = 0;
	
	charger_level_size = 0;
	
	for(i = 0; i < CHARGE_MAX_LEVEL; i++){
		charger_t2l[i].temp_trg = 999;
		charger_t2l[i].temp_clr = 999;
		charger_t2l[i].level = -1;
	}
}

/* Sort temp to charger level mapping table */
static void sort_t2l_table(void){
	/* Use bubble sort */
	int i = 0, j = 0;
	
	for(i = 0; i <= charger_level_size-1; i ++){
		for(j = (charger_level_size-1); j > i; j--){
			if(charger_t2l[j-1].level > charger_t2l[j].level){
				swap_value(&charger_t2l[j].temp_trg, &charger_t2l[j-1].temp_trg);
				swap_value(&charger_t2l[j].temp_clr, &charger_t2l[j-1].temp_clr);
				swap_value(&charger_t2l[j].level, &charger_t2l[j-1].level);
			}
		}
	}
}

/* Update temp to charger level mapping table */
int update_t2l_table(char const *src){
	char buf[MAX_BUF_SIZE];
	char *pch, *str = buf, *delim = ";\n";
	int level = 0, temp_trg = 0, temp_clr, rc = 0, count = 0;
	
	// Init table
	init_t2l_table();
	
	memcpy(buf, src, strlen(src));
	
	pch = strsep(&str, delim);

	while(pch != NULL){
		level = (int)str_to_int(pch,10);
		if(level != INVALID_NUM){
			pch = strsep(&str, delim);
			if(pch == NULL){
				pr_err("[%s] %s: Invalid param: miss mapping trg temp!!\n", CHARGER_TAG, __func__);
				break;
			}
			temp_trg = (int)str_to_int(pch,10);
			
			pch = strsep(&str, delim);
			if(pch == NULL){
				pr_err("[%s] %s: Invalid param: miss mapping clr temp!!\n", CHARGER_TAG, __func__);
				break;
			}
			temp_clr = (int)str_to_int(pch,10);

			if(temp_trg != INVALID_NUM && temp_clr != INVALID_NUM){
				if(level >= CHARGE_MAX_LEVEL){
					pr_err("[%s] %s: Error: Charger level %d is overflow, it will be ignore!!\n", 
						CHARGER_TAG, __func__, level);
				}else{
					charger_t2l[count].temp_trg = temp_trg;
					charger_t2l[count].temp_clr = temp_clr;
					charger_t2l[count].level = level;
					count++;
				}
			}else{
				//pr_err("[%s] %s: Invalid param: miss mapping temp(trg,clr):(%d,%d)!!\n", 
				//	CHARGER_TAG, __func__, temp_trg, temp_clr);
				break;
			}
		}else{
			//pr_err("[%s] %s: Invalid param: miss mapping level!!\n", CHARGER_TAG, __func__);
			break;
		}
		pch = strsep(&str, delim);
	}
	
	charger_level_size = count;
	
	// Sort table
	sort_t2l_table();

	return rc;
}

/***************************************************
 *                                                 *
 *          Mapping temp to charger table          *
 *                                                 *
 ***************************************************/
 /* Update current charger status */
 void update_cur_charger_status(int temp_trg, int temp_clr, int level){
	cur_charger_status.temp_trg = temp_trg;
	cur_charger_status.temp_clr = temp_clr;
	cur_charger_status.level = level;
}
 
/* Search mapping charger level */
int mapping_charger_level(int temp){
	int level = 0, i = 0;
	
	for(i = 0; i < charger_level_size; i++){
		if(temp >= charger_t2l[i].temp_trg){
			level = charger_t2l[i].level;
			update_cur_charger_status(charger_t2l[i].temp_trg,charger_t2l[i].temp_clr,level);
		}
		
		if(cur_charger_status.level == charger_t2l[i].level){
			if(temp < charger_t2l[i].temp_clr){
				level = charger_t2l[i-1].level;
				update_cur_charger_status(charger_t2l[i-1].temp_trg,charger_t2l[i-1].temp_clr,level);
			}
		}
	}
	
	return level;
}

/* Change temp to charger level */
int temp_to_level(int temp){
	int level = 0;

	level = mapping_charger_level(temp);

	return level;
}

/*********************************
 *          Sample Code          *
 *********************************/
/* Change voltage to temp sample code */
int v2t_sample(int voltage){
	printk("[%s] %s: voltage(%d) to temp(%d)\n", CHARGER_TAG, __func__, voltage, voltage_to_temp(voltage));
	return voltage_to_temp(voltage);
}

/* Update temp to charger level mapping table sample code (for char*) */
void t2l_table_update_sample(char *src){
	update_t2l_table(src);
}

/* Update temp to charger level mapping table sample code (for const char __user*) */
ssize_t t2l_table_update_sample2(const char __user *src, size_t len){
	char buf[MAX_BUF_SIZE];
	char *pch, *str = buf, *delim="\n\0";

	if(copy_from_user(buf, src, len)){
		printk("[%s] %s: command fail!!\n", CHARGER_TAG, __func__);
		return -EFAULT;

	}

	pch = strsep(&str, delim);

	pch[strlen(pch)] = '\n';

	update_t2l_table(pch);
	
	return len;
}

/* Change temp to charger level sample code */
int t2l_sample(int temp){
	printk("[%s] %s: temp(%d) to level(%d)\n", CHARGER_TAG, __func__, temp, temp_to_level(temp));
	return temp_to_level(temp);
}
