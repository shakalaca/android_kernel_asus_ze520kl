#ifndef MSM_LASER_FOCUS_VL6180X_API_H
#define MSM_LASER_FOCUS_VL6180X_API_H

#undef CDBG
#define CDBG(fmt, args...) pr_info(fmt, ##args)

#undef DBG_LOG
#define DBG_LOG(fmt, args...) pr_info(fmt, ##args)

#undef REG_RW_DBG
#define REG_RW_DBG(fmt, args...) pr_debug(fmt, ##args)

#undef API_DBG
#define API_DBG(fmt, args...) pr_debug(fmt, ##args)

/* Out of range */
#define OUT_OF_RANGE 9999

/* Time out value: mm */
#define TIMEOUT_VAL 80


#endif
