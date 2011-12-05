#ifndef _TSL2563_H_
#define _TSL2563_H_


#define TSL2563_DRIVER_NAME "tsl2563"

//. Operation mode
#define TSL2563_POWER_OFF               0
#define TSL2563_POWER_ON                3       // 0h

#define TSL2563_GAIN_16X                1
#define TSL2563_GAIN_1X                 0

#define TSL2563_INTEG_ENABLE            1
#define TSL2563_INTEG_DIABLE            0

#define TSL2563_SET_INTEG_400           2
#define TSL2563_SET_INTEG_100           1
#define TSL2563_SET_INTEG_13            0
#define TSL2563_SET_INTEG_NA            3   //1h


#define TSL2563_IOC_MAGIC               'Z'
#define TSL2563_SET_POWER               _IOW(TSL2563_IOC_MAGIC,  1, unsigned char)
#define TSL2563_SET_GAIN                _IOW(TSL2563_IOC_MAGIC,  2, unsigned char)
#define TSL2563_SET_INTEG               _IOW(TSL2563_IOC_MAGIC,  3, unsigned char)     //. output: '1' means interrupt triggered
#define TSL2563_SET_INTEG_TIME          _IOW(TSL2563_IOC_MAGIC,  4, unsigned char)     //. interrup presist: interrupt triggered after setting cycles
#define TSL2563_SET_THRES_LOW           _IOW(TSL2563_IOC_MAGIC,  5, unsigned char)
#define TSL2563_SET_THRES_HIGH          _IOW(TSL2563_IOC_MAGIC,  6, unsigned char)
#define TSL2563_GET_DATA0               _IOW(TSL2563_IOC_MAGIC,  7, unsigned char)
#define TSL2563_GET_DATA1               _IOW(TSL2563_IOC_MAGIC,  8, unsigned char)

#define TSL2563_IOC_MAXNR              9

#endif  // _TSL2563_H_

