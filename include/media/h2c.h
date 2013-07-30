/*
 * h2c.h
 */

#ifndef __H2C_H__
#define __H2C_H__

#include <linux/ioctl.h> /* For IOCTL macros */

//#define H2C_IOCTL_SET_MODE 		_IOW('o', 1, struct h2c_mode)
//#define H2C_IOCTL_GET_VIDEO_FORMAT 	_IOR('o', 5, __u8)

/* H2C Registers */
#define H2C_HDMI_COLOR_FORMAT  (0x8528)
#define H2C_HDMI_MODE_REG      (0x8521)

struct h2c_mode {
	int xres;
	int yres;
	//int resolution;
	//int hdmi_input;
	//int csi_output;
	//int progressive; //progressive or interlace?
	//__u32 frame_length;
};
#ifdef __KERNEL__
struct h2c_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
};
#endif /* __KERNEL__ */

#define OV5640_IOCTL_SET_SENSOR_MODE    _IOW('o', 1, struct ov5640_mode)
#define OV5640_IOCTL_GET_SENSOR_STATUS  _IOR('o', 2, __u8)
#define OV5640_IOCTL_GET_CONFIG         _IOR('o', 3, struct ov5640_config)
#define OV5640_IOCTL_SET_FPOSITION      _IOW('o', 4, __u32)
#define OV5640_IOCTL_POWER_LEVEL        _IOW('o', 5, __u32)
#define OV5640_IOCTL_GET_AF_STATUS      _IOR('o', 6, __u8)
#define OV5640_IOCTL_SET_AF_MODE        _IOR('o', 7, __u8)


#endif /*__H2C_H__ */
