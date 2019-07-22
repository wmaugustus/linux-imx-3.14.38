/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define GC0308_XCLK_MIN 6000000
#define GC0308_XCLK_MAX 24000000

enum gc0308_mode {
	gc0308_mode_MIN = 0,
	gc0308_mode_VGA_640_480 = 0,
	gc0308_mode_MAX = 1
};

enum gc0308_frame_rate {
	gc0308_15_fps,
	gc0308_30_fps
};

static int gc0308_framerates[] = {
	[gc0308_15_fps] = 15,
	[gc0308_30_fps] = 30,
};

struct gc0308_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct reg_value {
	u8 u8RegAddr;
	u8 u8Val;
};

struct gc0308_mode_info {
	enum gc0308_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct gc0308 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct gc0308_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct gc0308 gc0308_data;
static int pwn_gpio, rst_gpio;

static struct reg_value gc0308_setting_30fps_VGA_640_480[] = {

    {0xfe,0x80},
    {0xfe,0x00},
    {0x22,0x55},
    {0x03,0x02},
    {0x04,0x58},//12c
    {0x5a,0x56},
    {0x5b,0x40},
    {0x5c,0x4a},
    {0x22,0x57},

   // 50hz   24M MCLKauto
    {0x01 , 0x6a},  //32
    {0x02 , 0x70},
    {0x0f , 0x01},

    {0xe2 , 0x00},   //anti-flicker step [11:8]
    {0xe3 , 0x78},   //anti-flicker step [7:0]

    {0xe4 , 0x02},
    {0xe5 , 0x58},
    {0xe6 , 0x03},
    {0xe7 , 0x48},
    {0xe8 , 0x04},
    {0xe9 , 0xb0},
    {0xea , 0x05},
    {0xeb , 0xa0},

    {0x05,0x00},
    {0x06,0x00},
    {0x07,0x00},
    {0x08,0x02},
    {0x09,0x01},
    {0x0a,0xea},// ea
    {0x0b,0x02},
    {0x0c,0x88},
    {0x0d,0x02},
    {0x0e,0x02},
    {0x10,0x26},
    {0x11,0x0d},
    {0x12,0x2a},
    {0x13,0x00},
    {0x14,0x13}, //10 // h_v need mirror for preview
    {0x15,0x0a},
    {0x16,0x05},
    {0x17,0x01},
    {0x18,0x44},
    {0x19,0x44},
    {0x1a,0x2a},
    {0x1b,0x00},
    {0x1c,0x49},
    {0x1d,0x9a},
    {0x1e,0x61},
    {0x1f,0x16},
    {0x20,0xff},//ff
    {0x21,0xf8},//fa
    {0x22,0x57},
    {0x24,0xa2},    // output format
    {0x25,0x0f},
#ifdef CONFIG_ARCH_MESON3
    {0x26,0x01}, //03
#else
    {0x26,0x03}, //03
#endif
    {0x2f,0x01},
    {0x30,0xf7},
    {0x31,0x50},
    {0x32,0x00},
    {0x39,0x04},
    {0x3a,0x20},
    {0x3b,0x20},
    {0x3c,0x02},
    {0x3d,0x02}, //0x00
    {0x3e,0x02},
    {0x3f,0x02},
    {0x50,0x14}, // 0x14
    {0x53,0x80},
    {0x54,0x87},
    {0x55,0x87},
    {0x56,0x80},

    {0x57,0x7a},// r ratio
    {0x58,0x7e},// g ratio
    {0x59,0x84},//b ratio

    {0x8b,0x10},
    {0x8c,0x10},
    {0x8d,0x10},
    {0x8e,0x10},
    {0x8f,0x10},
    {0x90,0x10},
    {0x91,0x3c},
    {0x92,0x50},
    {0x5d,0x12},
    {0x5e,0x1a},
    {0x5f,0x24},
    {0x60,0x07},
    {0x61,0x15},
    {0x62,0x08}, // 0x08
    {0x64,0x03},  // 0x03
    {0x66,0xe8},
    {0x67,0x86},
    {0x68,0xa2},
    {0x69,0x18},
    {0x6a,0x0f},
    {0x6b,0x00},
    {0x6c,0x5f},
    {0x6d,0x8f},
    {0x6e,0x55},
    {0x6f,0x38},
    {0x70,0x15},
    {0x71,0x33}, // low light startion
    {0x72,0xdc},
    {0x73,0x80},
    {0x74,0x02},
    {0x75,0x3f},
    {0x76,0x02},
    {0x77,0x45}, // 0x47 //0x54
    {0x78,0x88},
    {0x79,0x81},
    {0x7a,0x81},
    {0x7b,0x22},
    {0x7c,0xff},
///////CC/////
    {0x93,0x42},  //0x48
    {0x94,0x00},
    {0x95,0x0c},//04
    {0x96,0xe0},
    {0x97,0x46},
    {0x98,0xf3},

    {0xb1,0x40},// startion
    {0xb2,0x40},
    {0xb3,0x3c}, //0x40  contrast
    {0xb5,0x00}, //
    {0xb6,0xe0},
    {0xbd,0x3C},
    {0xbe,0x36},
    {0xd0,0xCb},//c9
    {0xd1,0x10},
    {0xd2,0x90},
    {0xd3,0x50},//88
    {0xd5,0xF2},
    {0xd6,0x10},
    {0xdb,0x92},
    {0xdc,0xA5},
    {0xdf,0x23},
    {0xd9,0x00},
    {0xda,0x00},
    {0xe0,0x09},
    {0xed,0x04},
    {0xee,0xa0},
    {0xef,0x40},
    {0x80,0x03},

    {0x9F, 0x10},//     case 3:
    {0xA0, 0x20},
    {0xA1, 0x38},
    {0xA2, 0x4E},
    {0xA3, 0x63},
    {0xA4, 0x76},
    {0xA5, 0x87},
    {0xA6, 0xA2},
    {0xA7, 0xB8},
    {0xA8, 0xCA},
    {0xA9, 0xD8},
    {0xAA, 0xE3},
    {0xAB, 0xEB},
    {0xAC, 0xF0},
    {0xAD, 0xF8},
    {0xAE, 0xFD},
    {0xAF, 0xFF},

    {0xc0,0x00},
    {0xc1,0x14},
    {0xc2,0x21},
    {0xc3,0x36},
    {0xc4,0x49},
    {0xc5,0x5B},
    {0xc6,0x6B},
    {0xc7,0x7B},
    {0xc8,0x98},
    {0xc9,0xB4},
    {0xca,0xCE},
    {0xcb,0xE8},
    {0xcc,0xFF},
    {0xf0,0x02},
    {0xf1,0x01},
    {0xf2,0x01},
    {0xf3,0x30},
    {0xf9,0x9f},
    {0xfa,0x78},
    {0xfe,0x01},
    {0x00,0xf5},
    {0x02,0x20},
    {0x04,0x10},
    {0x05,0x10},
    {0x06,0x20},
    {0x08,0x15},
    {0x0a,0xa0},
    {0x0b,0x64},
    {0x0c,0x08},
    {0x0e,0x4C},
    {0x0f,0x39},
    {0x10,0x41},
    {0x11,0x37},
    {0x12,0x24},
    {0x13,0x39},
    {0x14,0x45},
    {0x15,0x45},
    {0x16,0xc2},
    {0x17,0xA8},
    {0x18,0x18},
    {0x19,0x55},
    {0x1a,0xd8},
    {0x1b,0xf5},

    {0x1c,0x60}, //r gain limit

    {0x70,0x40},
    {0x71,0x58},
    {0x72,0x30},
    {0x73,0x48},
    {0x74,0x20},
    {0x75,0x60},
    {0x77,0x20},
    {0x78,0x32},
    {0x30,0x03},
    {0x31,0x40},
    {0x32,0x10},
    {0x33,0xe0},
    {0x34,0xe0},
    {0x35,0x00},
    {0x36,0x80},
    {0x37,0x00},
    {0x38,0x04},
    {0x39,0x09},
    {0x3a,0x12},
    {0x3b,0x1C},
    {0x3c,0x28},
    {0x3d,0x31},
    {0x3e,0x44},
    {0x3f,0x57},
    {0x40,0x6C},
    {0x41,0x81},
    {0x42,0x94},
    {0x43,0xA7},
    {0x44,0xB8},
    {0x45,0xD6},
    {0x46,0xEE},
    {0x47,0x0d},
    {0xfe, 0x00},// update

    {0x10, 0x26},
    {0x11, 0x0d},  // fd,modified by mormo 2010/07/06
    {0x1a, 0x2a},  // 1e,modified by mormo 2010/07/06

    {0x1c, 0x49}, // c1,modified by mormo 2010/07/06
    {0x1d, 0x9a}, // 08,modified by mormo 2010/07/06
    {0x1e, 0x61}, // 60,modified by mormo 2010/07/06

    {0x3a, 0x20},

    {0x50, 0x14},  // 10,modified by mormo 2010/07/06
    {0x53, 0x80},
    {0x56, 0x80},

    {0x8b, 0x20}, //LSC
    {0x8c, 0x20},
    {0x8d, 0x20},
    {0x8e, 0x14},
    {0x8f, 0x10},
    {0x90, 0x14},

    {0x94, 0x02},
    {0x95, 0x0c}, //0x07
    {0x96, 0xe0},

    {0xb1, 0x40}, // YCPT
    {0xb2, 0x40},
    {0xb3, 0x3c},
    {0xb6, 0xe0},

    {0xd0, 0xcb}, // AECT  c9,modifed by mormo 2010/07/06
    {0xd3, 0x50}, // 80,modified by mormor 2010/07/06   58

    {0xf2, 0x02},
    {0xf7, 0x12},
    {0xf8, 0x0a},
    //Regsters of Page1
    {0xfe, 0x01},

    {0x02, 0x20},
    {0x04, 0x10},
    {0x05, 0x08},
    {0x06, 0x20},
    {0x08, 0x0a},

    {0x0e, 0x44},
    {0x0f, 0x32},
    {0x10, 0x41},
    {0x11, 0x37},
    {0x12, 0x22},
    {0x13, 0x19},
    {0x14, 0x44},
    {0x15, 0x44},

    {0x19, 0x50},
    {0x1a, 0xd8},

    {0x32, 0x10},

    {0x35, 0x00},
    {0x36, 0x80},
    {0x37, 0x00},
    //-----------Update the registers end---------//
    {0xfe, 0x00},
    {0xfe, 0x00},
    {0xff, 0xff},

};

static struct reg_value gc0308_default_setting[] = {
    {0xfe,0x00},
    //MCLK=24MHz 10fps
    {0x0f,0x05},     //0x00                                        
    {0x01,0xe1},   //0x6a                                        
    {0x02,0x70},   //0x70                                        
    {0xe2,0x00},
    {0xe3,0x96},
    {0xe4,0x02},
    {0xe5,0x58},
    {0xe6,0x02},
    {0xe7,0x58},
    {0xe8,0x02},
    {0xe9,0x58},
    {0xea,0x0e},
    {0xeb,0xa6},
    {0xfe,0x00},                                    
    {0xec,0x20},                                           
    {0x05,0x00},                                           
    {0x06,0x00},                                           
    {0x07,0x00},                                           
    {0x08,0x00},                                           
    {0x09,0x01},                                           
    {0x0a,0xe8},                                           
    {0x0b,0x02},                                           
    {0x0c,0x88},                                           
    {0x0d,0x02},                                           
    {0x0e,0x02},                                           
    {0x10,0x26},                                           
    {0x11,0x0d},                                           
    {0x12,0x2a},                                           
    {0x13,0x00},                                           
    {0x14,0x11},                                           
    {0x15,0x0a},                                           
    {0x16,0x05},                                           
    {0x17,0x01},                                           
    {0x18,0x44},                                           
    {0x19,0x44},                                           
    {0x1a,0x2a},                                           
    {0x1b,0x00},                                           
    {0x1c,0x49},                                           
    {0x1d,0x9a},                                           
    {0x1e,0x61},                                           
    {0x1f,0x00},  //pad drv <=24MHz, use 0x00 is ok
    {0x20,0x7f},                                           
    {0x21,0xfa},                                           
    {0x22,0x57},                                           
    {0x24,0xa2},    //YCbYCr                                 
    {0x25,0x0f},                                           
    {0x26,0x03}, // 0x01        
    {0x28,0x00},       
    {0x2d,0x0a},                      
    {0x2f,0x01},                                           
    {0x30,0xf7},                                           
    {0x31,0x50},                                           
    {0x32,0x00},   
    {0x33,0x28},    
    {0x34,0x2a},    
    {0x35,0x28},                                            
    {0x39,0x04},                                           
    {0x3a,0x20},                                           
    {0x3b,0x20},                                           
    {0x3c,0x00},                                           
    {0x3d,0x00},                                           
    {0x3e,0x00},                                           
    {0x3f,0x00},                                           
    {0x50,0x14}, // 0x14  
    {0x52,0x41},                                  
    {0x53,0x80},                                           
    {0x54,0x80},                                           
    {0x55,0x80},                                           
    {0x56,0x80},                                           
    {0x8b,0x20},                                           
    {0x8c,0x20},                                           
    {0x8d,0x20},                                           
    {0x8e,0x14},                                           
    {0x8f,0x10},                                           
    {0x90,0x14},                                           
    {0x91,0x3c},                                           
    {0x92,0x50},   
    //{0x8b,0x10},                                           
    //{0x8c,0x10},                                           
    //{0x8d,0x10},                                           
    //{0x8e,0x10},                                           
    //{0x8f,0x10},                                           
    //{0x90,0x10},                                           
    //{0x91,0x3c},                                           
    //{0x92,0x50},                                         
    {0x5d,0x12},                                           
    {0x5e,0x1a},                                           
    {0x5f,0x24},                                           
    {0x60,0x07},                                           
    {0x61,0x15},                                           
    {0x62,0x08}, // 0x08                                   
    {0x64,0x03},  // 0x03                                  
    {0x66,0xe8},                                           
    {0x67,0x86},                                           
    {0x68,0x82},                                           
    {0x69,0x18},                                           
    {0x6a,0x0f},                                           
    {0x6b,0x00},                                           
    {0x6c,0x5f},                                           
    {0x6d,0x8f},                                           
    {0x6e,0x55},                                           
    {0x6f,0x38},                                           
    {0x70,0x15},                                           
    {0x71,0x33},                                           
    {0x72,0xdc},                                           
    {0x73,0x00},                                           
    {0x74,0x02},                                           
    {0x75,0x3f},                                           
    {0x76,0x02},                                           
    {0x77,0x38}, // 0x47                                   
    {0x78,0x88},                                           
    {0x79,0x81},                                           
    {0x7a,0x81},                                           
    {0x7b,0x22},                                           
    {0x7c,0xff},                                          
    {0x93,0x48},  //color matrix default                                          
    {0x94,0x02},                                           
    {0x95,0x07},                                           
    {0x96,0xe0},                                           
    {0x97,0x40},                                           
    {0x98,0xf0},                                     
    {0xb1,0x40},                                           
    {0xb2,0x40},                                           
    {0xb3,0x40}, //0x40                                    
    {0xb6,0xe0},                                           
    {0xbd,0x38},                                           
    {0xbe,0x36},                                           
    {0xd0,0xCB},                                           
    {0xd1,0x10},                                           
    {0xd2,0x90},                                           
    {0xd3,0x48},                                           
    {0xd5,0xF2},                                           
    {0xd6,0x16},                                           
    {0xdb,0x92},                                           
    {0xdc,0xA5},                                           
    {0xdf,0x23},                                           
    {0xd9,0x00},                                           
    {0xda,0x00},                                           
    {0xe0,0x09},                                           
    {0xed,0x04},                                           
    {0xee,0xa0},                                           
    {0xef,0x40},                                           
    {0x80,0x03},                                           
                                                              
    {0x9F,0x10},                                           
    {0xA0,0x20},                                           
    {0xA1,0x38},                                           
    {0xA2,0x4e},                                           
    {0xA3,0x63},                                           
    {0xA4,0x76},                                           
    {0xA5,0x87},                                           
    {0xA6,0xa2},                                           
    {0xA7,0xb8},                                           
    {0xA8,0xca},                                           
    {0xA9,0xd8},                                           
    {0xAA,0xe3},                                           
    {0xAB,0xeb},                                           
    {0xAC,0xf0},                                           
    {0xAD,0xF8},                                           
    {0xAE,0xFd},                                           
    {0xAF,0xFF},                                                     
                                                               
    {0xc0,0x00},                                           
    {0xc1,0x10},                                           
    {0xc2,0x1c},                                           
    {0xc3,0x30},                                           
    {0xc4,0x43},                                           
    {0xc5,0x54},                                           
    {0xc6,0x65},                                           
    {0xc7,0x75},                                           
    {0xc8,0x93},                                           
    {0xc9,0xB0},                                           
    {0xca,0xCB},                                           
    {0xcb,0xE6},                                           
    {0xcc,0xFF},                                           
    {0xf0,0x02},                                           
    {0xf1,0x01},                                           
    {0xf2,0x02},                                           
    {0xf3,0x30},     
    {0xf7,0x12}, 
    {0xf8,0x0a},                                       
    {0xf9,0x9f},                                           
    {0xfa,0x78},                                           
    {0xfe,0x01},                                           
    {0x00,0xf5},                                           
    {0x02,0x20},                                           
    {0x04,0x10},                                           
    {0x05,0x08},                                           
    {0x06,0x20},                                           
    {0x08,0x0a},                                           
    {0x0a,0xa0},                                           
    {0x0b,0x60},                                           
    {0x0c,0x08},                                           
    {0x0e,0x44},                                           
    {0x0f,0x32},                                           
    {0x10,0x41},                                           
    {0x11,0x37},                                           
    {0x12,0x22},                                           
    {0x13,0x19},                                           
    {0x14,0x44},                                           
    {0x15,0x44},                                           
    {0x16,0xc2},                                           
    {0x17,0xA8},                                           
    {0x18,0x18},                                           
    {0x19,0x50},                                           
    {0x1a,0xd8},                                           
    {0x1b,0xf5},                                           
    {0x70,0x40},                                           
    {0x71,0x58},                                           
    {0x72,0x30},                                           
    {0x73,0x48},                                           
    {0x74,0x20},                                           
    {0x75,0x60},                                           
    {0x77,0x20},                                           
    {0x78,0x32},                                           
    {0x30,0x03},                                           
    {0x31,0x40},                                           
    {0x32,0x10},                                           
    {0x33,0xe0},                                           
    {0x34,0xe0},                                           
    {0x35,0x00},                                           
    {0x36,0x80},                                           
    {0x37,0x00},                                           
    {0x38,0x04},                                           
    {0x39,0x09},                                           
    {0x3a,0x12},                                           
    {0x3b,0x1C},                                           
    {0x3c,0x28},                                           
    {0x3d,0x31},                                           
    {0x3e,0x44},                                           
    {0x3f,0x57},                                           
    {0x40,0x6C},                                           
    {0x41,0x81},                                           
    {0x42,0x94},                                           
    {0x43,0xA7},                                           
    {0x44,0xB8},                                           
    {0x45,0xD6},                                           
    {0x46,0xEE},                                           
    {0x47,0x0d},   
    {0x62,0xf7},
    {0x63,0x68},
    {0x64,0xd3},
    {0x65,0xd3},
    {0x66,0x60},
    {0xfe,0x00},              
};

static struct gc0308_mode_info gc0308_mode_info_data[2][gc0308_mode_MAX + 1] = {
    {
        {gc0308_mode_VGA_640_480, 640, 480, 
         gc0308_default_setting, 
         ARRAY_SIZE(gc0308_default_setting)},
    },
    {
        {gc0308_mode_VGA_640_480, 640, 480, 
         gc0308_default_setting, 
         ARRAY_SIZE(gc0308_default_setting)},
    },
};

static int gc0308_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int gc0308_remove(struct i2c_client *client);

static s32 gc0308_read_reg(u8 reg, u8 *val);
static s32 gc0308_write_reg(u8 reg, u8 val);

static const struct i2c_device_id gc0308_id[] = {
	{"gc0308", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, gc0308_id);

static struct i2c_driver gc0308_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "gc0308",
		  },
	.probe  = gc0308_probe,
	.remove = gc0308_remove,
	.id_table = gc0308_id,
};

static const struct gc0308_datafmt gc0308_colour_fmts[] = {
	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct gc0308 *to_gc0308(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct gc0308, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct gc0308_datafmt
			*gc0308_find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gc0308_colour_fmts); i++)
		if (gc0308_colour_fmts[i].code == code)
			return gc0308_colour_fmts + i;

	return NULL;
}

static inline void gc0308_power_down(int enable)
{
	gpio_set_value_cansleep(pwn_gpio, enable);

	msleep(2);
}

static inline void gc0308_reset(void)
{
	/* camera reset */
	gpio_set_value_cansleep(rst_gpio, 1);

	/* camera power down */
	gpio_set_value_cansleep(pwn_gpio, 1);
	msleep(5);
	gpio_set_value_cansleep(pwn_gpio, 0);
	msleep(5);
	gpio_set_value_cansleep(rst_gpio, 0);
	msleep(1);
	gpio_set_value_cansleep(rst_gpio, 1);
	msleep(5);
	gpio_set_value_cansleep(pwn_gpio, 1);
}

static s32 gc0308_write_reg(u8 reg, u8 val)
{
    u8 au8Buf[3] = {0};

    au8Buf[0] = reg;
    au8Buf[1] = val;

    if (i2c_master_send(gc0308_data.i2c_client, au8Buf, 2) < 0) {
        pr_err("%s:write reg error:reg=%x,val=%x\n",
            __func__, reg, val);
        return -1;
    }


    return 0;
}

static s32 gc0308_read_reg(u8 reg, u8 *val)
{
    u8 au8RegBuf[2] = {0};
    u8 u8RdVal = 0;

    au8RegBuf[0] = reg;


    if (1 != i2c_master_send(gc0308_data.i2c_client, au8RegBuf, 1)) {
        pr_err("%s:write reg error:reg=%x\n",
                __func__, reg);
        return -1;
    }

    if (1 != i2c_master_recv(gc0308_data.i2c_client, &u8RdVal, 1)) {
        pr_err("%s:read reg error:reg=%x,val=%x\n",
                __func__, reg, u8RdVal);
        return -1;
    }

    *val = u8RdVal;

    return u8RdVal;
}

/* download gc0308 settings to sensor through i2c */
static int gc0308_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{	
	register u8 RegAddr = 0;
	register u8 Val = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		RegAddr = pModeSetting->u8RegAddr;
		Val = pModeSetting->u8Val;

		retval = gc0308_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;
	}
err:
	return retval;
}

static int gc0308_init_mode(void)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	pModeSetting = gc0308_default_setting;
	ArySize = ARRAY_SIZE(gc0308_default_setting);
	retval = gc0308_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	/* skip 9 vysnc: start capture at 10th vsync */
	msleep(300);

	gc0308_data.pix.width = 640;
	gc0308_data.pix.height = 480;
err:
	return retval;
}


/*!
 * gc0308_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int gc0308_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc0308 *sensor = to_gc0308(client);

	if (on)
		clk_enable(gc0308_data.sensor_clk);
	else
		clk_disable(gc0308_data.sensor_clk);

	sensor->on = on;

	return 0;
}

/*!
 * gc0308_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int gc0308_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc0308 *sensor = to_gc0308(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * gc0308_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int gc0308_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc0308 *sensor = to_gc0308(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum gc0308_frame_rate frame_rate;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = gc0308_15_fps;
		else if (tgt_fps == 30)
			frame_rate = gc0308_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			goto error;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

error:
	return ret;
}

static int gc0308_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct gc0308_datafmt *fmt = gc0308_find_datafmt(mf->code);

	if (!fmt) {
		mf->code	= gc0308_colour_fmts[0].code;
		mf->colorspace	= gc0308_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int gc0308_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc0308 *sensor = to_gc0308(client);

	/* MIPI CSI could have changed the format, double-check */
	if (!gc0308_find_datafmt(mf->code))
		return -EINVAL;

	gc0308_try_fmt(sd, mf);
	sensor->fmt = gc0308_find_datafmt(mf->code);

	return 0;
}

static int gc0308_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc0308 *sensor = to_gc0308(client);

	const struct gc0308_datafmt *fmt = sensor->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int gc0308_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(gc0308_colour_fmts))
		return -EINVAL;

	*code = gc0308_colour_fmts[index].code;
	return 0;
}

/*!
 * gc0308_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int gc0308_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > gc0308_mode_MAX)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width =
			max(gc0308_mode_info_data[0][fsize->index].width,
			    gc0308_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(gc0308_mode_info_data[0][fsize->index].height,
			    gc0308_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * gc0308_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int gc0308_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_frmivalenum *fival)
{
	int i, j, count;

	if (fival->index < 0 || fival->index > gc0308_mode_MAX)
		return -EINVAL;

	if (fival->width == 0 || fival->height == 0 ||
	    fival->pixel_format == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(gc0308_mode_info_data); i++) {
		for (j = 0; j < (gc0308_mode_MAX + 1); j++) {
			if (fival->pixel_format == gc0308_data.pix.pixelformat
			 && fival->width == gc0308_mode_info_data[i][j].width
			 && fival->height == gc0308_mode_info_data[i][j].height
			 && gc0308_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fival->index == (count - 1)) {
				fival->discrete.denominator =
						gc0308_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int gc0308_set_clk_rate(void)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = gc0308_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)GC0308_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)GC0308_XCLK_MIN);
	gc0308_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(gc0308_data.sensor_clk, gc0308_data.mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", gc0308_data.mclk);
	return ret;
}

/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(void)
{
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum gc0308_frame_rate frame_rate;
	int ret;

	gc0308_data.on = true;

	/* mclk */
	tgt_xclk = gc0308_data.mclk;

	/* Default camera frame rate is set in probe */
	tgt_fps = gc0308_data.streamcap.timeperframe.denominator /
		  gc0308_data.streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = gc0308_15_fps;
	else if (tgt_fps == 30)
		frame_rate = gc0308_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	ret = gc0308_init_mode();

	return ret;
}

static struct v4l2_subdev_video_ops gc0308_subdev_video_ops = {
	.g_parm = gc0308_g_parm,
	.s_parm = gc0308_s_parm,

	.s_mbus_fmt	= gc0308_s_fmt,
	.g_mbus_fmt	= gc0308_g_fmt,
	.try_mbus_fmt	= gc0308_try_fmt,
	.enum_mbus_fmt	= gc0308_enum_fmt,
	.enum_framesizes     = gc0308_enum_framesizes,
	.enum_frameintervals = gc0308_enum_frameintervals,
};

static struct v4l2_subdev_core_ops gc0308_subdev_core_ops = {
	.s_power	= gc0308_s_power,
};

static struct v4l2_subdev_ops gc0308_subdev_ops = {
	.core	= &gc0308_subdev_core_ops,
	.video	= &gc0308_subdev_video_ops,
};

static int get_device_id(void)
{
    u8 au8RegBuf[2] = {0};
    u8 u8RdVal = 0;
    au8RegBuf[0] = 0x00;

    if (1 != i2c_master_send(gc0308_data.i2c_client, au8RegBuf, 1)) {
        pr_err("%s:write reg error:reg=%x\n",
                __func__, 0xfb);
        return -1;
    }

    if (1 != i2c_master_recv(gc0308_data.i2c_client, &u8RdVal, 1)) {
        pr_err("%s:read reg error:reg=%x,val=%x\n",
                __func__, 0xfb, u8RdVal);
        return -1;
    }

    printk(KERN_INFO "Chip ID is %x\n", u8RdVal);

    return u8RdVal;
}

static int get_pix_format(void)
{
    u8 au8RegBuf[2] = {0};
    u8 u8RdVal = 0;
    au8RegBuf[0] = 0x24;

    if (1 != i2c_master_send(gc0308_data.i2c_client, au8RegBuf, 1)) {
        pr_err("%s:write reg error:reg=%x\n",
                __func__, 0xfb);
        return -1;
    }

    if (1 != i2c_master_recv(gc0308_data.i2c_client, &u8RdVal, 1)) {
        pr_err("%s:read reg error:reg=%x,val=%x\n",
                __func__, 0xfb, u8RdVal);
        return -1;
    }

    printk(KERN_INFO "Pix format is %x\n", u8RdVal);

    return u8RdVal;
}
/*!
 * gc0308 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int gc0308_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;

	/* gc0308 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"gc0308_pwdn");
	if (retval < 0)
		return retval;

	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
					"gc0308_reset");
	if (retval < 0)
		return retval;

	/* Set initial values for the sensor struct. */
	memset(&gc0308_data, 0, sizeof(gc0308_data));
	gc0308_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(gc0308_data.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(gc0308_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&gc0308_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(gc0308_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(gc0308_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	/* Set mclk rate before clk on */
	gc0308_set_clk_rate();

	clk_prepare_enable(gc0308_data.sensor_clk);

	gc0308_data.io_init = gc0308_reset;
	gc0308_data.i2c_client = client;
	gc0308_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	gc0308_data.pix.width = 640;
	gc0308_data.pix.height = 480;
	gc0308_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	gc0308_data.streamcap.capturemode = 0;
	gc0308_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	gc0308_data.streamcap.timeperframe.numerator = 1;

	gc0308_reset();

	gc0308_power_down(0);

	if( get_device_id() == -1 )
    {
        clk_disable_unprepare(gc0308_data.sensor_clk);
        printk(KERN_ERR "get_device_id: error, not GC0308");
        return -1;
    }

    get_pix_format();

	retval = init_device();
	if (retval < 0) {
		clk_disable_unprepare(gc0308_data.sensor_clk);
		pr_warning("camera gc0308 init failed\n");
		gc0308_power_down(1);
		return retval;
	}

	clk_disable(gc0308_data.sensor_clk);

	v4l2_i2c_subdev_init(&gc0308_data.subdev, client, &gc0308_subdev_ops);

	retval = v4l2_async_register_subdev(&gc0308_data.subdev);
	if (retval < 0)
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);

	pr_info("camera gc0308, is found\n");
	return retval;
}

/*!
 * gc0308 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int gc0308_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);

	clk_unprepare(gc0308_data.sensor_clk);

	gc0308_power_down(1);

	return 0;
}

module_i2c_driver(gc0308_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("GC0308 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
