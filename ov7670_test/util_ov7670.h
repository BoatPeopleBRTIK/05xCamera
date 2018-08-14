/****************************************************************************
*
* Copyright 2017 Samsung Electronics All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
* either express or implied. See the License for the specific
* language governing permissions and limitations under the License.
*
****************************************************************************/

#ifndef __UTIL_OV7670_H__
#define __UTIL_OV7670_H__

#define DEBUG_TIMING 1

//=====================================================
// Generate XCLK for OV7670
//=====================================================
// GPIO access method
#define GPIO_IOCTL 1
#define GPIO_REG 2

// CLK output
#define USE_PWM_AS_CLK 0
#define USE_GPIO_AS_CLK 1

#define PWM_PIN 1 // A053_CON704_XPWMOUT1
#define PWM_FREQUENCY 6000000
#define PWM_DUTY_CYCLE b16HALF

#define GPIO_PIN_32 32
#define GPIO_DEVPATH "/dev/gpio32"
#define GPIO_32_BIT 3
#define GPIO_32_MASK (1<<GPIO_32_BIT)

//=====================================================
// SCCB communication
//=====================================================

#define I2C_PORT 1 //changed from 0 to 1
#define OV7670_I2C_FREQ 100000 //changed to 400000
#define OV7670_I2C_ADDRLEN 7
#if !defined DEBUG_USING_MPU9250
#define OV7670_I2C_ADDRESS 0X21
#else
#define OV7670_I2C_ADDRESS 0x68  // debug I2C comm using MPU9250
#endif 

#define RETRY_TIMES 3

//=====================================================
// Image format
//=====================================================
// Format
#define USE_QQVGA 1

#if (defined(USE_QQVGA) && USE_QQVGA)
    #define NUM_ROW 160
    #define NUM_COL 120
#else // default size
    #define NUM_ROW 160
    #define NUM_COL 120
#endif

#define BUFFER_SIZE (NUM_ROW * NUM_COL)

typedef enum {
    QQVGA = 1,
    BAYER_RAW = 2,
} mode;


//=====================================================
// Image timing
//=====================================================
// Frame
// The rising edge of the VSYNC signals the start of a frame
#define VSYNC_PIN 46

// Line
// The rising edge of HREF signals the start of a line, and
// the falling edge of HREF signals the end of the line.
#define HREF_PIN 47

// Pixel
// The rising edge of PCLK signals the start of a byte
#define PCLK_PIN 48

// GPIO input
// D0-D7 must be sampled at the rising edge of the PCLK signal
#define D7_PIN 41 // GPG1[4] Base_address: 0x800400A4
#define D6_PIN 40 // GPG1[3] Base_address: 0x800400A4
#define D5_PIN 39 // GPG1[2] Base_address: 0x800400A4
#define D4_PIN 38 // GPG1[1] Base_address: 0x800400A4
#define D3_PIN 37 // GPG1[0] Base_address: 0x800400A4
#define D2_PIN 32 // GPG0[3] Base_address: 0x80040084
#define D1_PIN 31 // GPG0[2] Base_address: 0x80040084
#define D0_PIN 30 // GPG0[1] Base_address: 0x80040084

#define D7_D3_DATA_MASK 0x1F
#define D2_D0_DATA_MASK 0x7

#if (defined(DEBUG_TIMING) && DEBUG_TIMING)
// debug pin
#define DEBUG_PIN_VSYNC 58  // GPIO_XEINT1:   GPA0[1] 0x80040104
#define DEBUG_PIN_HREF  59  // GPIO_XEINT2:   GPA0[2] 0x80040104
#define DEBUG_PIN_PCLK  50  // GPIO_XGPIO21:  GPG2[5] 0x800400C4


#define DEBUG_PIN_VSYNC_MASK  1<<1
#define DEBUG_PIN_HREF_MASK   1<<2  
#define DEBUG_PIN_PCLK_MASK   1<<5
#endif


#define D7_DEVPATH "/dev/gpio41"
#define D6_DEVPATH "/dev/gpio40"
#define D5_DEVPATH "/dev/gpio39"
#define D4_DEVPATH "/dev/gpio38"
#define D3_DEVPATH "/dev/gpio37"
#define D2_DEVPATH "/dev/gpio32"
#define D1_DEVPATH "/dev/gpio31"
#define D0_DEVPATH "/dev/gpio30"
#define VSYNC_DEVPATH "/dev/gpio46"
#define HREF_DEVPATH  "/dev/gpio47"
#define PCLK_DEVPATH  "/dev/gpio48"


///////////////////////////////////////////////////////////////////////////////
////////////////////////////// GPG Macros /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


//#define GPG1_CON_ADDR				(0x800400A0)	//GPG1_CON
//#define GPG1_DAT_ADDR				(0x800400A4)	//GPG1_DAT

#define GPG1_CON_ADDR				(0x80040080)    //GPG0_CON
#define GPG1_DAT_ADDR				(0x80040084)	//GPG0_DAT

#define GPA0_CON_ADDR				(0x80040100)
#define GPA0_DAT_ADDR				(0x80040104)

#define GPG1_CON				(*(volatile unsigned int*)GPG1_CON_ADDR)
#define GPG1_DAT				(*(volatile unsigned int*)GPG1_DAT_ADDR)

#define GPA0_CON				(*(volatile unsigned int*)GPA0_CON_ADDR)
#define GPA0_DAT				(*(volatile unsigned int*)GPA0_DAT_ADDR)

#define PIXEL_READ				(GPG1_DAT & 0x0F)
#define PIXEL_SHIFT(a)				(a << 4)

#define HS_BIT					(GPA0_DAT & 0x4)
#define VS_BIT					(GPA0_DAT & 0x2)
#define PCLK_BIT				(GPA0_DAT & 0x1)

// #define QQVGA
#define QVGA

#ifdef QVGA
	#define H_RES					(320*2)  //QVGA 
	#define V_RES					(120*2)
	#define MAX_FRAMES				(1)
	#define BYTES_PER_LINE				(640) 	 //QVGA
	#define LINES					(240)    //QVGA
#endif

#ifdef QQVGA
	#define H_RES					(320)  //QVGA 
	#define V_RES					(120)
	#define MAX_FRAMES				(1)
	#define BYTES_PER_LINE				(320) 	 //QVGA
	#define LINES					(120)    //QVGA
#endif

#define PRINT					(23)

//=====================================================
// Device register map
//=====================================================
#if (defined(DEBUG_USING_MPU9250) && DEBUG_USING_MPU9250)

// Registers Addresses ******************************************************/
typedef enum {
    // register name = address
    REG_WHO_AM_I_MPU9250 = 0x75,// Should return 0x71
    REG_PWR_MGMT_1 = 0x6B,// Device defaults to the SLEEP mode
    REG_SMPLRT_DIV = 0x19,
    REG_CONFIG = 0x1A,
    REG_GYRO_CONFIG = 0x1B,
    REG_ACCEL_CONFIG = 0x1C,
    REG_ACCEL_CONFIG2 = 0x1D,
    REG_INT_PIN_CFG = 0x37,
    REG_INT_ENABLE = 0x38,
    REG_INT_STATUS = 0x3A,
} OV7670_REG;

typedef struct {
    OV7670_REG addr;
    uint16_t val;
    unsigned int delay;
} t_codec_init_script_entry;

t_codec_init_script_entry ov7670_init_script[] = {
    { REG_SMPLRT_DIV, 5, 0 },
    { REG_CONFIG, 6, 0 },
    { REG_GYRO_CONFIG, 7, 0 },
    { REG_ACCEL_CONFIG, 8 , 0 },
};

/*
// init regs
#define REG_WHO_AM_I_MPU9250    0x75 // Should return 0x71
#define REG_PWR_MGMT_1          0x6B // Device defaults to the SLEEP mode
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_CONFIG2       0x1D
#define REG_INT_PIN_CFG         0x37
#define REG_INT_ENABLE          0x38
#define REG_INT_STATUS          0x3A
#define REG_CONFIG              0x1A
*/

#elif (!defined(DEBUG_USING_MPU9250) || !DEBUG_USING_MPU9250)

#define COM1_CCIR656    		0x40	// CCIR656 enable
#define COM2_SSLEEP	    		0x10	// Soft sleep mode 
#define COM3_SWAP	    		0x40	// Byte swap 
#define COM3_SCALEEN			0x08	// Enable scaling 
#define COM3_DCWEN	    		0x04	// Enable downsamp/crop/window 
#define CLK_EXT		    		0x40	// Use external clock directly 
#define CLK_SCALE	    		0x3f	// Mask for internal clock scale 
#define COM7_RESET	    		0x80	// Register reset 
#define COM7_FMT_MASK			0x38
#define COM7_FMT_VGA			0x00
#define COM7_FMT_CIF			0x20	// CIF format 
#define COM7_FMT_QVGA			0x10	// QVGA format 
#define COM7_FMT_QCIF			0x08	// QCIF format 
#define COM7_RGB	    		0x04	// bits 0 and 2 - RGB format 
#define COM7_YUV	    		0x00	// YUV 
#define COM7_BAYER	    		0x01	// Bayer format 
#define COM7_PBAYER	    		0x05	// "Processed bayer" 
#define COM8_FASTAEC			0x80	// Enable fast AGC/AEC 
#define COM8_AECSTEP			0x40	// Unlimited AEC step size 
#define COM8_BFILT	    		0x20	// Band filter enable 
#define COM8_AGC	    		0x04	// Auto gain enable 
#define COM8_AWB	    		0x02	// White balance enable 
#define COM8_AEC	    		0x01	// Auto exposure enable 
#define COM10_HSYNC	    		0x40	// HSYNC instead of HREF 
#define COM10_PCLK_HB			0x20	// Suppress PCLK on horiz blank 
#define COM10_HREF_REV			0x08	// Reverse HREF 
#define COM10_VS_LEAD			0x04	// VSYNC on clock leading edge 
#define COM10_VS_NEG			0x02	// VSYNC negative 
#define COM10_VS_POS			0x00	// VSYNC positive
#define COM10_HS_NEG			0x01	// HSYNC negative 
#define MVFP_MIRROR	    		0x20	// Mirror image 
#define MVFP_FLIP	    		0x10	// Vertical flip 
#define TSLB_YLAST	    		0x04	// UYVY or VYUY - see com13 
#define COM11_NIGHT	    		0x80	// NIght mode enable 
#define COM11_NMFR	    		0x60	// Two bit NM frame rate 
#define COM11_HZAUTO			0x10	// Auto detect 50/60 Hz 
#define COM11_50HZ	    		0x08	// Manual 50Hz select 
#define COM11_EXP	    		0x02
#define COM12_HREF	    		0x80	// HREF always 
#define COM13_GAMMA	    		0x80	// Gamma enable 
#define COM13_UVSAT	    		0x40	// UV saturation auto adjustment 
#define COM13_UVSWAP			0x01	// V before U - w/TSLB 
#define COM14_DCWEN	    		0x10	// DCW/PCLK-scale enable 
#define COM15_R10F0	   		0x00	// Data range 10 to F0 
#define COM15_R01FE	    		0x80	//			01 to FE 
#define COM15_R00FF	    		0xc0	//			00 to FF 
#define COM15_RGB565			0x10	// RGB565 output 
#define COM15_RGB555			0x30	// RGB555 output 
#define COM16_AWBGAIN			0x08	// AWB gain enable 
#define COM17_AECWIN			0xc0	// AEC window - must match COM4 
#define COM17_CBAR	    		0x08	// DSP Color bar 
#define CMATRIX_LEN     		   6
#define R76_BLKPCOR	    		0x80	// Black pixel correction enable 
#define R76_WHTPCOR	    		0x40	// White pixel correction enable 
#define R444_ENABLE	    		0x02	// Turn on RGB444, overrides 5x5 
#define R444_RGBX	    		0x01	// Empty nibble at end 
#define COM7_FMT_CIF			0x20	// CIF format 
#define COM7_RGB	    		0x04	// bits 0 and 2 - RGB format 
#define COM7_YUV	    		0x00	// YUV 
#define COM7_BAYER	    		0x01	// Bayer format 
#define COM7_PBAYER	    		0x05	// "Processed bayer" 
#define COM10_VS_LEAD 			0x04	// VSYNC on clock leading edge 
#define COM11_50HZ	    		0x08	// Manual 50Hz select 
#define COM13_UVSAT	    		0x40	// UV saturation auto adjustment 
#define COM15_R01FE	    		0x80	//			01 to FE  


// Registers Addresses
typedef enum {
    // register name = address
    REG_GAIN = 			0x00,	// Gain lower 8 bits (rest in vref)
    REG_BLUE = 			0x01,	// blue gain
    REG_RED = 			0x02,	// red gain
    REG_VREF = 			0x03,	// Pieces of GAIN, VSTART, VSTOP
    REG_COM1 = 			0x04,	// Control 1
    REG_BAVE = 			0x05,	// U/B Average level
    REG_GbAVE = 		0x06,	// Y/Gb Average level
    REG_AECHH = 		0x07,	// AEC MS 5 bits
    REG_RAVE = 			0x08,	// V/R Average level
    REG_COM2 = 			0x09,	// Control 2
    REG_PID = 			0x0a,	// Product ID MSB
    REG_VER = 			0x0b,	// Product ID LSB
    REG_COM3 = 			0x0c,	// Control 3
    REG_COM4 = 			0x0d,	// Control 4
    REG_COM5 = 			0x0e,	// All "reserved"
    REG_COM6 = 			0x0f,	// Control 6
    REG_AECH = 			0x10,	// More bits of AEC value
    REG_CLKRC = 		0x11,	// Clock control
    REG_COM7 = 			0x12,	// Control 7
    REG_COM8 = 			0x13,	// Control 8
    REG_COM9 = 			0x14,	// Control 9- gain ceiling
    REG_COM10 = 		0x15,	// Control 10
    REG_HSTART = 		0x17,	// Horiz start high bits
    REG_HSTOP = 		0x18,	// Horiz stop high bits
    REG_VSTART = 		0x19,	// Vert start high bits
    REG_VSTOP = 		0x1a,	// Vert stop high bits
    REG_PSHFT = 		0x1b,	// Pixel delay after HREF
    REG_MIDH = 			0x1c,	// Manuf. ID high
    REG_MIDL = 			0x1d,	// Manuf. ID low
    REG_MVFP = 			0x1e,	// Mirror / vflip
    REG_AEW = 			0x24,	// AGC upper limit
    REG_AEB = 			0x25,	// AGC lower limit
    REG_VPT = 			0x26,	// AGC/AEC fast mode op region
    REG_HSYST = 		0x30,	// HSYNC rising edge delay
    REG_HSYEN = 		0x31,	// HSYNC falling edge delay
    REG_HREF = 			0x32,	// HREF pieces
    REG_TSLB = 			0x3a,	// lots of stuff
    REG_COM11 = 		0x3b,	// Control 11
    REG_COM12 = 		0x3c,	// Control 12
    REG_COM13 = 		0x3d,	// Control 13
    REG_COM14 = 		0x3e,	// Control 14
    REG_EDGE = 			0x3f,	// Edge enhancement factor
    REG_COM15 = 		0x40,	// Control 15
    REG_COM16 = 		0x41,	// Control 16
    REG_COM17 = 		0x42,	// Control 17

    // This matrix defines how the colors are generated, must be
    // tweaked to adjust hue and saturation.
    // 
    // Order: v-red, v-green, v-blue, u-red, u-green, u-blue
    // They are nine-bit signed quantities, with the sign bit
    // stored in0x58.Sign for v-red is bit 0, and up from there.

    REG_CMATRIX_BASE = 		0x4f,
    REG_MTX1 = 			0x4f, // Matrix Coefficient 1
    REG_MTX2 = 			0x50, // Matrix Coefficient 2
    REG_MTX3 = 			0x51, // Matrix Coefficient 3
    REG_MTX4 = 			0x52, // Matrix Coefficient 4
    REG_MTX5 = 			0x53, // Matrix Coefficient 5
    REG_MTX6 = 			0x54, // Matrix Coefficient 6
    REG_MTXS = 			0x58, // Matrix Coefficient Sign
    REG_AWBC7 = 		0x59, // AWB Control 7
    REG_AWBC8 = 		0x5a, // AWB Control 8
    REG_AWBC9 = 		0x5b, // AWB Control 9
    REG_AWBC10 = 		0x5c, // AWB Control 10
    REG_AWBC11 = 		0x5d, // AWB Control 11
    REG_AWBC12 = 		0x5e, // AWB Control 12
    REG_CMATRIX_SIGN =  	0x58,
    REG_BRIGHT = 		0x55, // Brightness
    REG_CONTRAS = 		0x56, // Contrast control
    REG_GFIX = 			0x69, // Fix gain control
    REG_SCALING_XSC =		0x70, // Horizontal Scaling 
    REG_SCALING_YSC =		0x71, // Vertical Scaling 
    REG_SCALING_DCWCTR = 	0x72, // Scaling DCW Control
    REG_SCALING_PCLK_DIV = 	0x73, // Scaling Pixel Clock Divider
    REG_REG76 = 		0x76, // OV's name    
    REG_RGB444 = 		0x8c, // RGB 444 control
    REG_HAECC1 = 		0x9f, // Hist AEC/AGC control 1
    REG_HAECC2 = 		0xa0, // Hist AEC/AGC control 2
    REG_BD50MAX = 		0xa5, // 50hz banding step limit
    REG_HAECC3 = 		0xa6, // Hist AEC/AGC control 3
    REG_HAECC4 = 		0xa7, // Hist AEC/AGC control 4
    REG_HAECC5 = 		0xa8, // Hist AEC/AGC control 5
    REG_HAECC6 = 		0xa9, // Hist AEC/AGC control 6
    REG_HAECC7 = 		0xaa, // Hist AEC/AGC control 7
    REG_BD60MAX = 		0xab, // 60hz banding step limit    
    GGAIN = 			0x6a, // G Channel AWB Gain
    DBLV = 			0x6b,
    AWBCTR3 = 			0x6c, // AWB Control 3
    AWBCTR2 = 			0x6d, // AWB Control 2
    AWBCTR1 = 			0x6e, // AWB Control 1
    AWBCTR0 = 			0x6f, // AWB Control 0

} OV7670_REG;


//=====================================================
// Register settings
//=====================================================

typedef struct {
    OV7670_REG addr;
    uint16_t val;
    unsigned int delay;
} t_codec_init_script_entry;


t_codec_init_script_entry ov7670_reset[] = {

    { REG_COM7, 0x80, 1} 				// Camera Reset, Wait for 1ms after reset (suggested)

};

t_codec_init_script_entry ov7670_pclk_rate[] = {

    { REG_CLKRC, 0x0c, 0}

};

t_codec_init_script_entry ov7670_QQVGA_320[] = {

    { REG_COM10, COM10_VS_POS, 0 },
    { REG_COM3, 0x04, 0},  				// DCW Enable 
    { REG_COM14, 0x1a, 0},				// COM14[3] - Manual DCW adjustment (manual scaling enabled)
    { REG_SCALING_XSC, 0x3A, 0},
    { REG_SCALING_YSC, 0x35, 0},
    { REG_SCALING_DCWCTR, 0x22, 50},			// Horizontal and Vertical Downsampling by factor of 4	
    { REG_SCALING_PCLK_DIV, 0xf2, 50},			// Pclk divider bypassed, divider still set to 4, first 4 bits reserved still being set	
    { REG_HSTART,0x16, 0},
    { REG_HSTOP,0x04, 0},
    { REG_HREF,0xa4, 0},
    { REG_VSTART,0x02, 0},
    { REG_VSTOP,0x7a, 0},
    { REG_VREF,0x0a, 0},
    { REG_TSLB, 0xC2, 0}

};

t_codec_init_script_entry ov7670_QVGA_640_240[] = {

    { REG_COM10, COM10_VS_POS, 0 },
    { REG_COM3, 0x04, 0},  				// DCW Enable 
    { REG_COM14, 0x19, 0},				// COM14[3] - Manual DCW adjustment (manual scaling enabled)
    { REG_SCALING_XSC, 0x3A, 0},
    { REG_SCALING_YSC, 0x35, 0},
    { REG_SCALING_DCWCTR, 0x11, 50},			// Horizontal and Vertical Downsampling by factor of 2	
    { REG_SCALING_PCLK_DIV, 0xf1, 50},			// Pclk divider bypassed, divider still set to 2, first 4 bits reserved still being set	
    { REG_HSTART,0x16, 0},
    { REG_HSTOP,0x04, 0},
    { REG_HREF,0xa4, 0},
    { REG_VSTART,0x02, 0},
    { REG_VSTOP,0x7a, 0},
    { REG_VREF,0x0a, 0},
    { REG_TSLB, 0xC2, 0}

};

t_codec_init_script_entry ov7670_color[] = {

    ////////////////////// AEC and AGC Registers ////////////////////////////////

    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC, 0 },
    { REG_GAIN, 0, 0},
    { REG_AECH, 0, 0},
    { REG_COM4, 0x40, 0 }, 				
    { REG_COM9, 0x18, 0 }, 				// 4x gainREG_HAECC7 
    { REG_BD50MAX, 0x05, 0 },
    { REG_BD60MAX, 0x07, 0 },
    { REG_AEW, 0x95, 0 },
    { REG_AEB, 0x33, 0 },
    { REG_VPT, 0xe3, 0 },
    { REG_HAECC1, 0x78, 0 },
    { REG_HAECC2, 0x68, 0 },
    { 0xa1, 0x03, 0 }, 					
    { REG_HAECC3, 0xd8, 0 },
    { REG_HAECC4, 0xd8, 0 },
    { REG_HAECC5, 0xf0, 0 },
    { REG_HAECC6, 0x90, 0 },
    { REG_HAECC7, 0x94, 0 },
    { 0x30,0, 0 }, 					// HSYNC Rising Edge Delay
    { 0x31,0, 0 },					// HSYNC Falling Edge Delay
    { REG_COM5, 0x61, 0 },
    { REG_COM6, 0x4b, 0 },
    { 0x16, 0x02, 0 },   				///////////// disabled -  reserved //////////////////
    { REG_MVFP, 0x07, 0 },   				// Black Sun Enable

    ////////////////////// ADCCTR Registers ////////////////////////////////

    { 0x21, 0x02, 0 },
    { 0x22, 0x91, 0 },

    ////////////////////////////////////////////////////////////////////////

    { 0x29, 0x07, 0 },    				/////////// disabled - reserved //////////////////
    { 0x33, 0x0b, 0 },					// Array Current Control
    { REG_COM12, 0, 0 },				/// No HREF when VSYNC is low (chaned from 0x78 to 0)
    { 0x4d, 0x40, 0 },					// Dummy row position - dummy row inserted before active row
    { 0x4e, 0x20, 0 },					//////// disabled - reserved register ///////////////
    { REG_GFIX, 0, 0 },					
    { 0x74, 0x10, 0 },					// Digital Gain control by REG74[1:0], Digital gain manual control - bypass
    { 0x8d, 0x4f, 0 },					
    { 0x8e, 0, 0 },
    { 0x8f, 0, 0 },
    { 0x90, 0, 0 },
    { 0x91, 0, 0 },
    { 0x96, 0, 0 },
    { 0x9a, 0, 0 },
    { 0xb0, 0x84, 0 },
    { 0xb1, 0x0c, 0 },
    { 0xb3, 0x82, 0 },

    ////////////////////////// AWBC Control Registers /////////////////////

    { 0x43, 0x0a, 0 },
    { 0x44, 0xf0, 0 },
    { 0x45, 0x34, 0 },
    { 0x46, 0x58, 0 },
    { 0x47, 0x28, 0 },
    { 0x48, 0x3a, 0 },
    { 0x59, 0x88, 0 },
    { 0x5a, 0x88, 0 },
    { 0x5b, 0x44, 0 },
    { 0x5c, 0x67, 0 },
    { 0x5d, 0x49, 0 },
    { 0x5e, 0x0e, 0 },

    ////////////////////////// AWBCTR Control Registers /////////////////////
    
    { 0x6c, 0x0a, 0 },
    { 0x6d, 0x55, 0 },
    { 0x6e, 0x11, 0 },
    { 0x6f, 0x9e, 0 }, // it was 0x9F "9e for advance AWB"

    //////////////////////////////////////////////////////////////////////

    { 0x6a, 0x40, 0 },     				// G-Channel AWB Gain
    { REG_BLUE, 0x40, 0 },
    { REG_RED, 0x60, 0 },
    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC | COM8_AWB, 0 },

    //////////////////////////////////////////////////////////////////////

    // Matrix coefficients
    { 0x4f, 0x80, 0 },
    { 0x50, 0x80, 0 },
    { 0x51, 0, 0 },
    { 0x52, 0x22, 0 },
    { 0x53, 0x5e, 0 },
    { 0x54, 0x80, 0 },
    { 0x58, 0x9e, 0 },

    //////////////////////////////////////////////////////////////////////

    { REG_COM16, COM16_AWBGAIN, 0 }, // Edge enhancement, denoise
    { REG_EDGE, 0, 0 },
    { 0x75, 0x05, 0 },
    { REG_REG76, 0xe1, 0 }, // Pix correction, 
    { 0x77, 0x01, 0 },
    { REG_COM13, 0x08, 0 }, // No gamma,
    { 0x4b, 0x09, 0 },
    { 0xc9, 0x60, 0 },		//{REG_COM16, 0x38, 0 },
    { REG_COM11, COM11_EXP | COM11_HZAUTO, 0 },
    { 0xa4, 0x82, 0 }, //Was 0x88
    { 0x9d, 0x4c, 0 },
    { 0x9e, 0x3f, 0 },

    //////////////////////////////////////////////////////////////////////

};

t_codec_init_script_entry ov7670_QQVGA_TEST[] = {

////////////////////////////////////////////////////////////
	
    { REG_COM7, 0x80, 1}, 				// Camera Reset, Wait for 1ms after reset (suggested)

////////////////////////////////////////////////////////////

    { REG_COM10, COM10_VS_POS, 0 },
    { REG_COM3, 0x04, 0},  				// DCW Enable 
    { REG_COM14, 0x1a, 0},				// COM14[3] - Manual DCW adjustment (manual scaling enabled)
    { REG_SCALING_XSC, 0x3A, 0},
    { REG_SCALING_YSC, 0x35, 0},
    { REG_SCALING_DCWCTR, 0x22, 50},			// Horizontal and Vertical Downsampling by factor of 4	
    { REG_SCALING_PCLK_DIV, 0xf2, 50},			// Pclk divider bypassed, divider still set to 4, first 4 bits reserved still being set	
    { REG_HSTART,0x16, 0},
    { REG_HSTOP,0x04, 0},
    { REG_HREF,0xa4, 0},
    { REG_VSTART,0x02, 0},
    { REG_VSTOP,0x7a, 0},
    { REG_VREF,0x0a, 0},
    { REG_TSLB, 0xC2, 0},

    // AGC and AEC parameters.  Note we start by disabling those features,
    // then turn them only after tweaking the values.

    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC, 0 },
    { REG_GAIN, 0, 0 },
    { REG_AECH, 0, 0 },
    { REG_COM4, 0x40, 0 }, 				// magic reserved bit
    { REG_COM9, 0x18, 0 }, 				// 4x gainREG_HAECC7 + magic rsvd bit
    { REG_BD50MAX, 0x05, 0 },
    { REG_BD60MAX, 0x07, 0 },
    { REG_AEW, 0x95, 0 },
    { REG_AEB, 0x33, 0 },
    { REG_VPT, 0xe3, 0 },
    { REG_HAECC1, 0x78, 0 },
    { REG_HAECC2, 0x68, 0 },
    { 0xa1, 0x03, 0 }, 					// magic
    { REG_HAECC3, 0xd8, 0 },
    { REG_HAECC4, 0xd8, 0 },
    { REG_HAECC5, 0xf0, 0 },
    { REG_HAECC6, 0x90, 0 },
    { REG_HAECC7, 0x94, 0 },
    { 0x30,0, 0 }, 					// HSYNC Rising Edge Delay
    { 0x31,0, 0 },					// HSYNC Falling Edge Delay
    
   // Almost all of these are magic "reserved" values. 

    { REG_COM5, 0x61, 0 },
    { REG_COM6, 0x4b, 0 },

    { 0x16, 0x02, 0 },   				///////////// disabled -  reserved //////////////////

    { REG_MVFP, 0x07, 0 },   				// Black Sun Enable

    ////////////////////// ADCCTR Registers ////////////////////////////////

    { 0x21, 0x02, 0 },
    { 0x22, 0x91, 0 },

    ////////////////////////////////////////////////////////////////////////

    { 0x29, 0x07, 0 },    				/////////// disabled - reserved //////////////////
    { 0x33, 0x0b, 0 },					// Array Current Control
    { REG_COM12, 0, 0 },				/// No HREF when VSYNC is low (chaned from 0x78 to 0)
    { 0x4d, 0x40, 0 },					// Dummy row position - dummy row inserted before active row
    { 0x4e, 0x20, 0 },					//////// disabled - reserved register ///////////////
    { REG_GFIX, 0, 0 },					///////// disabled - set to reset val ///////////////
    { 0x74, 0x10, 0 },					// Digital Gain control by REG74[1:0], Digital gain manual control - bypass
    { 0x8d, 0x4f, 0 },					//////// disabled - reserved registers //////////////
    { 0x8e, 0, 0 },
    { 0x8f, 0, 0 },
    { 0x90, 0, 0 },
    { 0x91, 0, 0 },
    { 0x96, 0, 0 },
    { 0x9a, 0, 0 },
    { 0xb0, 0x84, 0 },
    { 0xb1, 0x0c, 0 },
    { 0xb3, 0x82, 0 },

    ////////////////////////// AWBC Control Registers /////////////////////

    { 0x43, 0x0a, 0 },
    { 0x44, 0xf0, 0 },
    { 0x45, 0x34, 0 },
    { 0x46, 0x58, 0 },
    { 0x47, 0x28, 0 },
    { 0x48, 0x3a, 0 },
    { 0x59, 0x88, 0 },
    { 0x5a, 0x88, 0 },
    { 0x5b, 0x44, 0 },
    { 0x5c, 0x67, 0 },
    { 0x5d, 0x49, 0 },
    { 0x5e, 0x0e, 0 },

    //////////////////////////////////////////////////////////////////////

    ////////////////////////// AWBCTR Control Registers /////////////////////
    
    { 0x6c, 0x0a, 0 },
    { 0x6d, 0x55, 0 },
    { 0x6e, 0x11, 0 },
    { 0x6f, 0x9e, 0 }, // it was 0x9F "9e for advance AWB"

    //////////////////////////////////////////////////////////////////////

    { 0x6a, 0x40, 0 },     				// G-Channel AWB Gain
    { REG_BLUE, 0x40, 0 },
    { REG_RED, 0x60, 0 },
    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC | COM8_AWB, 0 },

    // Matrix coefficients
    { 0x4f, 0x80, 0 },
    { 0x50, 0x80, 0 },
    { 0x51, 0, 0 },
    { 0x52, 0x22, 0 },
    { 0x53, 0x5e, 0 },
    { 0x54, 0x80, 0 },
    { 0x58, 0x9e, 0 },

    { REG_COM16, COM16_AWBGAIN, 0 }, // Edge enhancement, denoise
    { REG_EDGE, 0, 0 },
    { 0x75, 0x05, 0 },
    { REG_REG76, 0xe1, 0 }, // Pix correction, magic rsvd
    { 0x77, 0x01, 0 },
    { REG_COM13, 0x08, 0 }, // No gamma, magic rsvd bit
    { 0x4b, 0x09, 0 },
    { 0xc9, 0x60, 0 },		//{REG_COM16, 0x38, 0 },
    { REG_COM11, COM11_EXP | COM11_HZAUTO, 0 },
    { 0xa4, 0x82, 0 }, //Was 0x88
    { 0x9d, 0x4c, 0 },
    { 0x9e, 0x3f, 0 },

};


#endif
#endif // __UTIL_h__


