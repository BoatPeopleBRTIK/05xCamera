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
/****************************************************************************
*   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
*   Author: Gregory Nutt <gnutt@nuttx.org>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name NuttX nor the names of its contributors may be

*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.

****************************************************************************
* Included Files
****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/gpio.h>
#include <sys/types.h>
#include <tinyara/clock.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <tinyara/i2c.h>
#include "util_ov7670.h"

#include "ov_mqtt_xfr.h"

#if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_PWM_AS_CLK))
#include "pwm_routines.h"
#endif

static struct i2c_dev_s *ov7670_i2c_dev;
static struct i2c_config_s ov_7670_i2c_config;

#if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_GPIO_AS_CLK))
static pthread_t clk_output_thread;
static int cnt1 = 0;
#endif

#if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_PWM_AS_CLK))

static int fd_pwm;

static void preparePWM(void)
{
    // Preparing PWM
    pwm_init(PWM_PIN, &fd_pwm);

    int ret = pwm_generator(PWM_FREQUENCY, PWM_DUTY_CYCLE, fd_pwm);
    if (ret < OK)
    {
        printf("failed to set PWM [%d]\n", ret);
        pwm_close(fd_pwm);
        return;
    }
}
#endif // #if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_PWM_AS_CLK)) 


#if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_GPIO_AS_CLK))
static pthread_addr_t prepare_gpio(void *arg)
{
    printf("Start [prepare_gpio] thread\n");


    // prepare GPIO
    int fd = open(GPIO_DEVPATH, O_RDWR);
    if (fd < 0) {
        printf("fd open fail\n");
        return NULL;
        //return;
    }

    // set direction
    ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);

    // set up the initial value
#if (defined(GPIO_CONTROL) && (GPIO_CONTROL == GPIO_IOCTL))
    char str[4];
    bool value = 1;

    if (write(fd, str, snprintf(str, 4, "%d", value != 0) + 1) < 0) {
        printf("write error\n");
        return NULL;
    }

#elif (defined(GPIO_CONTROL) && (GPIO_CONTROL == GPIO_REG))
    volatile unsigned int *ptrGPIO = (uint32_t*)0x80040084;
    *ptrGPIO |= GPIO_32_MASK;
#endif
    while (1)
    {
        printf("In [prepare_gpio]: cnt1=%d\n", cnt1);
        cnt1++;

        //nanosleep(&sleepTime, NULL);
#if (defined(GPIO_CONTROL) && (GPIO_CONTROL == GPIO_IOCTL))
        value = !value;
        if (write(fd, str, snprintf(str, 4, "%d", value != 0) + 1) < 0) {
            printf("write error\n");
            return NULL;
            //return;
        }
#elif (defined(GPIO_CONTROL) && (GPIO_CONTROL == GPIO_REG))
        *ptrGPIO ^= GPIO_32_MASK;
#endif
    }

    return NULL;
}
#endif  //#if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_GPIO_AS_CLK))


static void prepare_clk_output(void)
{
    printf("In prepareClkOutput\n");

#if (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_PWM_AS_CLK))
    preparePWM();

#elif (defined (GENERATE_CLK_OUTPUT) && (GENERATE_CLK_OUTPUT == USE_GPIO_AS_CLK))

    int status;

    pthread_attr_init(&attr);
    sparam.sched_priority = 100;
    status = pthread_attr_setschedparam(&attr, &sparam);
    status = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    status = pthread_attr_setstacksize(&attr, 1024 * 8);
    if (status != OK)
    {
        printf("timedwait_test: pthread_attr_setschedparam failed, status=%d\n", status);
    }

    status = pthread_create(&clk_output_thread, &attr, prepare_gpio, NULL);
    if (status != OK) {
        printf("%s: ERROR: failed to Start clk_output thread: %d\n", __func__, status);
        return;
    }
    pthread_setname_np(clk_output_thread, "prepare_gpio");


#endif

    return;
}

static void delay(unsigned int mS)
{
    systime_t start = clock_systimer();

    mS = mS / MSEC_PER_TICK + 1;

    while (1) {
        if ((start + mS) < clock_systimer()) {
            return;
        }
    }
}


static int ov7670_writeByte_2(uint8_t regaddr, uint8_t regval, uint8_t dev_addr, bool flag)
{
    uint8_t buffer[2];
    int retryCount = 0;
    int ret;
    flag = false;

    ov_7670_i2c_config.address = dev_addr;
    ov_7670_i2c_config.frequency = OV7670_I2C_FREQ;
    ov_7670_i2c_config.addrlen = 7;
    ov_7670_i2c_config.isSCCB = 1;

    // Set up for the transfer
    buffer[0] = regaddr; // Register address
    buffer[1] = regval;  // New register value

    ret = 0;
    for (retryCount = 0; retryCount < RETRY_TIMES; retryCount++)
    {
        ret = i2c_write(ov7670_i2c_dev, &ov_7670_i2c_config, buffer, 2);
        if (ret >= OK)
        {
            
		break;
        }
        else
        {
            printf(".");
        }
    }
    // still fails after RETRY_TIMES
    if (ret < OK)
    {
        printf("[ov7670_writeByte_2]ERROR: i2c_write failed: %d\n", ret);
        return ret;
    }
    else{
	flag = true;
    	return OK;}

}

static int ov7670_writeByte(uint8_t regaddr, uint8_t regval)
{
    uint8_t buffer[2];
    int retryCount = 0;
    int ret;

    ov_7670_i2c_config.address = OV7670_I2C_WRITE_ADDRESS;
    ov_7670_i2c_config.frequency = OV7670_I2C_FREQ;
    ov_7670_i2c_config.addrlen = 7;
    ov_7670_i2c_config.isSCCB = 1;

    // Set up for the transfer
    buffer[0] = regaddr; // Register address
    buffer[1] = regval;  // New register value

    ret = 0;
    for (retryCount = 0; retryCount < RETRY_TIMES; retryCount++)
    {
        ret = i2c_write(ov7670_i2c_dev, &ov_7670_i2c_config, buffer, 2);
        if (ret >= OK)
        {
            break;
        }
        else
        {
            printf(".");
        }
    }
    // still fails after RETRY_TIMES
    if (ret < OK)
    {
        printf("[ov7670_writeByte]ERROR: i2c_write failed: %d\n", ret);
        return ret;
    }
    return OK;

}

static int ov7670_readByte(uint8_t regaddr)
{
    int ret;
    int retryCount = 0;
    uint8_t regval = 0;

    ov_7670_i2c_config.frequency = OV7670_I2C_FREQ;
    ov_7670_i2c_config.address = OV7670_I2C_READ_ADDRESS;
    ov_7670_i2c_config.addrlen = OV7670_I2C_ADDRLEN;
    ov_7670_i2c_config.isSCCB = 1;

    // Write the register address
    ret = 0;
    for (retryCount = 0; retryCount < RETRY_TIMES; retryCount++)
    {
        ret = i2c_write(ov7670_i2c_dev, &ov_7670_i2c_config, &regaddr, 1);
        if (ret >= OK)
        {
            //printf("\t\t[ov7670_readByte] write to i2c device @address[0x%X], register[0x%X]\n => OK!\n", ov_7670_i2c_config.address, regaddr);
            break;
        }
        else
        {
            printf(".");
        }
    }
    // still fails after RETRY_TIMES
    if (ret < OK)
    {
        printf("[ov7670_readByte] Error write to i2c device @address[0x%X], register[0x%X] => FAIL (%d)!\n", ov_7670_i2c_config.address, regaddr, ret);
        return ret;
    }
    

    // Read a byte from the register
    ret = 0;
    for (retryCount = 0; retryCount < RETRY_TIMES; retryCount++)
    {
        ret = i2c_read(ov7670_i2c_dev, &ov_7670_i2c_config, &regval, 1);
        if (ret >= OK)
        {
            //printf("\t\t[ov7670_readByte] read from i2c device @address[0x%X], register[0x%X] => OK! [0x%X] => 0x%X\n", ov_7670_i2c_config.address, regaddr, regaddr, regval);
            break;
        }
        else
        {
            printf(".");
        }
    }
    // still fails after RETRY_TIMES
    if (ret < OK)
    {
        printf("[ov7670_readByte] Error read from i2c device @address[0x%X], register[0x%X] => FAIL (%d)!", ov_7670_i2c_config.address, regaddr, ret);
        return ret;
    }


    return regval;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int ov7670_readByte_2(uint8_t regaddr)
{
    int ret;
    int retryCount = 0;
    uint8_t regval = 0;

    ov_7670_i2c_config.frequency = OV7670_I2C_FREQ;
    ov_7670_i2c_config.address = OV7670_I2C_READ_ADDRESS;
    ov_7670_i2c_config.addrlen = OV7670_I2C_ADDRLEN;
    ov_7670_i2c_config.isSCCB = 1;

    // Write the register address
    ret = 0;
    for (retryCount = 0; retryCount < RETRY_TIMES; retryCount++)
    {
        ret = i2c_write(ov7670_i2c_dev, &ov_7670_i2c_config, &regaddr, 1);
        if (ret >= OK)
        {
            //printf("\t\t[ov7670_readByte] write to i2c device @address[0x%X], register[0x%X]\n => OK!\n", ov_7670_i2c_config.address, regaddr);
            break;
        }
        else
        {
            printf(".");
        }
    }
    // still fails after RETRY_TIMES
    if (ret < OK)
    {
        printf("[ov7670_readByte] Error write to i2c device @address[0x%X], register[0x%X] => FAIL (%d)!\n", ov_7670_i2c_config.address, regaddr, ret);
        return ret;
    }
    

    // Read a byte from the register
    ret = 0;
    for (retryCount = 0; retryCount < RETRY_TIMES; retryCount++)
    {
        ret = i2c_read(ov7670_i2c_dev, &ov_7670_i2c_config, &regval, 1);
        if (ret >= OK)
        {
            //printf("\t\t[ov7670_readByte] read from i2c device @address[0x%X], register[0x%X] => OK! [0x%X] => 0x%X\n", ov_7670_i2c_config.address, regaddr, regaddr, regval);
            break;
        }
        else
        {
            printf(".");
        }
    }
    // still fails after RETRY_TIMES
    if (ret < OK)
    {
        printf("[ov7670_readByte] Error read from i2c device @address[0x%X], register[0x%X] => FAIL (%d)!", ov_7670_i2c_config.address, regaddr, ret);
        return ret;
    }


    return regval;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int ov7670_modifyreg(uint8_t regaddr, uint8_t set)
{
    //uint8_t data;
    int data;

    data = ov7670_readByte(regaddr);
    
    if (data >= OK)
    {
        //printf("READ: Reg[0x%X]=0x%X\t", regaddr, data);
        data = set;
        ov7670_writeByte(regaddr, data);
        //printf("WRITE: 0x%X\t", data);
        int val = ov7670_readByte(regaddr);
        //printf("VERIFY: Reg[0x%X]=0x%X\n", regaddr, val);

        return val;

    }
    else
    {
        printf("+++++++++++++++++ERROR READ: Reg[0x%X]=0x%X\n", regaddr, data);
        return data;
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////

static int ov7670_mod_reg_2(uint8_t regaddr, uint8_t set)
{
    int data;
    data = ov7670_readByte(regaddr);
    if (data >= OK)
    {
        //printf("READ: Reg[0x%X]=0x%X\t", regaddr, data);
        data = set;
        ov7670_writeByte(regaddr, data);
        //printf("WRITE: 0x%X\t", data);
        int val = ov7670_readByte(regaddr);
        //printf("VERIFY: Reg[0x%X]=0x%X\n", regaddr, val);
	
        return val;

    }
    else
    {
        printf("+++++++++++++++++ERROR READ: Reg[0x%X]=0x%X\n", regaddr, data);
        return data;
    }

}



/////////////////////////////////////////////////////////////////////////////////////////////


static void ov7670_setRegisters(t_codec_init_script_entry *script, uint32_t size)
{
    uint32_t i;
    uint16_t ret;

    for (i = 0; i < size; i++) {
        ret = ov7670_modifyreg(script[i].addr, script[i].val);

        delay(script[i].delay);
    }
}

static uint16_t ov7670_setReg_2(t_codec_init_script_entry *script, uint32_t size)
{
    uint32_t i;
    uint16_t ret,reg_count;
    reg_count = 0;
    
    for (i = 0; i < size; i++) {
        ret = ov7670_mod_reg_2(script[i].addr, script[i].val);
	if(ret >= OK){
		reg_count++;
	}
	
        delay(script[i].delay);
    }
    return reg_count;
}

static int ov7670_readTest(void)
{
    printf("In ov7670_readProductID\n");

#if !defined DEBUG_USING_MPU9250
    int pidl = 0;
    int pidh = 0;

    // Check and show product ID and manufacturer ID
    pidl = ov7670_readByte(REG_PID);    // Product ID (MS)
    pidh = ov7670_readByte(REG_VER);    // Product ID (LS)

    if ((pidl >= OK) && (pidh >= OK))
    {
        printf("PID=0x%02x%02x\t", pidh, pidl);
    }

    int blueValue = ov7670_readByte(0x01);
    printf("BLUE[0x%x]=0x%x\n", 0x01, blueValue);

  
#else
    int who_am_I = 0;
    int int_status = 0;
    

    // read from MPU9250 register
    who_am_I = ov7670_readByte(REG_WHO_AM_I_MPU9250);
    if (who_am_I >= OK)
    {
        printf("This device is: 0x%X", who_am_I);
    }

    // read from MPU9250 register
    int_status = ov7670_readByte(REG_INT_STATUS);
    if (int_status >= OK)
    {
        printf("Interrupt status @REG[0x%X]=0x%X\n", REG_INT_STATUS, int_status);
    }
#endif

    return OK;
}


static void init_ov7670_communication(void)
{
    printf("In init_ov7670\n");

    // i2c should have already been initialized before here
    int port = 1;
    ov7670_i2c_dev = up_i2cinitialize(I2C_PORT);
    if (ov7670_i2c_dev == NULL)
    {
        printf("ERROR: up_i2cinitialize(i2c:%d) failed\n", port);
    }

    ov_7670_i2c_config.frequency = OV7670_I2C_FREQ;
    ov_7670_i2c_config.address = OV7670_I2C_WRITE_ADDRESS;
    ov_7670_i2c_config.addrlen = OV7670_I2C_ADDRLEN;
    ov_7670_i2c_config.isSCCB = 1;
}

/////////////////////////////////////////////////////////////////////////////////////

static uint8_t atoi_hex(const char *str){

    uint8_t num = 0;
    if(strlen(str) == 4){

        if((str[3] >= 48) && (str[3] < 58)){
            num = str[3] - 48;
        }
        else if((str[3] >=65) && (str[3] <=70)){
            num = str[3] - 55;
        }
        else if((str[3]>=97) && (str[3]<=102)){
            num = str[3] - 87;
        }
        else{
            printf("ERROR: Invalid hex number -1\n");
        }

        if((str[2] >= 48) && (str[2] < 58)){
            num += ((str[2] - 48)*16);
        }
        else if((str[2] >=65) && (str[2] <=70)){
            num += ((str[2] - 55)*16);
        }
        else if((str[2]>=97) && (str[2]<=102)){
            num += ((str[2] - 87)*16);
        }
        else{
            printf("ERROR: Invalid hex number -2\n");
        }

    }
    else{
        printf("ERROR: Invalid hex number -0\n");
    }
    return num;
}


static uint8_t ov7670_tash_read_reg(const char *str1){

	uint8_t reg_addr;
	uint8_t reg_val =0;

	if(strlen(str1)==4){
		reg_addr = atoi_hex(str1);
		reg_val = ov7670_readByte(reg_addr);
	}
	else{
		printf("ERROR: Invalid register address -1 1\n");
	}

	return reg_val;	
}

static int ov7670_tash_write_reg(const char *str1, const char *str2){

	int ret = 0;

	uint8_t reg_addr;
	uint8_t reg_val;

	if((strlen(str1)==4) && (strlen(str2)==4)){
		
		reg_addr = atoi_hex(str1);
		reg_val  = atoi_hex(str2);
		ret = ov7670_writeByte(reg_addr, reg_val);
		printf("I2C WRITE SUCCESS\n");
	}
	else{
		printf("ERROR: Invalid register address or write value -1 1\n");
	}

	return ret;	
}

static void ov7670_tash_modify_reg(const char *addr_str, const char *val_str){

	int ret = 0;
	
	uint8_t reg_addr;
	uint8_t reg_val;

	if((strlen(addr_str)==4) && (strlen(val_str)==4)){
		
		reg_addr = atoi_hex(addr_str);
		reg_val  = atoi_hex(val_str);
		ret = ov7670_modifyreg(reg_addr,reg_val);
		
		if(ret == reg_val){
			;
		}
		else{
			printf("ERROR : Modify failed\n");
		}
	}
	
}

////////////////////////// Initialized Variables ///////////////////////////////

static uint8_t *yuv_buff = NULL;			// CAPTURE Buffer pointer
static uint8_t *temp_start = NULL;			// Capture buffer start pointer
static uint8_t *temp_end = NULL;			// Capture buffer end pointer

static uint8_t yuv_arr[H_RES*(V_RES + 10)]; 		// Capture buffer
//static uint8_t yuv_arr_padded[H_RES*(V_RES + 2)]; 	//MQTT Buffer
static uint32_t line_arr[LINES + 10] = {0}; 		// Line BYTE count buffer
static uint32_t new_line_arr[LINES + 10] = {0}; 	// modified line BYTE count buffer
static uint16_t line_counter = 0; 			// Line counter variable keeps the count of number of captured lines
//static uint32_t line_shift;
static uint32_t bu;					// Captured byte count i.e., captured buffer size
//static uint32_t duplicate_lines;
//static uint32_t row_offset_buffer[121];

/********************** Profiling Variables *******************************/

static struct timespec start_time = {0,0};		
//static struct timespec end_time = {0,0};
//static struct timespec delta_time = {0,0};		

//////////////////////////////////// Functions ///////////////////////////////////////////////////


/*************************************************************************************************
*
* @name:	time_nsec
*
* @param:	1. s_time - timespec struct for start time
*		2. e_time - timespec struct for end time
*		3. d_time - timespec struct for determining time elapsed
*		4. clear  - flag to clear start time and end time
*
* @return:	none
*
* @brief:	Calculates time elapsed for the provided start and end time.
*
* @note:	Use clock_gettime(CLOCK_REALTIME, &s_time) and clock_gettime(CLOCK_REALTIME, &s_time)
*		before using this function.
*
**************************************************************************************************/

static void time_nsec(struct timespec *s_time, struct timespec *e_time, struct timespec *d_time, bool clear){
	
	d_time->tv_nsec = e_time->tv_nsec - s_time->tv_nsec;
	d_time->tv_sec = e_time->tv_sec - s_time->tv_sec;
	printf("\ntimestamp - %d",d_time->tv_nsec);	
	if(clear == true){
		s_time->tv_nsec = 0;
		s_time->tv_sec = 0;
		e_time->tv_nsec = 0;
		e_time->tv_sec = 0;
	}
}

// time_nsec(&start_time, &end_time, &delta_time);

//////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************************************
* @name:	print_lines
*
* @param:	1. str1 - start line number as input from tash
*		2. str2 - end line number as input from tash
*
* @return:	none
*
* @brief:	Prints pixels of the specified lines 
*
**************************************************************************************************/

static void print_lines(const char *str1, const char *str2){
    
	uint32_t i,j,k;
	uint16_t l1,l2;

	l1 = (uint16_t)(atoi(str1)-1);
	l2 = (uint16_t)(atoi(str2));

	printf("\n\nYUV pixels for lines %d - %d",l1+1,l2);
	for(i=l1;i<l2;i++){
		printf("\n\nLine: %d\n",i+1);
		for(j=0;j<(BYTES_PER_LINE/20);j++){
			printf("\nIndex:  %d\t",j);
			for(k=20*j;k<20*j+20;k++){
				printf("%x ",yuv_arr[i*BYTES_PER_LINE + k]);
			}
		}
	}
	printf("\n\nDone\n");
}

static void pixel_per_line(void){
    printf("\nTotal lines: %d",line_counter+1);
}

/*************************************************************************************************
* @name:	shift_line_counter (discarded now)
*
* @param:	none
*
* @return:	none
*
* @brief:	Shifts the line count in case of split condition during capture
*
**************************************************************************************************/

static uint32_t shift_line_counter(void){
	uint32_t k,i,m;
	m = 0;
	    for(i=0;i<line_counter;i++){
		if(((line_arr[i]%(BYTES_PER_LINE*2)) == 0) && (line_arr[i] != 0)){
		    for(k=line_counter;k>i;k--){
		        line_arr[k+1] = line_arr[k];
		    }
		    line_arr[i] = BYTES_PER_LINE;
		    line_arr[i+1] = BYTES_PER_LINE;
		    m++;
		}
	    }
	return m;
}

/*************************************************************************************************
* @name:
*
* @param:
*
* @return:
*
* @brief:
*
**************************************************************************************************/

#if 0
static void pad_yuv_buffer(void){
	
	uint32_t i,k,line_sum;
	line_sum = 0;
	duplicate_lines = 0;
	printf("\nline counter before padding - %d",line_counter);
	if((line_arr[0] == BYTES_PER_LINE) && line_counter == LINES){
	    for(i=0;i<line_counter;i++){
		if(line_arr[i] != BYTES_PER_LINE){
#ifdef PRINT
		   printf("\nD - %d\tp - %d",i,line_arr[i]);
#endif		
		   duplicate_lines++;
		    //printf("\nline - %d",i);
		    for(k=0;k<BYTES_PER_LINE;k++){
		        yuv_arr_padded[BYTES_PER_LINE*i + k] = yuv_arr_padded[BYTES_PER_LINE*(i-1) + k];
		       // printf("%d\t",yuv_arr_padded[BYTES_PER_LINE*i + k]);
		    }
		    line_sum += line_arr[i];
		}
		else{
		    for(k=0;k<BYTES_PER_LINE;k++){
		       yuv_arr_padded[BYTES_PER_LINE*i + k] =  yuv_arr[line_sum + k];
		    }
		    line_sum += line_arr[i];
		}
	    }
	}
	else{
#ifdef PRINT
	    printf("\n\n DISCARD FRAME");
#endif
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*********************************************************************************************************************************************
*
* @name:	pad_yuv_buffer_2
*
* @param:	1. line_count   - initial line count just after capture
*		2. line_ar      - ptr to line count array (line_arr[])
*		3. new_line_ar  - ptr to new line count array (new_line_arr[]) after procesing the captured buffer
*		4. yuv_buffer   - ptr to the start address of the capture buffer
*
* @return:      last_byte_offset- Offset of the last captured byte relative to the first captured byte i.e., size of capture
*
* @brief:	Processes the captured buffer by using two opeartions - 
*
*		1. line split   - If the falling edge of HS is not detected due to thread context switching, the byte count
*				  of the next line is added to that of the curent line as index of the line count array (line_arr[])
*				  is not incremented in state 4 of the image capture state machine. In such a case, as the captured 
*				  bytes are stored contiguously in the capture buffer(yuv_arr[]), no data is lost. So the byte count
* 				  values of the corresponding lines need to be changed i.e., the byte count of the current line is split
*				  into two lines.
*
*		2. line duplicate-If some bytes in a particular line are not grabbed, the byte count is less than the horizontal resolution.
*				  These lines have to be discarded while processing the image buffer since it is not possible to trace the 
*				  position of the lost pixels in a particular line and pad dummy pixels at those positions on the line.
*
************************************************************************************************************************************************/

static uint32_t pad_yuv_buffer_2(uint16_t line_count, uint32_t *line_ar, uint32_t *new_line_ar, uint8_t *yuv_buffer){

	uint32_t i,k,duplicate_lines,line_sum,last_byte_offset;
	line_sum = 0;
	//er_flag = 0x30;
	duplicate_lines = 0;
	last_byte_offset = 0; 		// Offset of the last captured byte relative to the first captured byte i.e., size of capture
	
#ifdef PRINT
	printf("\nline counter before splitting - %d",line_count);
#endif	

	//////////////////////////////////// line split operation /////////////////////////////////////
	
	///////////////////////////////////////////////////////////////////////////////////////////////

	if(line_arr[0] == BYTES_PER_LINE){
		k=0;
		for(i=0;i<line_count;i++){
			if((*(line_ar + i)%(BYTES_PER_LINE*2))==0){
				*(new_line_ar + k) = BYTES_PER_LINE;
				*(new_line_arr + k + 1) = BYTES_PER_LINE;
				k += 2;
#ifdef PRINT			
				printf("\nLine %d splitted into %d and %d",i,k-2,k-1);
#endif
			}
			/*else if((*(line_ar + i) > BYTES_PER_LINE) && (*(line_ar + i) < (BYTES_PER_LINE*2))){

			}*/
			else{
				*(new_line_ar + k) = *(line_ar + i);
				k++;
			}
		}
		line_count = k;
#ifdef PRINT
		printf("\nline counter after splitting - %d\n\n", line_count);
#endif	
	}
	else{
#ifdef PRINT
	    printf("\nDiscard Frame - lines less than H_RES");
#endif
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	
	////////////////////////////////// line duplicate operation ///////////////////////////////////

	
	if(*(new_line_ar) == BYTES_PER_LINE){
	    if(line_count == LINES){
	        for(i=0;i<LINES;i++){
	            last_byte_offset += *(new_line_ar + i);
	        }
	        for(i=0;i<line_count;i++){
			//if(*(new_line_ar + i) <= BYTES_PER_LINE){
			    if(*(new_line_ar + i) != BYTES_PER_LINE){
#ifdef PRINT
				    printf("\nD - %d\tp - %d",i,*(new_line_ar + i));
#endif  
				    duplicate_lines++; 
				    uint32_t line_byte_shift;
				    line_byte_shift = BYTES_PER_LINE - *(new_line_ar + i);
				    line_sum += *(new_line_ar + i);
				    for(k = last_byte_offset - 1; k > line_sum - 1; k--){
				        *(yuv_buffer + k + line_byte_shift) = *(yuv_buffer + k);
				    }
				    line_sum -= *(new_line_ar + i);
				    last_byte_offset += line_byte_shift;
				    for(k=0;k<BYTES_PER_LINE;k++){
		    	                *(yuv_buffer + BYTES_PER_LINE*i + k) = *(yuv_buffer + BYTES_PER_LINE*(i-1) + k);
		    	            }
		    	            line_sum += *(new_line_ar + i);
		    	            
			    }
			    else{
			            line_sum += *(new_line_ar + i);
				
			    }
			
			
	        }
	    }
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////
	
	
return last_byte_offset;	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************************************
* @name: 	ov7670_state_machine
*
* @param:	none
*
* @return:	none
*
* @brief:	captures single frame and calls the padding funtion (pad_yuv_buffer_2()) to process 
*		the captured buffer.
*
**************************************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ov7670_state_machine(void){

	yuv_buff = yuv_arr;
	temp_start = yuv_arr;
	temp_end = temp_start;

	volatile unsigned int eint_reg;
	uint8_t end_check;
	register uint8_t sync_state;
	register uint16_t pixel;

	eint_reg = 0;
	sync_state = 0;
	end_check = 0;
	pixel=0;

	while(!(end_check)){				// wait for state 6 

		switch(sync_state){

		case 0:
			GPA0_CON = 0x00000000;		// Configure GPA0 as input for HS, VS and PCLK sync signals
			GPG1_CON = 0x00000000;		// Configure GPG1 as input for data
			sync_state =1;
		break;

		// Wait for VS rise
		case 1:	 
			while((VS_BIT) != 0x2);
			{sync_state = 2;}			
		break;

		// Wait for HS rise
		case 2:
			while((HS_BIT) != 0x4);
			{
				sync_state = 3;
				line_counter = 0;}
		break;

		// Line Starts 
		// Collect data while HS is set (during active period)
		case 3:
			eint_reg = GPA0_DAT;	

			if((eint_reg & 0x5) == 0x5){ // HS high and PCLK high
				*yuv_buff = (uint8_t)(GPG1_DAT); // Read upper 7 data bits D[7:1]-> GPG1[7:1]
				yuv_buff++;
				pixel++	;		   // Increment pixel buffer index
				sync_state = 4;	
			}	
		break;

		case 4:
			// New Line
			eint_reg = GPA0_DAT;		// Read HS, VS and PCLK
			if((eint_reg & 0x5) == 0x4){ // HS high and PCLK low
				sync_state = 3;
				
			}
			if((eint_reg & 0x4) == 0x0){ // HS falls
				sync_state = 5;
				line_arr[line_counter] = pixel; // Save pixel count of the line
				pixel =0;
				line_counter++;			// Increment line count
			}		
		break;

		case 5:

			eint_reg = GPA0_DAT;		// Read HS, VS and PCLK

			if(eint_reg & 0x4){  		// HS high - end of line
				sync_state = 3;
			}
			if(eint_reg & 0x2){		// VS high - end of frame
				sync_state = 6;
			}	
		break;

		case 6:
			temp_end = yuv_buff;		// Cpature buffer end location
			end_check = 1; 			// exit state machine condition
			
#ifdef PRINT
			printf("\nLine counter just after capture: %d", line_counter);
#endif
			bu = pad_yuv_buffer_2(line_counter,line_arr, new_line_arr, yuv_arr); // pad the captured buffer by splitting and duplicating lines (if required)
			
#if 0
			line_shift = shift_line_counter();
#ifdef PRINT
			printf("line counter before shift : %d\n",line_counter);
#endif
			line_counter += line_shift;
#ifdef PRINT
			printf("line counter after shift : %d\n",line_counter);
#endif
			pad_yuv_buffer();//*/
#endif			
			
		break;
		default:
			printf("\nDefault State!"); 
		break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************************************
* @name:	ov7670_init
*
* @param:	none
*
* @return:	none
*
* @brief:	Initialize I2C registers
*
**************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ov7670_init(void){

	volatile uint16_t temp_reg_count;
	uint16_t init_count;
	init_count = 0;
	do{	
		temp_reg_count = 0;
		init_count ++;
		init_ov7670_communication();			// Initialize I2C channel 1 
#ifdef PRINT_INIT
		printf("\nConfiguring for QVGA resolution\n\n");
#endif
		//ov7670_setRegisters(ov7670_reset,(sizeof(ov7670_reset)/sizeof(t_codec_init_script_entry)));
		temp_reg_count += ov7670_setReg_2(ov7670_reset,(sizeof(ov7670_reset)/sizeof(t_codec_init_script_entry)));
#ifdef PRINT_INIT
		printf("\nCamera registers being reset\n\n");
#endif
	        //ov7670_setRegisters(ov7670_QVGA_640_240,(sizeof(ov7670_QVGA_640_240)/sizeof(t_codec_init_script_entry)));
#ifdef QVGA
	        temp_reg_count += ov7670_setReg_2(ov7670_QVGA_640_240,(sizeof(ov7670_QVGA_640_240)/sizeof(t_codec_init_script_entry)));
#endif
#ifdef QQVGA
		temp_reg_count += ov7670_setReg_2(ov7670_QQVGA_320,(sizeof(ov7670_QQVGA_320)/sizeof(t_codec_init_script_entry)));
#endif

#ifdef PRINT_INIT
#ifdef QVGA
		printf("\n..... QVGA_640_240 (320 x 240) resolution configured .....\n\n");
#endif
#ifdef QQVGA
		printf("\n..... QQVGA_320_120 (160 x 120) resolution configured .....\n\n");
#endif
#endif
				//ov7670_setRegisters(ov7670_color,(sizeof(ov7670_color)/sizeof(t_codec_init_script_entry)));
		temp_reg_count += ov7670_setReg_2(ov7670_color,(sizeof(ov7670_color)/sizeof(t_codec_init_script_entry)));
#ifdef PRINT_INIT
		printf("\n..... Color registers configured .....\n\n");
#endif
		//ov7670_setRegisters(ov7670_pclk_rate,(sizeof(ov7670_pclk_rate)/sizeof(t_codec_init_script_entry)));
		temp_reg_count += ov7670_setReg_2(ov7670_pclk_rate,(sizeof(ov7670_pclk_rate)/sizeof(t_codec_init_script_entry)));
#ifdef PRINT_INIT
		printf("\n..... Pixel Clock Rate configured .....\n\n");

				
#endif	
		temp_reg_count += ov7670_setReg_2(ov7670_luminance_ref,(sizeof(ov7670_luminance_ref)/sizeof(t_codec_init_script_entry)));	

		printf("\ntotal:        %d",temp_reg_count);
		}
		while((temp_reg_count != 100));
		if(temp_reg_count == 100){
			printf("\ntotal:        %d",temp_reg_count);
			printf("\ninit_count    %d",init_count);
			printf("\n\n\nOV7670 INITIALIZED\n\n");
		}
		else{printf("\n\nI2C init failed\t init_count: %d\n",init_count);}	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************************************
* @name:	ov7670_capture
*
* @param:	1. str_iter_count - number of captures provided as input from tash
*		2. pub		  - enable MQTT publish
*
* @return:	none
*
* @brief:	Capture multiple/ single frame and publish via MQTT to remote client
*
**************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ov7670_capture(const char *str_iter_count,bool pub){

	uint16_t iterations;
	iterations = (uint16_t)(atoi(str_iter_count));

	if((iterations > 0)){

		uint16_t loop_count;
		bool cap_flag;
		uint8_t pclk_val = 0;

		printf("\nImage Capture\n\n");

		for(loop_count=0;loop_count < iterations;loop_count++){

			line_counter = 0;
			line_arr[0] = 0;
			cap_flag = false;

			while(cap_flag == false){				// Loop until perfect caputre is obtained
				ov7670_state_machine();				
				if(line_counter == LINES){
					if(line_arr[0] == BYTES_PER_LINE){	// check for 1st line 
						cap_flag = true;			
					}
				}
			}

			pclk_val = ov7670_tash_read_reg("0x11");		// Read pixel precaler
			printf("\nCapture %d\tPixels: %d\tPCLK: 0x%x\n",loop_count + 1,bu,pclk_val);
			if(pub == true){
				mqtt_pub(yuv_arr,bu);				// Publish padded buffer to MQTT client
			}
		}
	}
	else{
		printf("\n\nERROR - enter correct number of iterations\n");	
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int ov7670_test_main(int argc, char *argv[])
#endif
{

	start_time.tv_nsec = 0;
	start_time.tv_sec = 0;
	switch(argc){

		case 2:			

			// RUN INIT BEFORE QVGA AND QQVGA
			if (strcmp(argv[1], "in") == 0){
				ov7670_init();
			}
			// Debug to check I2C transfer 
			else if (strcmp(argv[1],"dbg") == 0){
				init_ov7670_communication();
				uint8_t data_val;
				data_val = 0;
				ov7670_writeByte(0x12, 0x80);				
				data_val =ov7670_readByte(0x0b);
				printf("\ni2c addr write : 0x%x\n", data_val);	//expected Manufacturer ID (LSB) = 0X73
			}
			// Single Frame Capture without MQTT publish
			else if (strcmp(argv[1], "st") == 0){
				ov7670_capture("1",false); 	// ov7670_capture(<capture count>,true) for MQTT publish
			}
			// Publish captured buffer of size - bu
			else if (strcmp(argv[1], "pub") == 0){
				mqtt_pub(yuv_arr,bu);
			}
			// Print captured buffer size and pointer locations
			else if (strcmp(argv[1], "yuvs") == 0){
				printf("\n\nYUV Buffer Start Location:          %d",temp_start);
				printf("\nYUV Buffer End Location:            %d",temp_end);
				printf("\nYUV Original Buffer Size:           %d",(temp_end-temp_start));
				printf("\nYUV Processed BUffer Size:          %d",bu);
				printf("\nPixels Lost:			      %d",(bu - (uint32_t)(temp_end -temp_start)));
			}	
			// Single Frame Capture with MQTT publish	
			else if (strcmp(argv[1], "cap") == 0){
				ov7670_capture("1",true);
			}
		
			else{printf("ERROR: Invalid command\n");}
		
		break;

		case 3: 
			// Read a register value from input register address from tash 
			if(strcmp(argv[1], "r") == 0) {
					uint8_t read_val = 0;
					read_val = ov7670_tash_read_reg(argv[2]);
					printf("READ\tADDRESS : %s\tVALUE : 0x%X\n",argv[2],read_val);		
			} 
			// Initialize camera and establish MQTT connection with client at specified ip address.
			else if (strcmp(argv[1], "start") == 0){
				ov7670_init();
				mqtt_in(argv[2]);	
			}
			// Multiple frame capture without MQTT publish
			else if (strcmp(argv[1], "st") == 0){
				ov7670_capture(argv[2],false);
			}
			// Multiple frame capture with MQTT publish
			else if (strcmp(argv[1], "cap") == 0){
				ov7670_capture(argv[2],true);
			}
			// Print pixel count of the specified line
			else if (strcmp(argv[1], "line") == 0){
				uint16_t p  = 0;
				p = (uint16_t)(atoi(argv[2])) - 1;
				if(p<120){
					printf("\n%d",line_arr[p]);
				}
			}
			// Initialize MQTT  Connection at the specified IP address.
			else if (strcmp(argv[1], "pub_init") == 0){
				mqtt_in(argv[2]);
			}
			else{printf("ERROR: Invalid read command\n");}
		break;

		case 4:
			// Write value at the register address
			if (strcmp(argv[1], "w") == 0) {
					int ret_check = 0;
					
					ret_check = ov7670_tash_write_reg(argv[2],argv[3]);
					if(ret_check >= OK){
						printf("WRITE\tADDRESS : %s\tVALUE : %s\n",argv[2],argv[3]);
					}
					else{
						printf("I2C register write failed\n");
					}
			}
			// Modify register value at given address
			else if (strcmp(argv[1], "m") == 0) {
				ov7670_tash_modify_reg(argv[2],argv[3]);
			}
			// Print pixel count of the specified lines
			else if (strcmp(argv[1], "l") == 0){
				printf("\n\nTotal lines: %d\n\n",line_counter);
				uint16_t y,p,q;
				p = (uint16_t)(atoi(argv[2])) - 1;
				q = (uint16_t)(atoi(argv[3])) - 1;
				if((q<120)&&(p>=0)){
					for(y=p;y<=q;y++){
						printf("%d ",line_arr[y]);
					}
				}
			}
			// Print pixels of the specified lines
			else if(strcmp(argv[1], "pxl") == 0){
				print_lines(argv[2],argv[3]);
			}
			else{printf("ERROR: Invalid write command\n");}
		break;

		default:
			printf("ERROR: Invalid arguments - 0\n");
		break;
	}
return EXIT_SUCCESS;
}






