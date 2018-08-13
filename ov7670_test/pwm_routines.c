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

#include "pwm_routines.h"
#include "util.h"

static struct pwm_info_s pwm_info;

void print_pwm_info(void)
{
	printf("Frequency:%d, Duty:%d\n", pwm_info.frequency,  pwm_info.duty);
	return;
}

void pwm_init(int pwm_pin, int *fd)
{
	switch (pwm_pin) {
	case 0:
		*fd = open("/dev/pwm0", O_RDWR);
		break;
	case 1:
		*fd = open("/dev/pwm1", O_RDWR);
		break;
	case 2:
		*fd = open("/dev/pwm2", O_RDWR);
		break;
	case 3:
		*fd = open("/dev/pwm3", O_RDWR);
		break;
    case 4:
        *fd = open("/dev/pwm4", O_RDWR);
        break;
    case 5:
        *fd = open("/dev/pwm5", O_RDWR);
        break;
	default:
		printf("use pwm0 as the default device\n");
		*fd = open("/dev/pwm0", O_RDWR);
		break;
	}

	#if defined(DEBUG_TEST) && DEBUG_TEST
	printf("[pwm_init] PWM[%d] opened!\n", pwm_pin);
	#endif // #if defined(DEBUG_TEST) && DEBUG_TEST

	pwm_info.frequency = PWM_FREQUENCY;
    pwm_info.duty = PWM_DUTY_CYCLE;

	return;
}

int pwm_generator(int frequency, int dc, int fd)
{
    int ret = 0;

    ret = ioctl(fd, PWMIOC_STOP, 0);
    if (ret != OK)
    {
        printf("[pwm_generator]: ioctl(PWMIOC_STOP) failed: %d\n", ret);
        close(fd);
    }

    pwm_info.duty = dc;
    ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
	if (ret != OK)
	{
		printf("[pwm_generator]: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", ret);
		close(fd);
	}

	ret = ioctl(fd, PWMIOC_START, 0);
	if (ret != OK)
	{
		printf("[pwm_generator]: ioctl(PWMIOC_START) failed: %d\n", ret);
		close(fd);
	}

	//printf("[pwm_generator] Start generating PWM with duty cycle of %0.2f!\n", (dc/65535));
    return ret;
}

int pwm_close(int fd)
{
    close(fd);
    fflush(stdout);
    return OK;
}



