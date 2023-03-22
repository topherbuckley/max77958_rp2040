/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#ifndef PRJ_MAX77895_DEVICECONTROL_H_
#define PRJ_MAX77895_DEVICECONTROL_H_
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
typedef struct T_Q_DATA
{
	char msg[256];
}T_Q_DATA;
//i2c
int i2c_write_byte(uint8_t _slave7, uint8_t _reg, uint8_t _data);
int i2c_write_nbytes(uint8_t _slave, uint8_t _reg, uint8_t* _data, uint8_t _nsize);
int i2c_read_byte(uint8_t _slave7, uint8_t _reg, uint8_t* _data);
int i2c_read_nbytes(uint8_t _slave7, uint8_t _reg, uint8_t* _data, uint8_t _nsize);

//gpio
int register_gpio_isr(int _n, void (*_fp)());
int gpio_read(int _pinno);
int gpio_write(int _pinno, int _en);
uint32_t gpio_get_batt_detect();
void max77958_hw_config();
//Test Functions
uint32_t test_gpio();

//extern QueueHandle_t xQueue; //fixed the build erro

#endif /* PRJ_MAX77895_DEVICECONTROL_H_ */
