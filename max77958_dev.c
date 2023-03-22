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

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Maxim CMSIS SDK */
#include "rtc.h"
#include "uart.h"
#include "lp.h"
#include "board.h"
#include "pb.h"


#include "mxc_config.h"
#include "i2c.h"
#include "mxc_delay.h"
#include "max77958_uart.h"
#include "max77958_dev.h"
#include "max77958_thread.h"
#define GPIO_PORT    PORT_0
static gpio_cfg_t gpio_interrupt_status;
static void (*gpio_isr_6_cb)() = NULL;
int register_gpio_isr(int _n, void (*_fp)())
{
	switch(_n) {
	case 6:

		if(gpio_isr_6_cb != NULL)
			return -1;
		else
			gpio_isr_6_cb = _fp;
		break;
	default:
		return -1;
	}
	return 0;
}

void gpio_isr_6(void *cbdata)
{
	if(gpio_isr_6_cb != NULL)
		gpio_isr_6_cb();
}

/**
 * @brief
 * @detials
 *
 */
void gpio_interupt_config()
{
    gpio_cfg_t gpio_interrupt;
    /* Switch on EV kit is open when non-pressed, and grounded when pressed.  Use an internal pull-up so pin
       reads high when button is not pressed. */
    gpio_interrupt.port = PORT_0;
    gpio_interrupt.mask = PIN_6;
    GPIO_Config(&gpio_interrupt);
    GPIO_RegisterCallback(&gpio_interrupt, gpio_isr_6, &gpio_interrupt_status);
    GPIO_IntConfig(&gpio_interrupt, GPIO_INT_LEVEL, GPIO_INT_LOW);
    GPIO_IntDisable(&gpio_interrupt);        
    NVIC_EnableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(PORT_0));
}
/********************************
 *  I2C MASTER Config & Control
 *
 *  PQ81 : 0x35
 */

#define I2C_MASTER	    MXC_I2C0
#define I2C_MASTER_IDX	0

volatile int i2c_flag;

/**
 * @brief
 * @detials
 *
 */
//Master interrupt handler
void I2C0_IRQHandler(void)
{
    I2C_Handler(I2C_MASTER);
    return;
}

//I2C callback
void i2c_callback(i2c_req_t *req, int error)
{
    i2c_flag = error;
    return;
}

/**
 * @brief
 * @detials
 *
 */
int i2c_master_config()
{
    int error;
    const sys_cfg_i2c_t sys_i2c_cfg = NULL; /* No system specific configuration needed. */

    //Setup the I2CM
    I2C_Shutdown(I2C_MASTER);
    if((error = I2C_Init(I2C_MASTER, I2C_FAST_MODE, &sys_i2c_cfg)) != E_NO_ERROR) {
        printf("Error initializing I2C%d.  (Error code = %d)\n", I2C_MASTER_IDX, error);
        return 1;
    }
    NVIC_EnableIRQ(I2C0_IRQn);

    return 0;
}


static uint8_t i2c_txdata_[256+1];
/**
 * @brief
 * @detials
 *
 */
int i2c_write_byte(uint8_t _slave7, uint8_t _reg, uint8_t _data)
{
    int icnt = 0;
    int error;
    int opcode_en = 0;
    unsigned char opcode_slave;

	if ((_slave7<<1) == 0xCC) {
		opcode_en = 1;
		opcode_slave = 0x25;
	} else if ((_slave7<<1) == 0xCE) {
		opcode_en = 1;
		opcode_slave = 0x26;
	}

    i2c_txdata_[0] = _reg;
    i2c_txdata_[1] = _data;

    if (opcode_en) {
    	I2C_OpCodeWrite(MXC_I2C0, _slave7<<1, i2c_txdata_, 2);
    } else {
    	if ((error = I2C_MasterWrite(MXC_I2C0, _slave7<<1, i2c_txdata_, 2, 0)) != 2) {
			while(1){
				if(++icnt> 16)
					return 1;
			}
		}
    }
    return 0;
}

/**
 * @brief
 * @detials
 *
 */
int i2c_write_nbytes(uint8_t _slave, uint8_t _reg, uint8_t* _data, uint8_t _nsize)
{
    int icnt = 0;
    int error;
    int opcode_en = 0;
    unsigned char opcode_slave;

    if ((_slave<<1) == 0xCC) {
		opcode_en = 1;
		opcode_slave = 0x25;
	} else if ((_slave<<1) == 0xCE) {
		opcode_en = 1;
		opcode_slave = 0x26;
	}

    i2c_txdata_[0] = _reg;
    memcpy(&i2c_txdata_[1], _data, _nsize);

    if (opcode_en) {
        	I2C_OpCodeWrite(MXC_I2C0, _slave<<1, i2c_txdata_, _nsize+1);
    } else {
		if((error = I2C_MasterWrite(MXC_I2C0, _slave<<1, i2c_txdata_, _nsize+1, 0)) != _nsize+1) {
			//printf("Error writing %d\n", error);
			while(1){
				if(++icnt> 16)
					return 1;
			}
		}
    }
    return 0;
}


void opcode_delay(int usec)
{
	int i = 0;

	for (i = 0; i < usec; i++) {
		int nopcnt = 32;
		while (nopcnt > 0) {
			asm volatile("NOP");
			nopcnt--;
		}

	}
}

/**
 * @brief
 * @detials
 *
 */
int i2c_read_byte(uint8_t _slave7, uint8_t _reg, uint8_t* _data)
{
    int icnt = 0;
    int error;
    int opcode_en = 0;
    uint8_t opcode_slave;

    if ((_slave7<<1) == 0xCC) {
    	opcode_en = 1;
    	opcode_slave = 0x25;
	} else if ((_slave7<<1) == 0xCE) {
		opcode_en = 1;
		opcode_slave = 0x26;
	}

    if (opcode_en) {
    	I2C_OpCodeWrite(MXC_I2C0, _slave7<<1, &_reg, 1);
    	_reg = 0x51;
    }

    if (opcode_en) {
    	if((error = I2C_MasterWrite(MXC_I2C0, opcode_slave<<1, &_reg, 1, 1)) != 1){
			//printf("Error writing %d\n", error);
			while(1){
				if(++icnt> 16)
					return 1;
			}
		}

    	opcode_delay(250000);

    	if((error = I2C_OpCodeRead(MXC_I2C0, _slave7<<1, _data, 1, 0)) != 1) {
			//printf("Error reading%d\n", error);
			while(1){
				if(++icnt> 16)
					return 2;
			}
		}
    } else {
    	if((error = I2C_MasterWrite(MXC_I2C0, _slave7<<1, &_reg, 1, 1)) != 1){
			//printf("Error writing %d\n", error);
			while(1){
				if(++icnt> 16)
					return 1;
			}
		}

    	if((error = I2C_MasterRead(MXC_I2C0, _slave7<<1, _data, 1, 0)) != 1) {
			//printf("Error reading%d\n", error);
			while(1){
				if(++icnt> 16)
					return 2;
			}
		}
    }

    return 0;
}

/**
 * @brief
 * @detials
 *
 */
int i2c_read_nbytes(uint8_t _slave7, uint8_t _reg, uint8_t* _data, uint8_t _nsize)
{
    int icnt = 0;
    int error;
    int opcode_en = 0;
    uint8_t opcode_slave;

	if ((_slave7<<1) == 0xCC) {
		opcode_en = 1;
		opcode_slave = 0x25;
	} else if ((_slave7<<1) == 0xCE) {
		opcode_en = 1;
		opcode_slave = 0x26;
	}

	if (opcode_en) {
		I2C_OpCodeWrite(MXC_I2C0, _slave7<<1, &_reg, 1);
		_reg = 0x51;
	}



    if (opcode_en) {
    	if((error = I2C_MasterWrite(MXC_I2C0, opcode_slave<<1, &_reg, 1, 1)) != 1){
			//printf("Error writing %d\n", error);
			while(1){
				if(++icnt> 16)
					return 1;
			}
		}

    	opcode_delay(250000);

    	if((error = I2C_OpCodeRead(MXC_I2C0, _slave7<<1, _data, _nsize, 0)) != _nsize) {
			//printf("Error reading%d\n", error);
			while(1){
				if(++icnt> 16)
					return 2;
			}
		}
    } else {
    	if((error = I2C_MasterWrite(MXC_I2C0, _slave7<<1, &_reg, 1, 1)) != 1){
			//printf("Error writing %d\n", error);
			while(1){
				if(++icnt> 16)
					return 1;
			}
		}

    	if((error = I2C_MasterRead(MXC_I2C0, _slave7<<1, _data, _nsize, 0)) != _nsize) {
			//printf("Error reading%d\n", error);
			while(1){
				if(++icnt> 16)
					return 2;
			}
		}
    }

    return 0;
}


/**
 * @brief
 * @detials
 *
 */
void max77958_hw_config()
{
	i2c_master_config();
    max77958_probe_func();
	gpio_interupt_config();
	max77958_register_irq();    
	max77958_init();
    return;
}


