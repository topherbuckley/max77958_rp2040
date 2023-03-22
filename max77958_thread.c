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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* Maxim CMSIS SDK */
#include "uart.h"
#include "lp.h"
#include "board.h"
#include "pb.h"
#include "max77958_uart.h"
#include "max77958_dev.h"
#include "max77958_thread.h"


void max77958_statemachine_handler(char _device_id){
    gpio_cfg_t gpio_interrupt;
    gpio_interrupt.port = PORT_0;
    gpio_interrupt.mask = PIN_6;
    gpio_interrupt.pad = GPIO_PAD_PULL_UP;
    gpio_interrupt.func = GPIO_FUNC_IN;
    GPIO_IntDisable(&gpio_interrupt);    
    max77958_irq_thread();
    GPIO_IntEnable(&gpio_interrupt);    
}


void max77958_init(){
	max77958_probe_func();
	xTaskNotify(_typec_task_id, 0, 0);
}

void max77958p1_thread_wake_up( void )
{
	BaseType_t xHigherPriorityTaskWoken;
	static struct xMessage xdata;
    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;
    xdata.msg = 0x1;
	if( xStateMachine != NULL )
	{
        /* Post the byte to the back of the queue. */
    xQueueSendToBackFromISR( xStateMachine, &xdata, &xHigherPriorityTaskWoken );
    // Now we can switch context if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


void max77958_isr()
{
	max77958p1_thread_wake_up();
}


void max77958_register_irq()
{
	register_gpio_isr(6, max77958_isr);
}
