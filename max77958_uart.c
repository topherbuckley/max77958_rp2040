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
#include <stdarg.h>
#include <sys/stat.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "i2c.h"
#include "rtc.h"
#include "uart.h"
#include "lp.h"
#include "board.h"
#include "pb.h"
#include "max77958_uart.h"
#include "max77958_dev.h"
#include "max77958_thread.h"

enum {
	MSG_LEVEL_USER = 0,
	MSG_LEVEL_DEBUG,
	MSG_LEVEL_MAX,
}GUI_MSG_LEVEL;

enum {
	GUI_STAT_NOT_CONNECT = 0,
	GUI_STAT_CONNECTED,
}GUI_STAT;

enum {
	MSG_STAT_HEADER_DETECTING,
	MSG_STAT_HEADER_FAIL,
	MSG_STAT_BODY_RECEVING,
	MSG_STAT_CMD_HANDLING,
}MSG_STAT;

enum {
	GPIO_CONFIG_INPUT = 0,
	GPIO_CONFIG_OUTPUT,
	GPIO_CONFIG_ISR,
	GPIO_CONFIG_ALT,
}GPIO_USER_CONFIG_STAT;

/* Stringification macros */
#define STRING(x) STRING_(x)
#define STRING_(x) #x

#define HEADER_SIZE 6
#define TAIL_SIZE 5


#define I2C_MASTER	    MXC_I2C0

const char msg_Header_[HEADER_SIZE] = "<mxim>";
const char msg_Tail_[TAIL_SIZE] = "<end>";

int msg_level_ = MSG_LEVEL_DEBUG;
int msg_Status_ = MSG_STAT_HEADER_DETECTING;
int msg_size_ = -1;

unsigned int iBuff_;     /* Index into buffer */
unsigned int iHeadTail_;     /* Index into buffer */
BaseType_t xMore_;

char buffer_Rx_[RX_BUF_SIZE];      /* Buffer for input  */
char buffer_Tx_[TX_BUF_SIZE];      /* Buffer for output */

/**
 * @brief   mxim_printf
 * @details
 * @note
 *
 *
 */

void mxim_printf(int _msg_type, char *_msg, ...)
{
	int size;
	va_list fmtargs;
	char buffer[256];

	memset(buffer, 0, 256);

    va_start(fmtargs,_msg);
    size = vsnprintf(buffer, sizeof(buffer)-1, _msg, fmtargs);
	va_end(fmtargs);

	printf("<mxim>%c%c", size, _msg_type);
	printf("%s", buffer);
	printf("<end>");
	fflush(stdout);
}

/**
 * @brief    mxim_dbg_printf
 * @details
 *
 * @note
 *
 *
 */
void mxim_dbg_printf(int _msg_type, char *_msg, ...)
{
	int size;
	va_list fmtargs;
	char buffer[256];

	memset(buffer, 0, 256);

    va_start(fmtargs,_msg);
    size = vsnprintf(buffer,sizeof(buffer)-1,_msg,fmtargs);
	va_end(fmtargs);

	printf("<mxim>%c%c", size, MSG_CMD_DBG);
	printf("%s",buffer);
	printf("<end>");
	fflush(stdout);
}

