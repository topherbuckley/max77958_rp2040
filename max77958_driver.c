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

#include "mxc_config.h"
#include "board.h"

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
#include "max77958_driver.h"
#include "max77958_pass2.h"

#define	ARRAY_SIZE(x)	( sizeof((x))/sizeof((x)[0]) )



const struct DP_DP_DISCOVER_IDENTITY DP_DISCOVER_IDENTITY = {
	{
		.BITS.Num_Of_VDO = 1,
		.BITS.Cmd_Type = ACK,
		.BITS.Reserved = 0
	},

	{
		.BITS.VDM_command = Discover_Identity,
		.BITS.Rsvd2_VDM_header = 0,
		.BITS.VDM_command_type = REQ,
		.BITS.Object_Position = 0,
		.BITS.Rsvd_VDM_header = 0,
		.BITS.Structured_VDM_Version = Version_1_0,
		.BITS.VDM_Type = STRUCTURED_VDM,
		.BITS.Standard_Vendor_ID = 0xFF00
	}

};
const struct DP_DP_DISCOVER_ENTER_MODE DP_DISCOVER_ENTER_MODE = {
	{
		.BITS.Num_Of_VDO = 1,
		.BITS.Cmd_Type = ACK,
		.BITS.Reserved = 0
	},
	{
		.BITS.VDM_command = Enter_Mode,
		.BITS.Rsvd2_VDM_header = 0,
		.BITS.VDM_command_type = REQ,
		.BITS.Object_Position = 1,
		.BITS.Rsvd_VDM_header = 0,
		.BITS.Structured_VDM_Version = Version_1_0,
		.BITS.VDM_Type = STRUCTURED_VDM,
		.BITS.Standard_Vendor_ID = 0xFF01
	}
};

struct DP_DP_CONFIGURE DP_CONFIGURE = {
	{
		.BITS.Num_Of_VDO = 2,
		.BITS.Cmd_Type = ACK,
		.BITS.Reserved = 0
	},
	{
		.BITS.VDM_command = 17, /* SVID Specific Command */
		.BITS.Rsvd2_VDM_header = 0,
		.BITS.VDM_command_type = REQ,
		.BITS.Object_Position = 1,
		.BITS.Rsvd_VDM_header = 0,
		.BITS.Structured_VDM_Version = Version_1_0,
		.BITS.VDM_Type = STRUCTURED_VDM,
		.BITS.Standard_Vendor_ID = 0xFF01
	},
	{
		.BITS.SEL_Configuration = num_Cfg_UFP_U_as_UFP_D,
		.BITS.Select_DP_V1p3 = 1,
		.BITS.Select_USB_Gen2 = 0,
		.BITS.Select_Reserved_1 = 0,
		.BITS.Select_Reserved_2 = 0,
		.BITS.DFP_D_PIN_Assign_A = 0,
		.BITS.DFP_D_PIN_Assign_B = 0,
		.BITS.DFP_D_PIN_Assign_C = 0,
		.BITS.DFP_D_PIN_Assign_D = 1,
		.BITS.DFP_D_PIN_Assign_E = 0,
		.BITS.DFP_D_PIN_Assign_F = 0,
		.BITS.DFP_D_PIN_Reserved = 0,
		.BITS.UFP_D_PIN_Assign_A = 0,
		.BITS.UFP_D_PIN_Assign_B = 0,
		.BITS.UFP_D_PIN_Assign_C = 0,
		.BITS.UFP_D_PIN_Assign_D = 0,
		.BITS.UFP_D_PIN_Assign_E = 0,
		.BITS.UFP_D_PIN_Assign_F = 0,
		.BITS.UFP_D_PIN_Reserved = 0,
		.BITS.DP_MODE_Reserved = 0
	}
};




void max77958_check_apcmd(const struct max77958_apcmd_data *cmd_data);
void max77958_reset_ic();


struct max77958_apcmd_data_queue g_max77958_apcmd_queue;

/////////////QUEUE/////////////////////////////////////////////////

int max77958_i2c_opcode_write(unsigned char _opcode, unsigned char *_values,unsigned char _length){
	unsigned char _wbuffer[OPCODE_MAX_LENGTH] = { 0, };
    int _sid = MAX77958_SLAVE_P1;

	if (_length > OPCODE_DATA_LENGTH)
		return -2;
    
	_wbuffer[0] = _opcode;
    
	if (_length)
		memcpy(&_wbuffer[1], _values, _length);

    i2c_write_nbytes(_sid, OPCODE_WRITE, _wbuffer, _length + OPCODE_SIZE);    
	if (_length < OPCODE_DATA_LENGTH)
        i2c_write_byte(_sid, OPCODE_WRITE_END, 0x0);    
	return 0;
}

int max77958_i2c_opcode_read(		unsigned char opcode, unsigned char length, unsigned char *values)
{
	int size = 0;
    int _sid = MAX77958_SLAVE_P1;
	if (length > OPCODE_DATA_LENGTH)
		return -2;

    i2c_read_nbytes(_sid, OPCODE_READ, values,length);
    
	return size;
}


unsigned char max77958_circ_bbuf_invaild()
{
	unsigned char ret = 0x0;
	if (g_max77958_apcmd_queue.head==g_max77958_apcmd_queue.tail)
		ret = 1;
	return ret;
}
void max77958_circ_bbuf_init()
{
    unsigned char pos = 0x0;
    g_max77958_apcmd_queue.head = 0x0;
    g_max77958_apcmd_queue.tail = 0x0;
    g_max77958_apcmd_queue.prev_tail = 0x0;
    for(pos=0x0; pos<QUEUESIZE; pos++){
        memset(&g_max77958_apcmd_queue.queue[pos], 0x00, sizeof(struct max77958_apcmd_data));
        g_max77958_apcmd_queue.queue[pos].opcode = OPCODE_RSVD;
        g_max77958_apcmd_queue.queue[pos].response = OPCODE_RSVD;
    }
}

int max77958_circ_bbuf_push(struct max77958_apcmd_data *apcmd)
{
      unsigned char next = g_max77958_apcmd_queue.head+1;      
      unsigned char tail = g_max77958_apcmd_queue.tail;            

      if(next>=QUEUESIZE){
            next = 0;
      }
      if(next==tail){
        mxim_dbg_printf(MSG_CMD_LOG, "circular buffer is full.\r\n");        
        return -1;
      }    
     memset(&g_max77958_apcmd_queue.queue[g_max77958_apcmd_queue.head], 0x00, sizeof(struct max77958_apcmd_data));
     memcpy(&g_max77958_apcmd_queue.queue[g_max77958_apcmd_queue.head], apcmd, sizeof(struct max77958_apcmd_data));
     g_max77958_apcmd_queue.head = next;
     return 1;
}


int max77958_circ_bbuf_pop(struct max77958_apcmd_data *apcmd)
{
    int next;
    g_max77958_apcmd_queue.prev_tail = g_max77958_apcmd_queue.tail;
    if (g_max77958_apcmd_queue.head == g_max77958_apcmd_queue.tail)  // if the head == tail, we don't have any data
        return -1;
    next = g_max77958_apcmd_queue.tail+1;
    if(next>=QUEUESIZE){
          next = 0;
    }    
     memset(apcmd, 0x00, sizeof(struct max77958_apcmd_data));     
     memcpy(apcmd, &g_max77958_apcmd_queue.queue[g_max77958_apcmd_queue.tail], sizeof(struct max77958_apcmd_data));
     g_max77958_apcmd_queue.tail = next; 
     return 1;
}

void max77958_send_apcmd()
{

	struct max77958_apcmd_data cmd_data;
    memset(&cmd_data, 0x00, sizeof(struct max77958_apcmd_data));     
	if (max77958_circ_bbuf_invaild())
		goto error;
    max77958_circ_bbuf_pop(&cmd_data);            
	if (cmd_data.opcode != OPCODE_RSVD){
		max77958_i2c_opcode_write(cmd_data.opcode,
			cmd_data.write_data,cmd_data.write_length);
        goto exit;        
     }else{
        if(g_max77958_apcmd_queue.tail > 0){
            g_max77958_apcmd_queue.tail--;
        }else{
            g_max77958_apcmd_queue.tail = 0x0;
        }
        
        goto skip;
     }        
error:
    mxim_dbg_printf(MSG_CMD_LOG, "queue is invaild.\r\n");
    return;
skip:
    mxim_dbg_printf(MSG_CMD_LOG, "push the apcmd.\r\n");    
    return;
exit:
    mxim_dbg_printf(MSG_CMD_LOG, "send the apcmd.\r\n");
	return;

}

void max77958_execute_apcmd()
{

	struct max77958_apcmd_data cmd_data;
	if (max77958_circ_bbuf_invaild())
		goto error;
    max77958_circ_bbuf_pop(&cmd_data);
	max77958_check_apcmd(&cmd_data);
	if (max77958_circ_bbuf_invaild())
		goto error; 
	max77958_send_apcmd();
        goto exit;
error:
    mxim_dbg_printf(MSG_CMD_LOG, "queue is invaild.\r\n");
	return;    
exit:
    mxim_dbg_printf(MSG_CMD_LOG, "execute the apcmd.\r\n");
	return;
}


void max77958_request_apcmd(struct max77958_apcmd_data *input_apcmd)
{
	struct max77958_apcmd_data execute_cmd_data;
    memset(&execute_cmd_data, 0x00, sizeof(struct max77958_apcmd_data));         
	execute_cmd_data.opcode = input_apcmd->opcode;
	execute_cmd_data.write_length = input_apcmd->write_length;
	execute_cmd_data.is_uvdm = input_apcmd->is_uvdm;
	memcpy(execute_cmd_data.write_data, input_apcmd->write_data, OPCODE_DATA_LENGTH);
	execute_cmd_data.seq = OPCODE_CMD_REQUEST;
    max77958_circ_bbuf_push(&execute_cmd_data); //Sending Data
    memset(&execute_cmd_data, 0x00, sizeof(struct max77958_apcmd_data));         
	execute_cmd_data.opcode = OPCODE_RSVD;    
	execute_cmd_data.response = input_apcmd->opcode;
	execute_cmd_data.read_length = input_apcmd->read_length;
	execute_cmd_data.is_uvdm = input_apcmd->is_uvdm;
	execute_cmd_data.is_chg_int = input_apcmd->is_chg_int;
	execute_cmd_data.seq = OPCODE_CMD_REQUEST;    
    max77958_circ_bbuf_push(&execute_cmd_data); //Responsing Data
    max77958_send_apcmd();
}

void max77958_insert_apcmd(struct max77958_apcmd_data *input_apcmd)
{
	struct max77958_apcmd_data execute_cmd_data;
    memset(&execute_cmd_data, 0x00, sizeof(struct max77958_apcmd_data));         
	execute_cmd_data.opcode = input_apcmd->opcode;
	execute_cmd_data.write_length = input_apcmd->write_length;
	execute_cmd_data.is_uvdm = input_apcmd->is_uvdm;
	memcpy(execute_cmd_data.write_data, input_apcmd->write_data, OPCODE_DATA_LENGTH);
	execute_cmd_data.seq = OPCODE_CMD_REQUEST;
    max77958_circ_bbuf_push(&execute_cmd_data); //Sending Data
    memset(&execute_cmd_data, 0x00, sizeof(struct max77958_apcmd_data));         
	execute_cmd_data.opcode = OPCODE_RSVD;    
	execute_cmd_data.response = input_apcmd->opcode;
	execute_cmd_data.read_length = input_apcmd->read_length;
	execute_cmd_data.is_uvdm = input_apcmd->is_uvdm;
	execute_cmd_data.is_chg_int = input_apcmd->is_chg_int;
	execute_cmd_data.seq = OPCODE_CMD_REQUEST;    
    max77958_circ_bbuf_push(&execute_cmd_data); //Responsing Data
}
/////////////FIRMWARE UPDATE///////////
int max77958_write_apcmd(unsigned char _pos, unsigned char _opcode, unsigned char *_values,unsigned char _length){

	unsigned char _wbuffer[OPCODE_DATA_LENGTH+OPCODE_SIZE] = { 0, };
    int _sid = MAX77958_SLAVE_P1;
    int i = 0;
    int len = _length;    
    
	if (len > OPCODE_DATA_LENGTH) {
		return -1;
    }

	i2c_write_byte(_sid,REG_UIC_INT_M, 0xFF);
	_wbuffer[0] = _opcode;

    for (i = 0; i < len; i++) {
        _wbuffer[i+1] = _values[i];
    }
    i2c_write_nbytes(_sid, OPCODE_WRITE, _wbuffer, OPCODE_DATA_LENGTH);
    i2c_write_byte(_sid, OPCODE_WRITE_END, 0x0);
	return 0;
}




int max77958_read_apcmd(unsigned char _pos, unsigned char _opcode, unsigned char *_rbuffer, unsigned char _length){
	int _sid = MAX77958_SLAVE_P1;
    int cnt = 0;
    unsigned char uic_int =0;
    int i = 0, j =0;

	if (_length > OPCODE_DATA_LENGTH)
		goto err;
retry:
	for(i=0; i< 100; i ++) {j++;}
    i2c_read_byte(_sid, REG_UIC_INT, &uic_int);
    if (((uic_int & BIT_APCMDRESI) != BIT_APCMDRESI) && cnt ++ < 10000) { //wait for 5sec
        goto retry;
    }
    i2c_read_nbytes(_sid, OPCODE_READ, _rbuffer,_length);
    if(cnt  < 10){
        if(_opcode!=_rbuffer[0])
            goto retry;

    }
    else{
        goto err;
    }
	i2c_write_byte(_sid,REG_UIC_INT_M, REG_UIC_INT_M_INIT);    
	return 0;
 err:
	i2c_write_byte(_sid,REG_UIC_INT_M, REG_UIC_INT_M_INIT);    
    return -2;
}


static int max77958_usbc_wait_response(unsigned char _pos){
	int _sid = MAX77958_SLAVE_P1;
    int cnt = 0;
    unsigned char uic_int =0;
retry_wait:
    opcode_delay(2000);
    i2c_read_byte(_sid, REG_UIC_INT, &uic_int);
    if (((uic_int & BIT_APCMDRESI) != BIT_APCMDRESI) && cnt ++ < 10000) { //wait for 5sec
        goto retry_wait;
    }

    if(cnt == 10000){
        goto error_wait;
    }

    return 0;

error_wait:
    return FW_UPDATE_TIMEOUT_FAIL;
    
}


static int __max77958_usbc_fw_update(
		int _pos, const unsigned char *fw_bin)
{
	unsigned char fw_cmd = FW_CMD_END;
	unsigned char fw_len = 0;
	unsigned char fw_opcode = 0;
	unsigned char fw_data_len = 0;
	unsigned char fw_data[FW_CMD_WRITE_SIZE] = { 0, };
	unsigned char verify_data[FW_VERIFY_DATA_SIZE] = { 0, };
	int ret = -FW_UPDATE_CMD_FAIL;
    int _sid = MAX77958_SLAVE_P1;    

	/*
	 * fw_bin[0] = Write Command (0x01)
	 * or
	 * fw_bin[0] = Read Command (0x03)
	 * or
	 * fw_bin[0] = End Command (0x00)
	 */
	fw_cmd = fw_bin[0];

	/*
	 * Check FW Command
	 */
	if (fw_cmd == FW_CMD_END) {
		max77958_reset_ic();
		max77958_data.fw_update_state = FW_UPDATE_END;
		return FW_UPDATE_END;
	}

	/*
	 * fw_bin[1] = Length ( OPCode + Data )
	 */
	fw_len = fw_bin[1];

	/*
	 * Check fw data length
	 * We support 0x22 or 0x04 only
	 */
	if (fw_len != 0x22 && fw_len != 0x04)
		return FW_UPDATE_MAX_LENGTH_FAIL;

	/*
	 * fw_bin[2] = OPCode
	 */
	fw_opcode = fw_bin[2];

	/*
	 * In case write command,
	 * fw_bin[35:3] = Data
	 *
	 * In case read command,
	 * fw_bin[5:3]  = Data
	 */
	fw_data_len = fw_len - 1; /* exclude opcode */
	memcpy(fw_data, &fw_bin[3], fw_data_len);

	switch (fw_cmd) {
	case FW_CMD_WRITE:
        i2c_write_nbytes(_sid, fw_opcode, fw_data, fw_data_len);
    	max77958_data.fw_update_state = FW_UPDATE_WAIT_RESP_START;    
		ret = max77958_usbc_wait_response(_pos);
    	max77958_data.fw_update_state = FW_UPDATE_WAIT_RESP_STOP;   
        if(ret)
            return ret;
        
		return FW_CMD_WRITE_SIZE;


	case FW_CMD_READ:

        i2c_read_nbytes(_sid, fw_opcode, verify_data, fw_data_len);
		/*
		 * Check fw data sequence number
		 * It should be increased from 1 step by step.
		 */
		if (memcmp(verify_data, &fw_data[1], 2)) {
			mxim_dbg_printf(MSG_CMD_LOG,"[0x%02x 0x%02x], [0x%02x]\r\n",
				verify_data[0], fw_data[0], verify_data[1]);
			mxim_dbg_printf(MSG_CMD_LOG,"[0x%02x], [0x%02x, 0x%02x]\r\n",
				fw_data[1],	verify_data[2], fw_data[2]);
			return FW_UPDATE_VERIFY_FAIL;
		}

		return FW_CMD_READ_SIZE;
	}

	mxim_dbg_printf(MSG_CMD_LOG,"Command error\r\n");

	return ret;
}


int max77958_usbc_fw_update(int _pos,
		const unsigned char *fw_bin, int fw_bin_len, int enforce_do)
{
	struct max77958_fw_header fw_header;
	int offset = 0;
	int size = 0;
	int try_count = 0;
	unsigned char try_command = 0;
    uint32_t sec = 0x0, subsec = 0x0;
    int _sid = MAX77958_SLAVE_P1;    
    int duration = 0x0;
    int ret = 0x0;


    //Need to check the time.
    //RTC_GetTime(&sec,&subsec);

    
	i2c_write_byte(_sid,REG_UIC_INT_M, 0xFF);
	i2c_write_byte(_sid,REG_CC_INT_M, 0xFF);
	i2c_write_byte(_sid,REG_PD_INT_M, 0xFF);
	i2c_write_byte(_sid,REG_VDM_INT_M, 0xFF);     

retry:
	offset = 0;
	duration = 0;
	size = 0;
	ret = 0;

    i2c_read_byte(_sid, REG_UIC_HW_REV,&max77958_data.HW_Revision);   
    i2c_read_byte(_sid, REG_UIC_DEVICE_ID, &max77958_data.Device_Revision);   
    i2c_read_byte(_sid, REG_UIC_FW_REV, &max77958_data.FW_Revision);   
    i2c_read_byte(_sid, REG_UIC_FW_SUBREV, &max77958_data.FW_Minor_Revision);       


	mxim_dbg_printf(MSG_CMD_LOG,"chip : %02X.%02X, FW : %02X.%02X\r\n",
		max77958_data.FW_Revision, max77958_data.FW_Minor_Revision,
		fw_bin[4], fw_bin[5]);
	if ((max77958_data.FW_Revision == 0xff) ||
			(max77958_data.FW_Revision < fw_bin[4]) ||
			((max77958_data.FW_Revision == fw_bin[4]) &&
			(max77958_data.FW_Minor_Revision < fw_bin[5])) ||
			enforce_do) {

        i2c_write_byte(_sid,0x21, 0xD0);
        i2c_write_byte(_sid,0x41, 0x00);
        
        opcode_delay(100000);        
        i2c_read_byte(_sid, REG_UIC_FW_REV, &max77958_data.FW_Revision);   
        i2c_read_byte(_sid, REG_UIC_FW_SUBREV, &max77958_data.FW_Minor_Revision);       
    	mxim_dbg_printf(MSG_CMD_LOG,"Start FW updating (%02X.%02X) Time (%x,%x)\r\n",
			max77958_data.FW_Revision, max77958_data.FW_Minor_Revision,sec,subsec);
		if (max77958_data.FW_Revision != 0xFF && try_command < 3) {
			try_command++;
			goto retry;
		}

		/* Copy fw header */
		memcpy(&fw_header, fw_bin, FW_HEADER_SIZE);

		if (max77958_data.FW_Revision != 0xFF) {
			if (++try_count < FW_VERIFY_TRY_COUNT) {
				mxim_dbg_printf(MSG_CMD_LOG,"the Fail to enter secure mode %d\r\n",
						try_count);
				goto retry;
			} else {
				mxim_dbg_printf(MSG_CMD_LOG,"the Secure Update Fail!!\r\n");
				goto out;
			}
		}

		for (offset = FW_HEADER_SIZE;
				offset < fw_bin_len && size != FW_UPDATE_END;) {

			size = __max77958_usbc_fw_update(_pos,
				&fw_bin[offset]);

			switch (size) {
			case FW_UPDATE_VERIFY_FAIL:
				offset -= FW_CMD_WRITE_SIZE;
				/* FALLTHROUGH */
			case FW_UPDATE_TIMEOUT_FAIL:
				/*
				 * Retry FW updating
				 */
				if (++try_count < FW_VERIFY_TRY_COUNT) {
					mxim_dbg_printf(MSG_CMD_LOG,"Retry fw write.\r\n");
					mxim_dbg_printf(MSG_CMD_LOG,"ret %d, count %d, offset %d\r\n",
						size, try_count, offset);
					goto retry;
				} else {
					mxim_dbg_printf(MSG_CMD_LOG,"Failed to update FW.\r\n");
					mxim_dbg_printf(MSG_CMD_LOG,"ret %d, offset %d\r\n",
							size, (offset + size));
					goto out;
				}
				break;
			case FW_UPDATE_CMD_FAIL:
			case FW_UPDATE_MAX_LENGTH_FAIL:
				mxim_dbg_printf(MSG_CMD_LOG,"Failed to update FW.\r\n");
				mxim_dbg_printf(MSG_CMD_LOG,"ret %d, offset %d\r\n",
						size, (offset + size));
				goto out;
			case FW_UPDATE_END: /* 0x00 */
                i2c_read_byte(_sid, REG_UIC_FW_REV, &max77958_data.FW_Revision);   
                i2c_read_byte(_sid, REG_UIC_FW_SUBREV, &max77958_data.FW_Minor_Revision);                       
				mxim_dbg_printf(MSG_CMD_LOG,"chip : %02X.%02X, FW : %02X.%02X\r\n",
					max77958_data.FW_Revision,
					max77958_data.FW_Minor_Revision,
					fw_header.data4, fw_header.data5);
				mxim_dbg_printf(MSG_CMD_LOG,"1. Completed, Time (%x,%x)\r\n",sec,subsec);
                
				break;
			default:
				offset += size;
				break;
			}
			if (offset == fw_bin_len) {
				max77958_reset_ic();
                i2c_read_byte(_sid, REG_UIC_FW_REV, &max77958_data.FW_Revision);   
                i2c_read_byte(_sid, REG_UIC_FW_SUBREV, &max77958_data.FW_Minor_Revision);                       
				mxim_dbg_printf(MSG_CMD_LOG,"chip : %02X.%02X, FW : %02X.%02X\r\n",
					max77958_data.FW_Revision,
					max77958_data.FW_Minor_Revision,
					fw_header.data4, fw_header.data5);
				mxim_dbg_printf(MSG_CMD_LOG,"2. Completed\r\n");
			}
		}
	} else {
		mxim_dbg_printf(MSG_CMD_LOG,"Don't need to update!\r\n");
		goto out;
	}
out:    
	return 0;
}

void max77958_update_custm_info(int _pos)
{
	union max77958_custm_info customer_info;
	unsigned char w_data[OPCODE_MAX_LENGTH] = { 0, };
	unsigned char r_data[OPCODE_MAX_LENGTH] = { 0, };    
	int length = 0;
	length = sizeof(union max77958_custm_info);


	memset(w_data, 0x00, sizeof(w_data));
	max77958_write_apcmd(_pos, OPCODE_SET_CSTM_INFORMATION_R,w_data,length);
	max77958_read_apcmd(_pos, OPCODE_SET_CSTM_INFORMATION_R,r_data,length);
	memcpy(&customer_info, &r_data[1], sizeof(union max77958_custm_info));


    mxim_dbg_printf(MSG_CMD_LOG, "Update the customer info [%x][%x][%x][%x]\r\n",customer_info.CustmInfoArry[0],customer_info.CustmInfoArry[1],customer_info.CustmInfoArry[2],customer_info.CustmInfoArry[3]);

    
	if(_pos){

        if(customer_info.custm_bits.TypeCstate == DRP){
            mxim_dbg_printf(MSG_CMD_LOG, " 1. No Update the customer info\r\n");
            goto no_update;
        }   
    }
    else{

        if(customer_info.custm_bits.TypeCstate == SNK){
            mxim_dbg_printf(MSG_CMD_LOG, "2. No Update the customer info\r\n");            
            goto no_update;
        }

    }

	memset(&customer_info, 0x00, sizeof(customer_info));
	mxim_dbg_printf(MSG_CMD_LOG,"update customer information\r\n");
	if(_pos){
	mxim_dbg_printf(MSG_CMD_LOG,"1. update customer information\r\n");        
	customer_info.custm_bits.debugSRC = DISABLED;
	customer_info.custm_bits.debugSNK = DISABLED;
	customer_info.custm_bits.AudioAcc = DISABLED;
	customer_info.custm_bits.TrySNK = DISABLED;
	customer_info.custm_bits.TypeCstate = DRP;
	customer_info.custm_bits.notdef1 = MemoryUpdateMTP;
	customer_info.custm_bits.VID = PD_VID;
	customer_info.custm_bits.PID_H = 0x68;
	customer_info.custm_bits.PID_L = 0x60;    
	customer_info.custm_bits.i2cSID1 = I2C_SID0;
	customer_info.custm_bits.i2cSID2 = I2C_SID1;
	customer_info.custm_bits.i2cSID3 = I2C_SID2;
	customer_info.custm_bits.i2cSID4 = I2C_SID3;
	customer_info.custm_bits.snkPDOVoltage_H = 0x64; /* 5000mV */    
	customer_info.custm_bits.snkPDOVoltage_L = 0x64; /* 5000mV */
	customer_info.custm_bits.snkPDOOperatingI = 0x012c; /* 3000mA */
	customer_info.custm_bits.srcPDOpeakCurrent = 0x0; /* 100% IOC */
	customer_info.custm_bits.srcPDOVoltage = 0x0064; /* 5000mV */
	customer_info.custm_bits.srcPDOMaxCurrent = 0x0096; /* 1500mA */
	customer_info.custm_bits.srcPDOOverIDebounce = 0x003c; /* 60ms */
	customer_info.custm_bits.srcPDOOverIThreshold = 0x005A; /* 900mA */
	memcpy(&w_data[0], &customer_info, sizeof(union max77958_custm_info));
    }else{
	mxim_dbg_printf(MSG_CMD_LOG,"2. update customer information\r\n");             
	customer_info.custm_bits.debugSRC = DISABLED;
	customer_info.custm_bits.debugSNK = DISABLED;
	customer_info.custm_bits.AudioAcc = DISABLED;
	customer_info.custm_bits.TrySNK = DISABLED;
	customer_info.custm_bits.TypeCstate = SNK;
	customer_info.custm_bits.notdef1 = MemoryUpdateMTP;
	customer_info.custm_bits.VID = PD_VID;
	customer_info.custm_bits.PID_H = 0x68;
	customer_info.custm_bits.PID_L = 0x60;    
	customer_info.custm_bits.i2cSID1 = I2C_SID0;
	customer_info.custm_bits.i2cSID2 = I2C_SID1;
	customer_info.custm_bits.i2cSID3 = I2C_SID2;
	customer_info.custm_bits.i2cSID4 = I2C_SID3;
	customer_info.custm_bits.snkPDOVoltage_H = 0x64; /* 5000mV */    
	customer_info.custm_bits.snkPDOVoltage_L = 0x64; /* 5000mV */
	customer_info.custm_bits.snkPDOOperatingI = 0x012c; /* 3000mA */
	customer_info.custm_bits.srcPDOpeakCurrent = 0x0; /* 100% IOC */
	customer_info.custm_bits.srcPDOVoltage = 0x0064; /* 5000mV */
	customer_info.custm_bits.srcPDOMaxCurrent = 0x0096; /* 1500mA */
	customer_info.custm_bits.srcPDOOverIDebounce = 0x003c; /* 60ms */
	customer_info.custm_bits.srcPDOOverIThreshold = 0x005A; /* 900mA */
	memcpy(&w_data[0], &customer_info, sizeof(union max77958_custm_info));
    }

	max77958_write_apcmd(_pos, OPCODE_SET_CSTM_INFORMATION_W,w_data,length);
	max77958_read_apcmd(_pos, OPCODE_SET_CSTM_INFORMATION_W,r_data,length);
no_update:
	mxim_dbg_printf(MSG_CMD_LOG,"exit customer information\r\n");

}

int max77958_update_action_mtp(int _pos,
		const unsigned char *action_bin, int fw_bin_len, int force_download)
{
	int offset = 0;
	int size = 0;
	int try_count = 0;
	unsigned char action_data[28];
	unsigned char w_data[OPCODE_DATA_LENGTH] = { 0, };
	unsigned char r_data[OPCODE_DATA_LENGTH] = { 0, };
	int length = 0;
	int mtp_pos = 0;
	int prev_pos = 0;
    int _sid = MAX77958_SLAVE_P1;
    int i = 0x0, j= 0x0;
    unsigned char uic_int;
    int cnt = 0;    
    unsigned char FW_Revision = 0xFF;

retry_1:
	offset = 0;
	size = 0;
	mtp_pos = 0;
	prev_pos = 0x0;
	try_count++;


    i2c_read_byte(_sid, REG_UIC_INT, &FW_Revision);

	if ((FW_Revision != 0xFF) && try_count < 10 && force_download) {

		while (offset < fw_bin_len) {
			memset(action_data, 0, sizeof(action_data));
			memset(w_data, 0, sizeof(w_data));
			memset(r_data, 0, sizeof(r_data));
			for (size = 0x0;
				size < 28 && offset < fw_bin_len; size++){
				action_data[size] = action_bin[offset++];
            }    
			if (mtp_pos) {
				prev_pos = mtp_pos;
				w_data[0] = mtp_pos++;
			} else {
				w_data[0] = 0xF0;
				mtp_pos++;
			}
			length = size;
			memcpy(&w_data[1], &action_data, length);
            max77958_write_apcmd(_pos, OPCODE_ACTION_BLOCK_MTP_UPDATE,w_data,length);
            cnt = 0x0;
retry_opcode:            
            for(i=0; i< 100; i ++) {j++;}     
            i2c_read_byte(_sid, REG_UIC_INT, &uic_int);
            
            if (((uic_int & BIT_APCMDRESI) != BIT_APCMDRESI) && cnt ++ < 1000) {
                goto retry_opcode;
            }
            i2c_read_nbytes(_sid, OPCODE_READ, r_data,0x3);	
			if (r_data[1] == 0xFF) {
	            mxim_dbg_printf(MSG_CMD_LOG,"MTP Erase Fail\r\n");                
				break;
			} else if (r_data[1] == prev_pos || r_data[1] == 0xF0) {
	            mxim_dbg_printf(MSG_CMD_LOG,"MTP Write Done\r\n");                     
			} else {
	            mxim_dbg_printf(MSG_CMD_LOG,"MTP ReTry [%x][%x][%x]\r\n",
                    r_data[0],r_data[1],r_data[3]);                   
				goto retry_1;
			}
		}
	} else {
        mxim_dbg_printf(MSG_CMD_LOG,"Need to update the firmware! or MTP write fail [%x][%x][%x]\r\n",
                r_data[0],r_data[1],r_data[3]); 	
		goto out;
	}
out:
	return 0;
}



/////////////VDM/////////////////////////////////////////////////

static int max77958_process_vdm_identity()
{
	struct max77958_apcmd_data enqueue_data;    
	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_DISCOVER_IDENTITY;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 31;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_identity\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_process_vdm_svids()
{
	struct max77958_apcmd_data enqueue_data;
	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_DISCOVER_SVIDS;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 31;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_svids\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_process_vdm_modes()
{
	struct max77958_apcmd_data enqueue_data;

	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_DISCOVER_MODES;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 11;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_modes\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_process_vdm_enter_mode()
{
	struct max77958_apcmd_data enqueue_data;

	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_ENTER_MODE;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 7;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_enter_mode\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_process_vdm_dp_status()
{
	struct max77958_apcmd_data enqueue_data;

	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_SVID_DP_STATUS;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 11;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_dp_status\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}


static int max77958_process_vdm_dp_configure()
{
	struct max77958_apcmd_data enqueue_data;

	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_SVID_DP_CONFIGURE;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 11;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_dp_configure\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_process_vdm_attention()
{
	struct max77958_apcmd_data enqueue_data;

	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_ATTENTION;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 11;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_attention\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_process_vdm_exit_mode()
{
	struct max77958_apcmd_data enqueue_data;
    
	enqueue_data.opcode = OPCODE_GET_VDM_RESP;
	enqueue_data.write_data[0] = OPCODE_ID_VDM_EXIT_MODE;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 7;
    mxim_dbg_printf(MSG_CMD_LOG,"max77958_process_vdm_exit_mode\r\n");
	max77958_request_apcmd(&enqueue_data);
	return 0;
}

static int max77958_vdm_irq()
{
	unsigned char mode = 0;
    i2c_read_byte(MAX77958_SLAVE_P1, REG_DP_STATUS, &mode);
	if (mode & VDM_DISCOVER_ID) {
		max77958_process_vdm_identity();
        max77958_data.last_alternate = VDM_DISCOVER_ID;
        max77958_data.received_discover_mode = 0x0;    
    }
	if (mode & VDM_DISCOVER_SVIDS){
		max77958_process_vdm_svids();
        max77958_data.last_alternate = VDM_DISCOVER_SVIDS;
    }
	if (mode & VDM_DISCOVER_MODES){
        if(max77958_data.received_discover_mode == 0x0)
    		max77958_process_vdm_modes();        
        max77958_data.last_alternate = VDM_DISCOVER_MODES;
        max77958_data.received_discover_mode = 0x1; 
    }
	if (mode & VDM_ENTER_MODE){
		max77958_process_vdm_enter_mode();
        max77958_data.last_alternate = VDM_ENTER_MODE;
    }
	if (mode & VDM_DP_STATUS_UPDATE){
		max77958_process_vdm_dp_status();
        max77958_data.last_alternate = VDM_DP_STATUS_UPDATE;
    }
	if (mode & VDM_DP_CONFIGURE){
		max77958_process_vdm_dp_configure();
        max77958_data.last_alternate = VDM_DP_STATUS_UPDATE;
        max77958_data.received_discover_mode = 0x0;    
    }
	if (mode & VDM_ATTENTION){
		max77958_process_vdm_attention();
        max77958_data.last_alternate = VDM_ATTENTION;
    }
	if (mode & VDM_EXIT_MODE){
		max77958_process_vdm_exit_mode();
        max77958_data.last_alternate = VDM_EXIT_MODE;        
    }
	return 0;
}


void max77958_vdm_process_set_identity_req()
{
	struct max77958_apcmd_data cmd_data;
	int len = sizeof(DP_DISCOVER_IDENTITY);
	int vdm_header_num = sizeof(UND_DATA_MSG_VDM_HEADER_Type);
	int vdo0_num = 0;
	cmd_data.opcode = OPCODE_SEND_VDM;
	memcpy(cmd_data.write_data, &DP_DISCOVER_IDENTITY, sizeof(DP_DISCOVER_IDENTITY));
	cmd_data.write_length = len;
	vdo0_num = DP_DISCOVER_IDENTITY.byte_data.BITS.Num_Of_VDO * 4;
	cmd_data.read_length = OPCODE_SIZE + OPCODE_HEADER_SIZE + vdm_header_num + vdo0_num;
    max77958_request_apcmd(&cmd_data);
}

void max77958_vdm_process_set_DP_enter_mode_req()
{
	struct max77958_apcmd_data cmd_data;
	int len = sizeof(DP_DISCOVER_ENTER_MODE);
	int vdm_header_num = sizeof(UND_DATA_MSG_VDM_HEADER_Type);
	int vdo0_num = 0;
	cmd_data.opcode = OPCODE_SEND_VDM;
	memcpy(cmd_data.write_data, &DP_DISCOVER_ENTER_MODE, sizeof(DP_DISCOVER_ENTER_MODE));
	cmd_data.write_length = len;
	vdo0_num = DP_DISCOVER_ENTER_MODE.byte_data.BITS.Num_Of_VDO * 4;
	cmd_data.read_length = OPCODE_SIZE + OPCODE_HEADER_SIZE + vdm_header_num + vdo0_num;
    max77958_request_apcmd(&cmd_data);
}

void max77958_vdm_process_set_DP_configure_mode_req(uint8_t W_DATA)
{
	struct max77958_apcmd_data cmd_data;
	int len = sizeof(DP_CONFIGURE);
	int vdm_header_num = sizeof(UND_DATA_MSG_VDM_HEADER_Type);
	int vdo0_num = 0;
	cmd_data.opcode = OPCODE_SEND_VDM;
	memcpy(cmd_data.write_data, &DP_CONFIGURE, sizeof(DP_CONFIGURE));
	cmd_data.write_data[6] = W_DATA;
	cmd_data.write_length = len;
	vdo0_num = DP_CONFIGURE.byte_data.BITS.Num_Of_VDO * 4;
	cmd_data.read_length = OPCODE_SIZE + OPCODE_HEADER_SIZE + vdm_header_num + vdo0_num;
    max77958_request_apcmd(&cmd_data);
}

static int max77958_vdm_process_discover_svids(unsigned char *vdm_data, int len)
{
	UND_VDO1_Type  *DATA_MSG_VDO1 = (UND_VDO1_Type  *)&vdm_data[8];

	max77958_data.SVID_0 = DATA_MSG_VDO1->BITS.SVID_0;
	max77958_data.SVID_1 = DATA_MSG_VDO1->BITS.SVID_1;

	if (max77958_data.SVID_0 == TypeC_DP_SUPPORT) {
		max77958_data.dp_is_connect = 1;
		/* If you want to support USB SuperSpeed when you connect
		 * Display port dongle, You should change dp_hs_connect depend
		 * on Pin assignment.If DP use 4lane(Pin Assignment C,E,A),
		 * dp_hs_connect is 1. USB can support HS.
		 * If DP use 2lane(Pin Assigment B,D,F),dp_hs_connect is 0. USB
		 * can support SS
		 */
		 max77958_data.dp_hs_connect = 1;
        mxim_dbg_printf(MSG_CMD_LOG, "host turn on wait.\r\n");            
		}
    mxim_dbg_printf(MSG_CMD_LOG, "SVID_0 : 0x%X, SVID_1 : 0x%X\r\n",
		max77958_data.SVID_0, max77958_data.SVID_1);
	return 0;
}

static int max77958_vdm_process_discover_mode(char *vdm_data, int len)
{
	DIS_MODE_DP_CAPA_Type *pDP_DIS_MODE = (DIS_MODE_DP_CAPA_Type *)&vdm_data[0];
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = (UND_DATA_MSG_VDM_HEADER_Type *)&vdm_data[4];

	mxim_dbg_printf(MSG_CMD_LOG,"vendor_id = 0x%04x , svid_1 = 0x%04x\r\n", DATA_MSG_VDM->BITS.Standard_Vendor_ID, max77958_data.SVID_1);
	if (DATA_MSG_VDM->BITS.Standard_Vendor_ID == TypeC_DP_SUPPORT && max77958_data.SVID_0  == TypeC_DP_SUPPORT) {
		/*  pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS. */
		mxim_dbg_printf(MSG_CMD_LOG,"pDP_DIS_MODE->MSG_HEADER.DATA = 0x%08X\r\n", pDP_DIS_MODE->MSG_HEADER.DATA);
		mxim_dbg_printf(MSG_CMD_LOG,"pDP_DIS_MODE->DATA_MSG_VDM_HEADER.DATA = 0x%08X\r\n", pDP_DIS_MODE->DATA_MSG_VDM_HEADER.DATA);
		mxim_dbg_printf(MSG_CMD_LOG,"pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.DATA = 0x%08X\r\n", pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.DATA);

		if (pDP_DIS_MODE->MSG_HEADER.BITS.Number_of_obj > 1) {
			if ((pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Port_Capability == num_UFP_D_Capable)
				&& (pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Receptacle_Indication == num_USB_TYPE_C_Receptacle)) {
				max77958_data.pin_assignment = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.UFP_D_Pin_Assignments;
           		mxim_dbg_printf(MSG_CMD_LOG,"1. support UFP_D 0x%08x\r\n", max77958_data.pin_assignment);
			} else if (((pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Port_Capability == num_UFP_D_Capable)
				&& (pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Receptacle_Indication == num_USB_TYPE_C_PLUG))) {
				max77958_data.pin_assignment = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.DFP_D_Pin_Assignments;
        		mxim_dbg_printf(MSG_CMD_LOG,"2. support DFP_D 0x%08x\r\n", max77958_data.pin_assignment);
			} else if (pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Port_Capability == num_DFP_D_and_UFP_D_Capable) {
				if (pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Receptacle_Indication == num_USB_TYPE_C_PLUG) {
					max77958_data.pin_assignment = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.DFP_D_Pin_Assignments;
            		mxim_dbg_printf(MSG_CMD_LOG,"3. support DFP_D 0x%08x\r\n", max77958_data.pin_assignment);
				} else {
					max77958_data.pin_assignment = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.UFP_D_Pin_Assignments;
            		mxim_dbg_printf(MSG_CMD_LOG,"4. support UFP_D 0x%08x\r\n", max77958_data.pin_assignment);
				}
			} else if (pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Port_Capability == num_DFP_D_Capable) {
				max77958_data.pin_assignment = DP_PIN_ASSIGNMENT_NODE;
			    mxim_dbg_printf(MSG_CMD_LOG,"do not support Port_Capability num_DFP_D_Capable!!!\r\n");
				return -2;
			} else {
				max77958_data.pin_assignment = DP_PIN_ASSIGNMENT_NODE;
				mxim_dbg_printf(MSG_CMD_LOG,"there is not valid object information!!!\r\n");
				return -2;
			}
		}
	}

	if (DATA_MSG_VDM->BITS.Standard_Vendor_ID == TypeC_DP_SUPPORT) {
		max77958_vdm_process_set_DP_enter_mode_req();
	}
	return 0;
}

static int max77958_vdm_process_enter_mode(char *vdm_data, int len)
{
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = (UND_DATA_MSG_VDM_HEADER_Type *)&vdm_data[4];

	if (DATA_MSG_VDM->BITS.VDM_command_type == 1) {
		mxim_dbg_printf(MSG_CMD_LOG,"EnterMode ACK.\r\n");
	} else {
		mxim_dbg_printf(MSG_CMD_LOG,"EnterMode NAK.\r\n");
	}
	return 0;
}

static int max77958_vdm_dp_status_update(char *vdm_data, int len)
{
	int i;
	unsigned char multi_func_preference = 0;
	int pin_assignment = 0;
	int hpd = 0;
	int hpdirq = 0;    
	VDO_MESSAGE_Type *VDO_MSG;
	DP_STATUS_UPDATE_Type *DP_STATUS;
	uint8_t W_DATA = 0x0;

	if (max77958_data.SVID_0 == TypeC_DP_SUPPORT) {
		DP_STATUS = (DP_STATUS_UPDATE_Type *)&vdm_data[0];

		mxim_dbg_printf(MSG_CMD_LOG,"DP_STATUS_UPDATE = 0x%08X\r\n", DP_STATUS->DATA_DP_STATUS_UPDATE.DATA);

		if (DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.Port_Connected == 0x00) {
			mxim_dbg_printf(MSG_CMD_LOG,"port disconnected!\r\n");
		} else {
			if (max77958_data.is_sent_pin_configuration == 0) {

				multi_func_preference =
					DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.Multi_Function_Preference;
				if (multi_func_preference == 1) {
					if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_D) {
						W_DATA = DP_PIN_ASSIGNMENT_D;
						pin_assignment = DP_PIN_ASSIGNMENT_D;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_B) {
						W_DATA = DP_PIN_ASSIGNMENT_B;
						pin_assignment = DP_PIN_ASSIGNMENT_B;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_F) {
						W_DATA = DP_PIN_ASSIGNMENT_F;
						pin_assignment = DP_PIN_ASSIGNMENT_F;
					} else {
						mxim_dbg_printf(MSG_CMD_LOG,"wrong pin assignment value\r\n");
					}
				} else {
					if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_C) {
						W_DATA = DP_PIN_ASSIGNMENT_C;
						pin_assignment = DP_PIN_ASSIGNMENT_C;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_E) {
						W_DATA = DP_PIN_ASSIGNMENT_E;
						pin_assignment = DP_PIN_ASSIGNMENT_E;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_A) {
						W_DATA = DP_PIN_ASSIGNMENT_A;
						pin_assignment = DP_PIN_ASSIGNMENT_A;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_D) {
						W_DATA = DP_PIN_ASSIGNMENT_D;
						pin_assignment = DP_PIN_ASSIGNMENT_D;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_B) {
						W_DATA = DP_PIN_ASSIGNMENT_B;
						pin_assignment = DP_PIN_ASSIGNMENT_B;
					} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_F) {
						W_DATA = DP_PIN_ASSIGNMENT_F;
						pin_assignment = DP_PIN_ASSIGNMENT_F;
					} else {
						mxim_dbg_printf(MSG_CMD_LOG,"2.wrong pin assignment value\r\n");
					}
				}
				max77958_data.dp_selected_pin = pin_assignment;

				mxim_dbg_printf(MSG_CMD_LOG,"multi_func_preference %d, W_DATA : %d\r\n",
					multi_func_preference,
					W_DATA);

				max77958_vdm_process_set_DP_configure_mode_req(W_DATA);

				max77958_data.is_sent_pin_configuration = 1;
			} else {
				mxim_dbg_printf(MSG_CMD_LOG,"pin configuration is already sent as %d!\r\n",
					max77958_data.dp_selected_pin );
			}
		}

		if (DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.HPD_State == 1)
			hpd = CCIC_NOTIFY_HIGH;
		else if (DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.HPD_State == 0)
			hpd = CCIC_NOTIFY_LOW;

		if (DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.HPD_Interrupt == 1)
			hpdirq = CCIC_NOTIFY_IRQ;
        
	} else {
		/* need to check F/W code */
		VDO_MSG = (VDO_MESSAGE_Type *)&vdm_data[8];
		for (i = 0; i < 6; i++)
			mxim_dbg_printf(MSG_CMD_LOG,"VDO_%d : %d\r\n", i+1, VDO_MSG->VDO[i]);
	}
	return 0;
}

static int max77958_vdm_dp_attention(char *vdm_data, int len)
{
	int i;
	int hpd = 0;
	int hpdirq = 0;
	int pin_assignment = 0;

	VDO_MESSAGE_Type *VDO_MSG;
	DIS_ATTENTION_MESSAGE_DP_STATUS_Type *DP_ATTENTION;
	unsigned char multi_func_preference = 0;
	uint8_t W_DATA = 0;

	if (max77958_data.SVID_0 == TypeC_DP_SUPPORT) {
		DP_ATTENTION = (DIS_ATTENTION_MESSAGE_DP_STATUS_Type *)&vdm_data[0];

		mxim_dbg_printf(MSG_CMD_LOG,"DP_ATTENTION = 0x%08X\r\n",
			DP_ATTENTION->DATA_MSG_DP_STATUS.DATA);
		if (max77958_data.is_sent_pin_configuration == 0) {
		/* to do list */
			multi_func_preference =
				DP_ATTENTION->DATA_MSG_DP_STATUS.BITS.Multi_Function_Preference;
			if (multi_func_preference == 1) {
				if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_D) {
					W_DATA = DP_PIN_ASSIGNMENT_D;
					pin_assignment = DP_PIN_ASSIGNMENT_D;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_B) {
					W_DATA = DP_PIN_ASSIGNMENT_B;
					pin_assignment = DP_PIN_ASSIGNMENT_B;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_F) {
					W_DATA = DP_PIN_ASSIGNMENT_F;
					pin_assignment = DP_PIN_ASSIGNMENT_F;
				} else {
					pin_assignment = DP_PIN_ASSIGNMENT_NODE;
					mxim_dbg_printf(MSG_CMD_LOG,"wrong pin assignment value\r\n");
				}
			} else {
				if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_C) {
					W_DATA = DP_PIN_ASSIGNMENT_C;
					pin_assignment = DP_PIN_ASSIGNMENT_C;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_E) {
					W_DATA = DP_PIN_ASSIGNMENT_E;
					pin_assignment = DP_PIN_ASSIGNMENT_E;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_A) {
					W_DATA = DP_PIN_ASSIGNMENT_A;
					pin_assignment = DP_PIN_ASSIGNMENT_A;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_D) {
					W_DATA = DP_PIN_ASSIGNMENT_D;
					pin_assignment = DP_PIN_ASSIGNMENT_D;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_B) {
					W_DATA = DP_PIN_ASSIGNMENT_B;
					pin_assignment = DP_PIN_ASSIGNMENT_B;
				} else if (max77958_data.pin_assignment & DP_PIN_ASSIGNMENT_F) {
					W_DATA = DP_PIN_ASSIGNMENT_F;
					pin_assignment = DP_PIN_ASSIGNMENT_F;
				} else {
					pin_assignment = DP_PIN_ASSIGNMENT_NODE;
                    mxim_dbg_printf(MSG_CMD_LOG,"wrong pin assignment value\r\n");
				}
			}
			max77958_data.dp_selected_pin = pin_assignment;

            mxim_dbg_printf(MSG_CMD_LOG,"2. multi_func_preference %d, W_DATA : %d\r\n",
                multi_func_preference,
                W_DATA);
            
            max77958_vdm_process_set_DP_configure_mode_req(W_DATA);            
            max77958_data.is_sent_pin_configuration = 1;
		} else {
			mxim_dbg_printf(MSG_CMD_LOG,"%s : pin configuration is already sent as \r\n");
		}
		if (DP_ATTENTION->DATA_MSG_DP_STATUS.BITS.HPD_State == 1)
			hpd = CCIC_NOTIFY_HIGH;
		else if (DP_ATTENTION->DATA_MSG_DP_STATUS.BITS.HPD_State == 0)
			hpd = CCIC_NOTIFY_LOW;

		if (DP_ATTENTION->DATA_MSG_DP_STATUS.BITS.HPD_Interrupt == 1)
			hpdirq = CCIC_NOTIFY_IRQ;

	} else {
	/* need to check the F/W code. */
		VDO_MSG = (VDO_MESSAGE_Type *)&vdm_data[8];

		for (i = 0; i < 6; i++)
			mxim_dbg_printf(MSG_CMD_LOG,"2.VDO_%d : %d\n",
				i+1, VDO_MSG->VDO[i]);
	}

	return 0;
}

static int max77958_vdm_dp_configure(char *vdm_data, int len)
{
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = (UND_DATA_MSG_VDM_HEADER_Type *)&vdm_data[4];

	mxim_dbg_printf(MSG_CMD_LOG,"vendor_id = 0x%04x , svid_1 = 0x%04x\r\n", DATA_MSG_VDM->BITS.Standard_Vendor_ID, max77958_data.SVID_1);
	if (max77958_data.SVID_0 == TypeC_DP_SUPPORT) {
        //to do list.
	}
	return 0;
}



void max77958_vendor_msg_response(unsigned char *apcmd_data,int len)
{
	unsigned char vendor_msg[OPCODE_DATA_LENGTH] = {0,};
	union VDM_HEADER_Type vendor_msg_header;
	UND_DATA_MSG_ID_HEADER_Type *DATA_MSG_ID = NULL;
	UND_PRODUCT_VDO_Type *DATA_MSG_PRODUCT = NULL;
    DIS_MODE_DP_CAPA_Type *pDP_DIS_MODE= NULL;
	DP_STATUS_UPDATE_Type *DP_STATUS;
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = NULL;
	uint16_t svid = 0;    
    int i = 0x0;
	uint32_t SVID_DP;

	memcpy(vendor_msg, apcmd_data, OPCODE_DATA_LENGTH);
	memcpy(&vendor_msg_header, &vendor_msg[4], sizeof(vendor_msg_header));
	if ((vendor_msg_header.BITS.VDM_command_type) == VDM_RESPONDER_NAK) {
        mxim_dbg_printf(MSG_CMD_LOG, "Vendor Define message is NAK[%d] \r\n", vendor_msg[1]);        
		return;
	}

	switch (vendor_msg[1]) {
	case OPCODE_ID_VDM_DISCOVER_IDENTITY:
        DATA_MSG_ID = (UND_DATA_MSG_ID_HEADER_Type *)&vendor_msg[8];
        DATA_MSG_PRODUCT = (UND_PRODUCT_VDO_Type *)&vendor_msg[16];
		max77958_data.is_sent_pin_configuration = 0;
		max77958_data.Vendor_ID = DATA_MSG_ID->BITS.USB_Vendor_ID;
		max77958_data.Product_ID = DATA_MSG_PRODUCT->BITS.Product_ID;
		max77958_data.Device_Version = DATA_MSG_PRODUCT->BITS.Device_Version;
		mxim_dbg_printf(MSG_CMD_LOG, "VDM_DISCOVER_IDENTITY\r\n");    
        mxim_dbg_printf(MSG_CMD_LOG, "Vendor_ID : 0x%X, Product_ID : 0x%X Device Version 0x%X",
        DATA_MSG_ID->BITS.USB_Vendor_ID, DATA_MSG_PRODUCT->BITS.Product_ID, DATA_MSG_PRODUCT->BITS.Device_Version);        
		break;
	case OPCODE_ID_VDM_DISCOVER_SVIDS:
		mxim_dbg_printf(MSG_CMD_LOG, "VDM_DISCOVER_SVIDS\r\n");
		max77958_vdm_process_discover_svids(&vendor_msg[0], len);
		break;
	case OPCODE_ID_VDM_DISCOVER_MODES:
		vendor_msg[0] = vendor_msg[2];
		vendor_msg[1] = vendor_msg[3];        
		max77958_vdm_process_discover_mode(&vendor_msg[0], len);        
		break;
	case OPCODE_ID_VDM_ENTER_MODE:
        max77958_vdm_process_enter_mode(&vendor_msg[0], len);        
    
		break;
	case OPCODE_ID_VDM_SVID_DP_STATUS:
		mxim_dbg_printf(MSG_CMD_LOG, "VDM_SVID_DP_STATUS\r\n");
        vendor_msg[0] = vendor_msg[2];
        vendor_msg[1] = vendor_msg[3];
        max77958_vdm_dp_status_update(&vendor_msg[0], len);

		break;
	case OPCODE_ID_VDM_SVID_DP_CONFIGURE:
		mxim_dbg_printf(MSG_CMD_LOG, "VDM_SVID_DP_CONFIGURE!!\r\n");
		max77958_vdm_dp_configure(&vendor_msg[0], len);        
		break;
	case OPCODE_ID_VDM_ATTENTION:
		mxim_dbg_printf(MSG_CMD_LOG, "VDM_ATTENTION\r\n");        
		vendor_msg[0] = vendor_msg[2];
		vendor_msg[1] = vendor_msg[3];        
		max77958_vdm_dp_attention(&vendor_msg[0], len);        
		break;
	case OPCODE_ID_VDM_EXIT_MODE:
		mxim_dbg_printf(MSG_CMD_LOG, "VDM_EXIT_MODE\r\n");
		break;

	default:
		break;
	}
}


void max77705_select_pdo(unsigned char num)
{
    struct max77958_apcmd_data cmd_data;
    if(max77958_data.current_pdo_num != max77958_data.selected_pdo_num){
    cmd_data.opcode = OPCODE_SRCCAP_REQ;
    cmd_data.write_data[0] =num;    
    cmd_data.write_length =1;
    cmd_data.read_length =1;    
    max77958_request_apcmd(&cmd_data);
    mxim_dbg_printf(MSG_CMD_LOG, "REQ POS [%d]\r\n",num);
    }
}

void max77958_current_pdo(unsigned char *_data)
{
	int i;
	unsigned char pdo_num = 0x00;
	int pdo_value;
	int max_cur = 0;
	int max_vol = 0;
	int selected_pos = ((_data[1] >> 3) & 0x07);
	pdo_num = ((_data[1]) & 0x07);

    if(!selected_pos)
           goto exit;
	for (i = 0; i < pdo_num; i++) {
		pdo_value = 0;
		max_cur = 0;
		max_vol = 0;
		pdo_value = (_data[2 + (i * 4)]
			| (_data[3 + (i * 4)] << 8)
			| (_data[4 + (i * 4)] << 16)
			| (_data[5 + (i * 4)] << 24));
		max_cur = (0x3FF & pdo_value)*10;
		max_vol = (0x3FF & (pdo_value >> 10))* 50;
		mxim_dbg_printf(MSG_CMD_LOG, "PDO[%d] = 0x%x MAX_CURR(%d) MAX_VOLT(%d) POS(%d)\r\n",
			i, pdo_value,max_cur, max_vol,selected_pos);        	
	}
    max77958_data.current_pdo_num = selected_pos;
    max77958_data.selected_pdo_num = 0x1;    
    max77705_select_pdo(max77958_data.selected_pdo_num); //send the request message again.
exit:
	return;
}

void max77958_response_pdo_request(unsigned char *data)
{
	unsigned char result = data[1];

	switch (result) {
	case 0x00:
		mxim_dbg_printf(MSG_CMD_LOG,"Sent PDO Request Message to Port Partner(0x%02X)\n", result);
		break;
	case 0xFE:
		mxim_dbg_printf(MSG_CMD_LOG,"Error, SinkTxNg(0x%02X)\n", result);
		break;
	case 0xFF:
		mxim_dbg_printf(MSG_CMD_LOG,"Error, Not in SNK Ready State(0x%02X)\n", result);
		break;
	default:
		break;
	}

	/* retry if the state of sink is not stable yet */
	if (result == 0xFE || result == 0xFF) {
        //to do list
	}
}


void max77958_check_apcmd(const struct max77958_apcmd_data *cmd_data)
{
	int len = cmd_data->read_length;
	unsigned char data[OPCODE_DATA_LENGTH] = {0,};
	unsigned char vdm_opcode_header = 0x0;    
	unsigned char response = 0xff;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;
	unsigned char vdm_command = 0x0;
	unsigned char vdm_type = 0x0;
	unsigned char vdm_response = 0x0;
	unsigned char reqd_vdm_command = 0;
	uint8_t W_DATA = 0x0;
    int i=0x0;


	max77958_i2c_opcode_read(&cmd_data->opcode,
			len, data);

	/* opcode identifying the messsage type. (0x51)*/
	response = data[0];

	if (response != cmd_data->response) {
	mxim_dbg_printf(MSG_CMD_LOG, "Response [0x%02x] != [0x%02x]\r\n",
				response, cmd_data->response);
	}

	/* to do(read switch case) */
	switch (response) {
	case OPCODE_BC_CTRL1_R:
	case OPCODE_BC_CTRL1_W:
	case OPCODE_BC_CTRL2_R:
	case OPCODE_BC_CTRL2_W:
	case OPCODE_CONTROL1_R:
	case OPCODE_CONTROL1_W:
	case OPCODE_CCCONTROL1_R:
	case OPCODE_CCCONTROL1_W:
	case OPCODE_CCCONTROL2_R:
	case OPCODE_CCCONTROL2_W:
	case OPCODE_CCCONTROL3_R:
	case OPCODE_CCCONTROL3_W:
	case OPCODE_CCCONTROL4_R:
	case OPCODE_CCCONTROL4_W:
	case OPCODE_VCONN_ILIM_R:
	case OPCODE_VCONN_ILIM_W:
	case OPCODE_CCCONTROL5_R:
	case OPCODE_CCCONTROL5_W:
	case OPCODE_CCCONTROL6_R:
	case OPCODE_CCCONTROL6_W:
	case OPCODE_GET_SINK_CAP:
	case OPCODE_GET_SOURCE_CAP:        
		break;
	case OPCODE_CURRENT_SRC_CAP:
		max77958_current_pdo(data);
		break;

	case OPCODE_SRCCAP_REQ:
		/*
		 * If response of Source_Capablities message is SinkTxNg(0xFE) or Not in Ready State(0xFF)
		 * It means that the message can not be sent to Port Partner.
		 * After Attaching Rp 3.0A, send again the message.
		 */
		 max77958_response_pdo_request(data);
        break;
	case OPCODE_SET_SOURCE_CAP:
	case OPCODE_SEND_GET_REQ:
	case OPCODE_READ_GET_REQ_RESP:
	case OPCODE_SEND_GET_RESP:
	case OPCODE_SWAP_REQ:
	case OPCODE_SWAP_RESP:
	case OPCODE_VDM_DISCOVER_ID_RESP:
	case OPCODE_VDM_DISCOVER_ID_REQ:
	case OPCODE_VDM_DISCOVER_SVID_RESP:
	case OPCODE_VDM_DISCOVER_SVID_REQ:
	case OPCODE_VDM_DISCOVER_MODE_RESP:
	case OPCODE_VDM_DISCOVER_MODE_REQ:
	case OPCODE_VDM_ENTER_MODE_REQ:
	case OPCODE_VDM_EXIT_MODE_REQ:
        break;
	case OPCODE_SEND_VDM:
        vdm_opcode_header = data[1];
        switch (vdm_opcode_header) {
        case 0xFF:
        	mxim_dbg_printf(MSG_CMD_LOG,
        	"This isn't invalid response(OPCODE : 0x48, HEADER : 0xFF)\r\n");
        	break;
        default:
        	memcpy(&vdm_header, &data[2], sizeof(vdm_header));
        	vdm_type = vdm_header.BITS.VDM_Type;
        	vdm_command = vdm_header.BITS.VDM_command;
        	vdm_response = vdm_header.BITS.VDM_command_type;
        	mxim_dbg_printf(MSG_CMD_LOG,
                "vdm_type[%x], vdm_command[%x], vdm_response[%x]\r\n",
        		vdm_type, vdm_command, vdm_response);
        	switch (vdm_type) {
        	case STRUCTURED_VDM:
                		if (vdm_response == SEC_UVDM_RESPONDER_ACK) {
                			switch (vdm_command) {
                			case Discover_Identity:
                				break;
                			case Discover_SVIDs:
                				break;
                			case Discover_Modes:
                				break;
                			case Enter_Mode:
                				break;
                			case Exit_Mode:
                				break;
                			case Attention:
                				break;
                			case Configure:
                				break;
                			default:
                				mxim_dbg_printf(MSG_CMD_LOG,
                                    "vdm_command isn't valid[%x]\r\n", vdm_command);
                				break;
                			};
                		} else if (vdm_response == SEC_UVDM_ININIATOR) {
                            
                			switch (vdm_command) {
                			case Attention:
                				/* Attention message is not able to be received via 0x48 OPCode */
                				/* Check requested vdm command and responded vdm command */
                				{
                					/* Read requested vdm command */
                                    i2c_read_byte(MAX77958_SLAVE_P1, 0x23, &reqd_vdm_command);                            
                					reqd_vdm_command &= 0x1F; /* Command bit, b4...0 */

                					if (reqd_vdm_command == Configure) {
                						W_DATA = 1 << (max77958_data.dp_selected_pin - 1);
                						/* Retry Configure message */
                						mxim_dbg_printf(MSG_CMD_LOG,"Retry Configure message, W_DATA = %x, dp_selected_pin = %d\r\n",
                								W_DATA, max77958_data.dp_selected_pin);
                						max77958_vdm_process_set_DP_configure_mode_req(W_DATA);
                					}
                				}
                				break;
                			case Discover_Identity:
                			case Discover_SVIDs:
                			case Discover_Modes:
                			case Enter_Mode:
                			case Configure:
                			default:
                				/* Nothing */
                				break;
                			};
                		} else
                			mxim_dbg_printf(MSG_CMD_LOG,"vdm_response is error value[%x]\r\n", vdm_response);
                }
            }
		break;        
	case OPCODE_GET_PD_MSG:
		break;
	case OPCODE_GET_VDM_RESP:
		max77958_vendor_msg_response(data,len + OPCODE_SIZE);
		break;
	case OPCODE_SET_CSTM_INFORMATION_R:
	case OPCODE_SET_CSTM_INFORMATION_W:
	case OPCODE_SET_DEVCONFG_INFORMATION_R:
	case OPCODE_SET_DEVCONFG_INFORMATION_W:
	case OPCODE_ACTION_BLOCK_MTP_UPDATE:
		break;

	case OPCODE_MASTER_I2C_READ:
	case OPCODE_MASTER_I2C_WRITE:
		break;
	default:
		break;
	}
}
void max77958_current_pdo_num(unsigned char *_data)
{
	int i;
	unsigned char pdo_num = 0x00;
	int pdo_value;
	int max_cur = 0;
	int max_vol = 0;
	int selected_pos = ((_data[1] >> 3) & 0x07);

	max77958_data.current_pdo_num=selected_pos;
	pdo_num = ((_data[1]) & 0x07);

    if(!selected_pos)
           goto exit;
	for (i = 0; i < pdo_num; i++) {
		pdo_value = 0;
		max_cur = 0;
		max_vol = 0;
		pdo_value = (_data[2 + (i * 4)]
			| (_data[3 + (i * 4)] << 8)
			| (_data[4 + (i * 4)] << 16)
			| (_data[5 + (i * 4)] << 24));
        max77958_data.sink_status_power_list[i] = pdo_value;
		max_cur = (0x3FF & pdo_value)*10;
		max_vol = (0x3FF & (pdo_value >> 10))* 50;
		mxim_dbg_printf(MSG_CMD_LOG, "PDO[%d] = 0x%x MAX_CURR(%d) MAX_VOLT(%d) POS(%d)\r\n",
			i, pdo_value,max_cur, max_vol,selected_pos);

		if(selected_pos == (i+1)){
            max77958_data.max_power = max_cur*max_vol;
            if((0x1FFF & max_cur) > 100)
                max77958_data.max_current = ((0x1FFF & max_cur) + 50) / 50;
            else    
                max77958_data.max_current = 0x00;
        }
	}
exit:  
    return;
}


void max77958_set_path_dpdn(){
    struct max77958_apcmd_data cmd_data;
    int num = 0x0;
    switch (max77958_data.cur_ccstat) {
        case CC_NO_CON :
                break;
        case CC_SNK:
                switch (max77958_data.chgtype) {
                    case CHGTYP_USB_SDP:  
                        num = 0x9;
                        break;
                    case CHGTYP_CDP:
                        break;
                    case CHGTYP_DCP:
                        break;
                    default:
                        break;
                }            
                break;
        case CC_SRC:                    
                break;                
        default:
                break;
    }
    cmd_data.opcode = OPCODE_CONTROL1_W;
    cmd_data.write_data[0] =num;    
    cmd_data.write_length =1;
    cmd_data.read_length =1;    
    max77958_request_apcmd(&cmd_data);
}

void max77958_check_maxpower(){

	int array[3] = {0,};
	int i = 0;
	unsigned int max_power =0;

	switch (max77958_data.ccistat) {

		case CCISTAT_500MA:
			array[0] = PW_5WH;
			break;

		case CCISTAT_1500MA:
			array[0] = PW_15WH;
			break;

		case CCISTAT_3000MA:
			array[0] = PW_30WH;
			break;
	}
    
	switch (max77958_data.chgtype) {
		case CHGTYP_USB_SDP:
			array[1] = PW_5WH;
			break;
		case CHGTYP_CDP:
			array[1] = PW_15WH;
			break;
		case CHGTYP_DCP:
			array[1] = PW_15WH;
			break;
	}
    
	max_power = array[0];
	for(i =1 ; i<2; i++) {
		if(max_power < array[i])
			max_power = array[i];
	}
	max77958_data.max_voltage = max_power;
}

void max77958_check_ccistat(){
	switch (max77958_data.ccistat) {

		case CCISTAT_500MA:
        	mxim_dbg_printf(MSG_CMD_LOG, "CCISTAT_500MA\r\n");
			break;

		case CCISTAT_1500MA:
        	mxim_dbg_printf(MSG_CMD_LOG, "CCISTAT_1500MA\r\n");
			break;

		case CCISTAT_3000MA:
            mxim_dbg_printf(MSG_CMD_LOG, "CCISTAT_3000MA\r\n");
			break;
	}
    max77958_check_maxpower();
}


void max77958_check_chgtype(){
	switch (max77958_data.chgtype) {
		case CHGTYP_USB_SDP:            
            mxim_dbg_printf(MSG_CMD_LOG, "USB_SDP\r\n");            
            max77958_set_path_dpdn();
			break;
		case CHGTYP_CDP:
             mxim_dbg_printf(MSG_CMD_LOG, "CDP\r\n");
			break;
		case CHGTYP_DCP:            
            mxim_dbg_printf(MSG_CMD_LOG, "DCP\r\n");
			break;
	}
    max77958_check_maxpower();
}

void max77958_cc_no_connection()
{
    //TO Do list.
    max77958_set_path_dpdn();
    max77958_circ_bbuf_init();
    max77958_data.current_dr = 0xFF;
    max77958_data.previous_dr = 0xFF;    
    max77958_data.cur_ccstat = 0xFF;
    max77958_data.current_pdo_num = 0x0;    
    max77958_data.selected_pdo_num = 0x0;
        
}

void max77958_cc_sink()
{
    //TO Do list.
        
}

void max77958_cc_source()
{
    //TO Do list.
        
}



void max77958_probe_func(){
    int slave = MAX77958_SLAVE_P1;
#if 0
	max77958_usbc_fw_update(slave, max77958,
		ARRAY_SIZE(max77958), 0);
#endif
    max77958_circ_bbuf_init();
    max77958_irq_thread();
	i2c_write_byte(slave, REG_UIC_INT_M, REG_UIC_INT_M_INIT);
	i2c_write_byte(slave, REG_CC_INT_M, REG_CC_INT_M_INIT);
	i2c_write_byte(slave, REG_PD_INT_M, REG_PD_INT_M_INIT);
	i2c_write_byte(slave, REG_VDM_INT_M, REG_VDM_INT_M_INIT); 
}


void max77958_pd_check_pdmsg(unsigned char pd_msg){

    mxim_dbg_printf(MSG_CMD_LOG, "PDMSG [%x]\r\n",pd_msg);


    switch (pd_msg) {
        case PDMSG_SNK_PSRDY_RECEIVED :
            mxim_dbg_printf(MSG_CMD_LOG, "PDMSG_SNK_PSRDY_RECEIVED\r\n");
            break;       
        case PDMSG_HARDRESET_SENT:
            mxim_dbg_printf(MSG_CMD_LOG, "PDMSG_HARDRESET_SENT\r\n");
            /* Setting the BC52 Configuration */
            break;
        case PDMSG_HARDRESET_RECEIVED:            
            mxim_dbg_printf(MSG_CMD_LOG, "PDMSG_HARDRESET_RECEIVED\r\n");                    
            /* Setting the BC52 Configuration */
            break;

        case PDMSG_PRSWAP_SRCTOSWAP:
        case PDMSG_PRSWAP_SWAPTOSNK:
            //turn off VBUS
            mxim_dbg_printf(MSG_CMD_LOG,"PDMSG_PRSWAP_SRCTOSWAPSNK : [%x]\r\n", pd_msg);
            break;
        case PDMSG_PRSWAP_SNKTOSWAP:
            mxim_dbg_printf(MSG_CMD_LOG,"PDMSG_PRSWAP_SNKTOSWAP received\r\n");
            /* CHGINSEL disable */
            break;
        case PDMSG_PRSWAP_SWAPTOSRC:
            //turn on VBUS
            mxim_dbg_printf(MSG_CMD_LOG,"PDMSG_PRSWAP_SWAPTOSRC received\r\n");
            break;

            
        default:
            break;
    }


}

void max77958_check_pdo()
{
	struct max77958_apcmd_data enqueue_data;    
	enqueue_data.opcode = OPCODE_CURRENT_SRC_CAP;
	enqueue_data.write_length = 0x1;
	enqueue_data.read_length = 31;
	mxim_dbg_printf(MSG_CMD_LOG,"max77958_check_pdo\r\n");
	max77958_request_apcmd(&enqueue_data);
}


void max77958_reset_ic()
{
    int _sid = MAX77958_SLAVE_P1;
    unsigned char uic_int =0;
    
	i2c_write_byte(_sid,0x80, 0x0F);
//  wait for 100ms
//	msleep(100); //250000
    opcode_delay(100000); //wait for 500ms
    i2c_read_byte(_sid, REG_UIC_INT, &uic_int);
    i2c_write_byte(_sid, REG_UIC_INT_M,
        REG_UIC_INT_M_INIT);
    i2c_write_byte(_sid, REG_CC_INT_M,
        REG_CC_INT_M_INIT);
    i2c_write_byte(_sid, REG_PD_INT_M,
        REG_PD_INT_M_INIT);
	i2c_write_byte(_sid,
        REG_VDM_INT_M, REG_VDM_INT_M_INIT);
}

void max77958_execute_sysmsg(int sysmsg)
{

    struct max77958_apcmd_data cmd_data;

    mxim_dbg_printf(MSG_CMD_LOG, "SYSMSG IRQ [%x] \r\n",
        sysmsg);

	switch (sysmsg) {
	case SYSERROR_NONE:
		break;
	case SYSERROR_BOOT_WDT:
	case SYSMSG_BOOT_POR:
        if (max77958_data.boot_complete) {          
            max77958_reset_ic();        
        }
		break;
    case SYSERROR_BOOT_SWRSTREQ:
		break;

	case SYSERROR_AFC_DONE:
		break;

	case SYSERROR_APCMD_UNKNOWN:
		break;
	case SYSERROR_APCMD_INPROGRESS:
		break;
	case SYSERROR_APCMD_FAIL:
        g_max77958_apcmd_queue.tail = g_max77958_apcmd_queue.prev_tail;        
        max77958_circ_bbuf_pop(&cmd_data);
        if (cmd_data.opcode != OPCODE_SEND_VDM){
            max77958_send_apcmd();            
        }
		break;
	case SYSERROR_CCx_5V_SHORT:        
        mxim_dbg_printf(MSG_CMD_LOG, "SYSERROR_CCx_5V_SHORT \r\n");
		break;
	default:
		break;
	}
}


void max77958_check_ccpinstat()
{
	unsigned char ccpinstat = 0;

	ccpinstat = (max77958_data.cc_status0 & BIT_CCPINSTAT)
		>> FFS(BIT_CCPINSTAT);

	switch (ccpinstat) {
	case NO_DETERMINATION:
		mxim_dbg_printf(MSG_CMD_LOG,"CCPINSTAT : [NO_DETERMINATION]\r\n");
		break;
	case CC1_ACTIVE:
		mxim_dbg_printf(MSG_CMD_LOG,"CCPINSTAT : [CC1_ACTIVE]\r\n");        
		break;
	case CC2_ACTVIE:
		mxim_dbg_printf(MSG_CMD_LOG,"CCPINSTAT : [CC2_ACTIVE]\r\n");                
		break;
	default:
		mxim_dbg_printf(MSG_CMD_LOG,"CCPINSTAT [%d]\r\n", ccpinstat);
		break;

	}
}



void max77958_check_ccvnstat()
{
	unsigned char ccvcnstat = 0;

	ccvcnstat = (max77958_data.cc_status0 & BIT_CCVCNSTAT) >> FFS(BIT_CCVCNSTAT);

	switch (ccvcnstat) {
	case 0:
		mxim_dbg_printf(MSG_CMD_LOG,"Vconn Disabled\r\n");        
		break;
	case 1:
		mxim_dbg_printf(MSG_CMD_LOG,"Vconn Enabled\r\n");        
		break;
	default:
		mxim_dbg_printf(MSG_CMD_LOG,"ccvnstat(Never Call this routine)\r\n");                
		break;

	}
}

void max77958_check_pd_psrdy()
{
	unsigned char ccvcnstat = 0;
	unsigned char psrdy_received = 0;

	psrdy_received = (max77958_data.pd_status1 & BIT_PD_PSRDY)
			>> FFS(BIT_PD_PSRDY);


    if(max77958_data.cur_ccstat == CC_SNK && psrdy_received) {
        max77958_check_pdo();
        mxim_dbg_printf(MSG_CMD_LOG, "Receive the PSRDY_IRQ(SINK)\r\n");
    }else if(max77958_data.cur_ccstat == CC_SRC){   
        mxim_dbg_printf(MSG_CMD_LOG, "Sent the PSRDY_IRQ(Source)\r\n");
    }else {

    }

}

void max77958_check_datarole()
{

	unsigned char data_role = 0;

    data_role=(max77958_data.pd_status1 & BIT_DATAROLE)
        >> FFS(BIT_DATAROLE);


	switch (data_role) {
	case UFP:
		if (max77958_data.current_dr != UFP) {
			max77958_data.previous_dr = max77958_data.current_dr;
			max77958_data.current_dr = UFP;
		}
        mxim_dbg_printf(MSG_CMD_LOG,"UFP\r\n");  
		break;

	case DFP:
		if (max77958_data.current_dr != DFP) {
			max77958_data.previous_dr = max77958_data.current_dr;
			max77958_data.current_dr = DFP;
            if(max77958_data.cur_ccstat==CC_SNK){
                max77958_vdm_process_set_identity_req();
            }
		}
        mxim_dbg_printf(MSG_CMD_LOG,"DFP\r\n");  
		break;
   }
    

}

void max77958_check_ccstat()
{
    int ccstat = 0x0;
    ccstat=(max77958_data.cc_status0 & BIT_CCSTAT) >> FFS(BIT_CCSTAT);
    switch (ccstat) {
        case CC_NO_CON :
                mxim_dbg_printf(MSG_CMD_LOG, "[MAX77958_P1] CC_NO_CONNECTION\r\n");
                max77958_data.cur_ccstat=ccstat;
                max77958_cc_no_connection();
                break;
        case CC_SNK:
                mxim_dbg_printf(MSG_CMD_LOG, "[MAX77958_P1] CC_SINK\r\n");
                max77958_data.cur_ccstat=ccstat;
                max77958_cc_sink();
                break;
        case CC_SRC:                    
                mxim_dbg_printf(MSG_CMD_LOG, "[MAX77958_P1] CC_SOURCE\r\n");
                max77958_data.cur_ccstat=ccstat;
                max77958_cc_source();
                break;                
        default:
                mxim_dbg_printf(MSG_CMD_LOG, "[MAX77958_P1] [%x]\r\n", ccstat);                
                break;
    }

    

}

void max77958_vsafe0v_irq()
{
    unsigned char vsafe0v = 0x0;
	vsafe0v = (max77958_data.cc_status1 & BIT_VSAFE0V)
		>> FFS(BIT_VSAFE0V);
	if (vsafe0v) {
		mxim_dbg_printf(MSG_CMD_LOG, " VSAFE5V\r\n");
    } else {
		mxim_dbg_printf(MSG_CMD_LOG, " VSAFE0V\r\n");
	}
}

void max77958_vconncop_irq()
{
    unsigned char vconnocp = 0x0;
	vconnocp = (max77958_data.cc_status1  & BIT_VCONNOCPI)
		>> FFS(BIT_VCONNOCPI);
    mxim_dbg_printf(MSG_CMD_LOG, " VCONNOCPI : [%x]\r\n",vconnocp);
}


void max77958_vbusdet_irq()
{
	if (max77958_data.vbus_on) {
		mxim_dbg_printf(MSG_CMD_LOG, " VBUS > VBDET\r\n");
    } else {
		mxim_dbg_printf(MSG_CMD_LOG, "VBUS < VBDET\r\n");
	}
}

void max77958_check_vadc()
{
    unsigned char vbadc = 0x0;
    vbadc = (max77958_data.usbc_status1 & BIT_VBADC) >> FFS(BIT_VBADC);
	mxim_dbg_printf(MSG_CMD_LOG," ADC [%x]\r\n", vbadc);
}


void max77958_check_dcdtmo()
{
    unsigned char dcdtmo = 0x0;
    dcdtmo = (max77958_data.bc_status& BIT_DCDTMO) >> FFS(BIT_DCDTMO);    
	mxim_dbg_printf(MSG_CMD_LOG, " BIT_DCDTmoI occured [%x]\r\n", dcdtmo);

}

void max77958_irq_thread(){

        int slave = MAX77958_SLAVE_P1;
        int ccstat_p0, ccstat_p1;
        int i = 0;
        unsigned char uic_int[4]={0,};
        unsigned char uic_int_m[4]={0,};        
    
        max77958_data.reg_uic_int = 0x0;
        max77958_data.reg_cc_int = 0x0;
        max77958_data.reg_pd_int = 0x0;
        max77958_data.reg_vdm_int = 0x0;
        max77958_data.usbc_status1 = 0x0;
        max77958_data.usbc_status2 = 0x0;
        max77958_data.bc_status = 0x0;
        max77958_data.cc_status0 = 0x0;
        max77958_data.cc_status1 = 0x0;
        max77958_data.pd_status0 = 0x0;
        max77958_data.pd_status1 = 0x0;
    
        i2c_read_nbytes(slave,REG_UIC_INT,&uic_int[0],4);
        max77958_data.reg_uic_int = uic_int[0];
        max77958_data.reg_cc_int = uic_int[1];
        max77958_data.reg_pd_int = uic_int[2];
        max77958_data.reg_vdm_int = uic_int[3];
        i2c_read_byte(slave,REG_USBC_STATUS1,&max77958_data.usbc_status1);
        i2c_read_byte(slave,REG_USBC_STATUS2,&max77958_data.usbc_status2);
        i2c_read_byte(slave,REG_BC_STATUS,&max77958_data.bc_status);
        i2c_read_byte(slave,REG_CC_STATUS0,&max77958_data.cc_status0);
        i2c_read_byte(slave,REG_CC_STATUS1,&max77958_data.cc_status1);
        i2c_read_byte(slave,REG_PD_STATUS0,&max77958_data.pd_status0);
        i2c_read_byte(slave,REG_PD_STATUS1,&max77958_data.pd_status1);



        i2c_read_byte(slave, REG_UIC_INT_M, &uic_int_m[0]);
        i2c_read_byte(slave, REG_CC_INT_M, &uic_int_m[1]);
        i2c_read_byte(slave, REG_PD_INT_M, &uic_int_m[2]);
        i2c_read_byte(slave, REG_VDM_INT_M, &uic_int_m[3]); 


        mxim_dbg_printf(MSG_CMD_LOG, "USBCM = 0x%x CCM = 0x%x PDM = 0x%x VDMM = 0x%x\r\n",
            uic_int_m[0],
            uic_int_m[1],
            uic_int_m[2],
            uic_int_m[3]);


        mxim_dbg_printf(MSG_CMD_LOG, "USBCI = 0x%x CCI = 0x%x PDI = 0x%x VDMI = 0x%x\r\n",
            max77958_data.reg_uic_int,
            max77958_data.reg_cc_int,
            max77958_data.reg_pd_int,
            max77958_data.reg_vdm_int);

    
    
        mxim_dbg_printf(MSG_CMD_LOG, "USBC = 0x%x BC = 0x%x CC0 = 0x%x CC1 = 0x%x PD0 = 0x%x PD1 = 0x%x\r\n",
            max77958_data.usbc_status1, 
            max77958_data.bc_status,
            max77958_data.cc_status0,
            max77958_data.cc_status1,
            max77958_data.pd_status0,
            max77958_data.pd_status1);

        max77958_data.ccistat = (max77958_data.cc_status0 & BIT_CCISTAT)
                >> FFS(BIT_CCISTAT);
        
        max77958_data.chgtype = (max77958_data.bc_status & BIT_CHGTYP)
                >> FFS(BIT_CHGTYP);
        
        max77958_data.prchgtype =  (max77958_data.bc_status & BIT_PRCHGTYP)
                >> FFS(BIT_PRCHGTYP);
        
        max77958_data.vsafe5V = (max77958_data.cc_status1 & BIT_VSAFE0V)
            >> FFS(BIT_VSAFE0V);
        
        max77958_data.vbus_on = (max77958_data.bc_status & BIT_VBUSDET)
            >> FFS(BIT_VBUSDET);
            

        if(max77958_data.reg_uic_int & BIT_APCMDRESI){
            max77958_execute_apcmd();
        }
            
    
        if(max77958_data.reg_cc_int & BIT_CCSTATI){    
            max77958_check_ccstat();
        }
        

        if(max77958_data.reg_cc_int & BIT_CCISTATI){    
            max77958_check_ccistat();            
        }        


        if(max77958_data.reg_cc_int & BIT_CCPINSTATI){    
            max77958_check_ccpinstat();
        }        

        if(max77958_data.reg_cc_int & BIT_CCIVCNSTATI){    
            max77958_check_ccvnstat();

        }        

        if(max77958_data.reg_uic_int & BIT_VBUSDETI){    
            max77958_vbusdet_irq();

        }        

        if(max77958_data.reg_uic_int & BIT_CHGTYPI){    
            max77958_check_chgtype();            
        }        
        
        if(max77958_data.reg_pd_int & BIT_DATAROLEI){
            max77958_check_datarole();
        }
                
        if(max77958_data.reg_pd_int & BIT_PSRDYI){
            max77958_check_pd_psrdy();
        }

        if(max77958_data.reg_pd_int & BIT_PDMSGI){
            max77958_pd_check_pdmsg(max77958_data.pd_status0);
        }

        if((max77958_data.reg_pd_int& BIT_VDMI)){
            max77958_vdm_irq(); 
        }    

        if(max77958_data.reg_uic_int & BIT_VBADCI){    
            max77958_check_vadc();

        }   

        if(max77958_data.reg_uic_int & BIT_DCDTMOI){    
            max77958_check_dcdtmo();

        }   

        if(max77958_data.reg_cc_int & BIT_VCONNOCPI){    
            max77958_vconncop_irq();
        }

        if(max77958_data.reg_cc_int & BIT_VSAFE0VI){    
            max77958_vsafe0v_irq();
        }

        
        if(max77958_data.reg_uic_int & BIT_SYSMSGI){
            max77958_execute_sysmsg(max77958_data.usbc_status2);
        }
}

