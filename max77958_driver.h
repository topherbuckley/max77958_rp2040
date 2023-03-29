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

#ifndef PRJ_DEVICE_MAX77958_H_
#define PRJ_DEVICE_MAX77958_H_

#include <stdint.h>

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
	((_x) & 0x0F ? \
	 ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) : ((_x) & 0x04 ? 2 : 3)) : \
	 ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) : ((_x) & 0x40 ? 6 : 7)))

#undef FFS
#define FFS(_x) \
	((_x) ? __CONST_FFS(_x) : 0)

#undef  BIT_RSVD
#define BIT_RSVD  0

#undef  BITS
#define BITS(_end, _start) \
	((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_word, _mask, _shift) \
	(((_word) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_word, _bit) \
	__BITS_GET(_word, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_word, _mask, _shift, _val) \
	(((_word) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_word, _bit, _val) \
	__BITS_SET(_word, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_word, _bit) \
	(((_word) & (_bit)) == (_bit))
#define BIT(nr)         (1UL << (nr))


#define OPCODE_WAIT_TIMEOUT (3000) /* 3000ms */

#define OPCODE_WRITE_COMMAND 0x21
#define OPCODE_READ_COMMAND 0x51
#define OPCODE_SIZE 1
#define OPCODE_DATA_LENGTH 32
#define OPCODE_MAX_LENGTH (OPCODE_DATA_LENGTH + OPCODE_SIZE)
#define OPCODE_WRITE 0x21
#define OPCODE_WRITE_END 0x41
#define OPCODE_READ 0x51
#define GPIO_OUTPUT 0x1


#define PDO_LIST_NUM				0x8

#define	REG_UIC_HW_REV			0x00
#define	REG_UIC_DEVICE_ID		0x01
#define	REG_UIC_FW_REV			0x02
#define	REG_UIC_FW_SUBREV		0x03
#define	REG_UIC_INT				0x04
#define	REG_CC_INT				0x05
#define	REG_PD_INT				0x06
#define	REG_VDM_INT				0x07
#define	REG_USBC_STATUS1		0x08
#define	REG_USBC_STATUS2		0x09
#define	REG_BC_STATUS			0x0A


#define REG_DP_STATUS           0x0B
#define	REG_CC_STATUS0			0x0C
#define	REG_CC_STATUS1			0x0D

#define	REG_PD_STATUS0			0x0E
#define	REG_PD_STATUS1			0x0F

#define	REG_UIC_INT_M			0x10
#define	REG_CC_INT_M			0x11
#define	REG_PD_INT_M			0x12
#define	REG_VDM_INT_M			0x13


#define MAX77958_SLAVE_P1		0x25


#define BIT_APCMDRESI			BIT(7)
#define BIT_SYSMSGI				BIT(6)
#define BIT_VBUSDETI			BIT(5)
#define BIT_VBADCI				BIT(4)
#define BIT_DCDTMOI				BIT(3)
#define BIT_CHGTYPI				BIT(1)


#define BIT_VCONNOCPI			BIT(7)
#define BIT_VSAFE0VI			BIT(6)
#define BIT_CCPINSTATI			BIT(3)
#define	BIT_CCISTATI			BIT(2)
#define	BIT_CCIVCNSTATI			BIT(1)
#define	BIT_CCSTATI				BIT(0)


#define BIT_PDMSGI				BIT(7)
#define BIT_PSRDYI				BIT(6)
#define BIT_DATAROLEI			BIT(5)
#define BIT_VDMI				BIT(2)



#define BIT_Action3I			BIT(7)
#define BIT_Action2I			BIT(6)
#define BIT_Action1I			BIT(5)
#define BIT_Action0I			BIT(4)


#define BIT_VBUSDET				BIT(7)
#define BIT_PRCHGTYP			BITS(5, 3)
#define BIT_DCDTMO				BIT(2)
#define BIT_CHGTYP				BITS(1, 0)


#define BIT_CCPINSTAT			BITS(7, 6)
#define BIT_CCISTAT				BITS(5, 4)
#define BIT_CCVCNSTAT			BIT(3)
#define BIT_CCSTAT				BITS(2, 0)



#define BIT_VSAFE0V				BIT(3)


#define BIT_VBADC				BITS(7, 3)

#define BIT_SYSMSG				BITS(7, 0)

#define BIT_DATAROLE			BIT(7)
#define BIT_PD_PSRDY			BIT(4)


#define VSAFE5V						0x1
#define VSAFE0V						0x0


#define CONNECTION                   0x1
#define NO_CONNECTION                0x0


#define REG_UIC_INT_M_INIT		0x04
#define REG_CC_INT_M_INIT		0x20
#define REG_PD_INT_M_INIT		0x00
#define REG_VDM_INT_M_INIT		0x00


enum MAX77958_CHG_TYPE {
	CHGTYP_NOTHING = 0,
	CHGTYP_USB_SDP,
	CHGTYP_CDP,
	CHGTYP_DCP,
};
enum MAX77958_PR_CHG_TYPE {
	UNKNOWN = 0,
	RSVD0,
	RSVD1,
	RSVD2,
	RSVD3,
	RSVD4,
	RSVD5,
	NIKON_TA,
};
enum MAX77958_CCISTAT_TYPE{
	CCISTAT_NONE,
	CCISTAT_500MA,
	CCISTAT_1500MA,
	CCISTAT_3000MA,
};

enum {
	NO_DETERMINATION = 0,
	CC1_ACTIVE,
	CC2_ACTVIE,
};


#define OPCODE_WAIT_TIMEOUT (3000) /* 3000ms */

#define OPCODE_WRITE_COMMAND 0x21
#define OPCODE_READ_COMMAND 0x51
#define OPCODE_SIZE 1
#define OPCODE_DATA_LENGTH 32
#define OPCODE_WRITE 0x21
#define OPCODE_WRITE_END 0x41
#define OPCODE_READ 0x51
#define GPIO_OUTPUT 0x1

enum MAX77958_OPCODE_LIST{
	OPCODE_BC_CTRL1_R			 = 0x01,
	OPCODE_BC_CTRL1_W			 = 0x02,
	OPCODE_BC_CTRL2_R			 = 0x03,
	OPCODE_BC_CTRL2_W			 = 0x04,
	OPCODE_CONTROL1_R			 = 0x05,
	OPCODE_CONTROL1_W			 = 0x06,
	OPCODE_CCCONTROL1_R			 = 0x0B,
	OPCODE_CCCONTROL1_W			 = 0x0C,
	OPCODE_CCCONTROL2_R			 = 0x0D,
	OPCODE_CCCONTROL2_W			 = 0x0E,
	OPCODE_CCCONTROL3_R			 = 0x0F,
	OPCODE_CCCONTROL3_W			 = 0x10,
	OPCODE_CCCONTROL4_R			 = 0x11,
	OPCODE_CCCONTROL4_W			 = 0x12,
	OPCODE_VCONN_ILIM_R			 = 0x13,
	OPCODE_VCONN_ILIM_W			 = 0x14,
	OPCODE_CCCONTROL5_R			 = 0x15,
	OPCODE_CCCONTROL5_W			 = 0x16,
	OPCODE_CCCONTROL6_R			 = 0x17,
	OPCODE_CCCONTROL6_W			 = 0x18,
	OPCODE_GET_SINK_CAP			 = 0x2F,
	OPCODE_READ_GPIO			 = 0x23,
	OPCODE_SET_GPIO				 = 0x24,
	OPCODE_CURRENT_SRC_CAP		 = 0x30,
	OPCODE_GET_SOURCE_CAP		 = 0x31,
	OPCODE_SRCCAP_REQ			 = 0x32,
	OPCODE_SET_SOURCE_CAP		 = 0x33,
	OPCODE_SEND_GET_REQ			 = 0x34,
	OPCODE_READ_GET_REQ_RESP 	 = 0x35,
	OPCODE_SEND_GET_RESP 		 = 0x36,
	OPCODE_SWAP_REQ				 = 0x37,
	OPCODE_SWAP_RESP 			 = 0x38,
	OPCODE_VDM_DISCOVER_ID_RESP	 = 0x40,
	OPCODE_VDM_DISCOVER_ID_REQ	 = 0x41,
	OPCODE_VDM_DISCOVER_SVID_RESP = 0x42,
	OPCODE_VDM_DISCOVER_SVID_REQ  = 0x43,
	OPCODE_VDM_DISCOVER_MODE_RESP = 0x44,
	OPCODE_VDM_DISCOVER_MODE_REQ  = 0x45,
	OPCODE_VDM_ENTER_MODE_REQ	 = 0x46,
	OPCODE_VDM_EXIT_MODE_REQ 	 = 0x47,
	OPCODE_SEND_VDM				 = 0x48,
	OPCODE_GET_PD_MSG			 = 0x4A,
	OPCODE_GET_VDM_RESP			 = 0x4B,
	OPCODE_SET_CSTM_INFORMATION_R  = 0x55, // Customer VIF Information
	OPCODE_SET_CSTM_INFORMATION_W  = 0x56, // Customer VIF Information
	OPCODE_SET_DEVCONFG_INFORMATION_R  = 0x57, // Device Config Information
	OPCODE_SET_DEVCONFG_INFORMATION_W  = 0x58, // Device Config Information
	OPCODE_ACTION_BLOCK_MTP_UPDATE  = 0x60, // Action Block update
	OPCODE_MASTER_I2C_READ  = 0x85, // I2C_READ
	OPCODE_MASTER_I2C_WRITE  = 0x86, // I2C_WRITE
	OPCODE_RSVD = 0xFF,
};

enum MAX77958_PDMSG {
	PDMSG_NOTHING_HAPPENED                = 0x00,
	PDMSG_SNK_PSRDY_RECEIVED              = 0x01,
	PDMSG_SNK_ERROR_RECOVERY              = 0x02,
	PDMSG_SNK_SENDERRESPONSETIMER_TIMEOUT = 0x03,
	PDMSG_SRC_PSRDY_SENT                  = 0x04,
	PDMSG_SRC_ERROR_RECOVERY              = 0x05,
	PDMSG_SRC_SENDERRESPONSETIMER_TIMEOUT = 0x06,
	PDMSG_DR_SWAP_REQ_RECEIVED            = 0x07,
	PDMSG_PR_SWAP_REQ_RECEIVED            = 0x08,
	PDMSG_VCONN_SWAP_REQ_RECEIVED         = 0x09,
	PDMSG_VDM_ATTENTION_MSG_RECEIVED      = 0x11,
	PDMSG_REJECT_RECEIVED                 = 0x12,
	PDMSG_PRSWAP_SNKTOSRC_SENT            = 0x14,
	PDMSG_PRSWAP_SRCTOSNK_SENT            = 0x15,
	PDMSG_HARDRESET_RECEIVED              = 0x16,
	PDMSG_HARDRESET_SENT                  = 0x19,
	PDMSG_PRSWAP_SRCTOSWAP                = 0x1A,
	PDMSG_PRSWAP_SWAPTOSNK                = 0x1B,
	PDMSG_PRSWAP_SNKTOSWAP                = 0x1C,
	PDMSG_PRSWAP_SWAPTOSRC                = 0x1D,
	PDMSG_SNK_DISABLED                    = 0x20,
	PDMSG_SRC_DISABLED                    = 0x21,
	PDMSG_VDM_NAK_RECEIVED                = 0x40,
	PDMSG_VDM_BUSY_RECEIVED               = 0x41,
	PDMSG_VDM_ACK_RECEIVED                = 0x42,
	PDMSG_VDM_REQ_RECEIVED                = 0x43,
	PDMSG_VDM_DISCOVERMODEs               = 0x63,
	PDMSG_VDM_DP_STATUS                   = 0x65,
};



enum MAX77958_SYSMSG {
	SYSERROR_NONE					= 0x00,
	SYSERROR_BOOT_WDT				= 0x03,
	SYSERROR_BOOT_SWRSTREQ		    = 0x04,
	SYSMSG_BOOT_POR					= 0x05,
	SYSERROR_AFC_DONE				= 0x20,
	SYSERROR_APCMD_UNKNOWN			= 0x31,
	SYSERROR_APCMD_INPROGRESS		= 0x32,
	SYSERROR_APCMD_FAIL				= 0x33,
	SYSERROR_USBPD_RX_ERROR			= 0x50,
	SYSERROR_USBPD_TX_ERROR			= 0x51,
	SYSERROR_CCx_5V_SHORT			= 0x61,
	SYSERROR_SBUx_GND_SHORT			= 0x62,
	SYSERROR_SBUx_5V_SHORT			= 0x63,
	SYSERROR_UNKNOWN_RID_TWICE		= 0x70,
	SYSERROR_USER_PD_COLLISION		= 0x80,
	SYSERROR_MTP_WRITEFAIL			= 0xC0,
	SYSERROR_MTP_READFAIL		= 0xC1,
	SYSERROR_MTP_ERASEFAIL		= 0xC2,
	SYSERROR_MTP_CUSTMINFONOTSET	= 0xC3,
	SYSERROR_MTP_OVERACTIONBLKSIZE	= 0xC4,
	SYSERROR_FIRMWAREERROR			= 0xE0,
	SYSERROR_ACTION_UNKNOWN			= 0xE1,
	SYSERROR_ACTION_OFFSET			= 0xE2,
	SYSERROR_ACTION_UNKNOWN_CMD		= 0xE3,
	SYSERROR_EXECUTE_I2C_WRITEBURST_FAIL = 0xEA,
	SYSERROR_EXECUTE_I2C_READBURST_FAIL	= 0xEB,
	SYSERROR_EXECUTE_I2C_READ_FAIL			= 0xEC,
	SYSERROR_EXECUTE_I2C_WRITE_FAIL		= 0xED,
};


enum MAX77958_CC_PIN_STATE {
	CC_NO_CON = 0,
	CC_SNK,
	CC_SRC,
	CC_AUD_ACC,
	CC_DBG_SRC,
	CC_ERR,
	CC_DISABLED,
	CC_DBG_SNK,
	CC_RFU,
};

#define PW_0WH                      0x00
#define PW_5WH                      0x4C4B40
#define PW_8WH                      0x7A1200
#define PW_10WH                     0x989680
#define PW_15WH				        0xE4E1C0
#define PW_20WH				        0x1312D00
#define PW_24WH				        0x16E3600
#define PW_30WH				        0X1C9C380

#define OPCODE_HEADER_SIZE 1


#define TypeC_POWER_SINK_INPUT     0
#define TypeC_POWER_SOURCE_OUTPUT     1
#define TypeC_DP_SUPPORT	(0xFF01)


#define SEC_UVDM_ININIATOR		0x0
#define SEC_UVDM_RESPONDER_ACK	0x1
#define SEC_UVDM_RESPONDER_NAK	0x2
#define SEC_UVDM_RESPONDER_BUSY	0x3
#define SEC_UVDM_UNSTRUCTURED_VDM	0x0
#define STRUCTURED_VDM 			1
/*For DP Pin Assignment */
#define DP_PIN_ASSIGNMENT_NODE	0x00000000
#define DP_PIN_ASSIGNMENT_A	0x00000001	/* ( 1 << 0 ) */
#define DP_PIN_ASSIGNMENT_B	0x00000002	/* ( 1 << 1 ) */
#define DP_PIN_ASSIGNMENT_C	0x00000004	/* ( 1 << 2 ) */
#define DP_PIN_ASSIGNMENT_D	0x00000008	/* ( 1 << 3 ) */
#define DP_PIN_ASSIGNMENT_E	0x00000010	/* ( 1 << 4 ) */
#define DP_PIN_ASSIGNMENT_F	0x00000020	/* ( 1 << 5 ) */


typedef union src_pdo_object {
	uint32_t		data;
	struct {
		uint8_t		bdata[4];
	} BYTES;
	struct {
		uint32_t	reserved:30,
					type:2;
	} BITS_supply;
	struct {
		uint32_t	max_current:10,        /* 10mA units */	
				voltage:10,            /* 50mV units */
				peak_current:2,
				reserved:2,
				unchuncked_extended_messages_supported:1,
				data_role_data:1,
				usb_communications_capable:1,
				unconstrained_power:1,
				usb_suspend_supported:1,
				dual_role_power:1,
				supply:2;			/* Fixed supply : 00b */
	} BITS_pdo_fixed;
	struct {
		uint32_t	max_current:10,		/* 10mA units */
				min_voltage:10,		/* 50mV units */
				max_voltage:10,		/* 50mV units */
				supply:2;		/* Variable Supply (non-Battery) : 10b */
	} BITS_pdo_variable;
	struct {
		uint32_t	max_allowable_power:10,		/* 250mW units */
				min_voltage:10,		/* 50mV units  */
				max_voltage:10,		/* 50mV units  */
				supply:2;		/* Battery : 01b */
	} BITS_pdo_battery;
	struct {
		uint32_t	max_current:7, 	/* 50mA units */
				reserved1:1,
				min_voltage:8, 	/* 100mV units	*/
				reserved2:1,
				max_voltage:8, 	/* 100mV units	*/
				reserved3:2,
				pps_power_limited:1,
				pps_supply:2,
				supply:2;		/* APDO : 11b */
	} BITS_pdo_programmable;
} U_SEC_PDO_OBJECT;



typedef enum { /* There is another Macro definitions which are similiar to this */
	REQ			= 0,
	ACK			= 1,
	NAK 			= 2,
	BUSY 			= 3
} VDM_CMD_TYPE;


typedef enum {
	Version_1_0		= 0,
	Version_2_0		= 1,
	Reserved1		= 2,
	Reserved2		= 3
} STRUCTURED_VDM_VERSION;

typedef enum {
	Reserved		= 0,
	Discover_Identity	= 1,
	Discover_SVIDs 		= 2,
	Discover_Modes	 	= 3,
	Enter_Mode 		= 4,
	Exit_Mode		= 5,
	Attention		= 6,
	Configure		= 11
} VDM_HEADER_COMMAND;



/* For DP VDM Modes VDO Port_Capability */
typedef enum {
	num_Reserved_Capable        = 0,
	num_UFP_D_Capable           = 1,
	num_DFP_D_Capable           = 2,
	num_DFP_D_and_UFP_D_Capable = 3
} Num_DP_Port_Capability_Type;

/* For DP VDM Modes VDO Receptacle_Indication */
typedef enum {
	num_USB_TYPE_C_PLUG        = 0,
	num_USB_TYPE_C_Receptacle  = 1
} Num_DP_Receptacle_Indication_Type;


/* For DP_Status_Update Port_Connected */
typedef enum {
	num_Adaptor_Disable         = 0,
	num_Connect_DFP_D           = 1,
	num_Connect_UFP_D           = 2,
	num_Connect_DFP_D_and_UFP_D = 3
} Num_DP_Port_Connected_Type;

/* For DP_Configure Select_Configuration */
typedef enum {
	num_Cfg_for_USB             = 0,
	num_Cfg_UFP_U_as_DFP_D      = 1,
	num_Cfg_UFP_U_as_UFP_D      = 2,
	num_Cfg_Reserved            = 3
} Num_DP_Sel_Configuration_Type;

union VDM_HEADER_Type {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	VDM_command:5,
				Rsvd2_VDM_header:1,
				VDM_command_type:2,
				Object_Position:3,
				Rsvd_VDM_header:2,
				Structured_VDM_Version:2,
				VDM_Type:1,
				Standard_Vendor_ID:16;
	} BITS;
};


typedef union { /* new defined union for MD05 Op Code Command Data */
	unsigned char        DATA;
	struct {
		unsigned char     BDATA[1];
	} BYTES;
	struct {
		unsigned char	Num_Of_VDO:3,
				Cmd_Type:2,
				Reserved:3;
	} BITS;
} SEND_VDM_BYTE_DATA;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	Number_of_obj:3,
				MSG_ID:3,
				Port_Power_Role:1,
				Specification_Rev:2,
				Port_Data_Role:1,
				Reserved:1,
				MSG_Type:4;
	} BITS;
} UND_DATA_MSG_HEADER_Type;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	VDM_command:5,
				Rsvd2_VDM_header:1,
				VDM_command_type:2,
				Object_Position:3,
				Rsvd_VDM_header:2,
				Structured_VDM_Version:2,
				VDM_Type:1,
				Standard_Vendor_ID:16;
	} BITS;
} UND_DATA_MSG_VDM_HEADER_Type;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	USB_Vendor_ID:16,
				Rsvd_ID_header:10,
				Modal_Operation_Supported:1,
				Product_Type:3,
				Data_Capable_USB_Device:1,
				Data_Capable_USB_Host:1;
	} BITS;
} UND_DATA_MSG_ID_HEADER_Type;

typedef union {
	unsigned int	DATA;
	struct {
		unsigned char		BDATA[4];
	} BYTES;
	struct {
		unsigned int	Cert_TID:20,
				Rsvd_cert_VDOer:12;
	} BITS;
} UND_CERT_STAT_VDO_Type;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int    Device_Version:16,
			    Product_ID:16;
	} BITS;
} UND_PRODUCT_VDO_Type;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	USB_Superspeed_Signaling_Support:3,
				SOP_controller_present:1,
				Vbus_through_cable:1,
				Vbus_Current_Handling_Capability:2,
				SSRX2_Directionality_Support:1,
				SSRX1_Directionality_Support:1,
				SSTX2_Directionality_Support:1,
				SSTX1_Directionality_Support:1,
				Cable_Termination_Type:2,
				Cable_Latency:4,
				TypeC_to_Plug_Receptacle:1,
				TypeC_to_ABC:2,
				Rsvd_CABLE_VDO:4,
				Cable_Firmware_Version:4,
				Cable_HW_Version:4;
	} BITS;
} UND_CABLE_VDO_Type;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int    SVID_1:16,
			    SVID_0:16;
	} BITS;
} UND_VDO1_Type;

typedef union {
	unsigned int	DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	Port_Capability:2,
				Signalling_DP:4,
				Receptacle_Indication:1,
				USB_2p0_Not_Used:1,
				DFP_D_Pin_Assignments:8,
				UFP_D_Pin_Assignments:8,
				DP_MODE_VDO_Reserved:8;
	} BITS;
} UND_VDO_MODE_DP_CAPABILITY_Type;

typedef union {
	unsigned int    DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int    Port_Connected:2,
			    Power_Low:1,
			    Enabled:1,
			    Multi_Function_Preference:1,
			    USB_Configuration_Req:1,
			    Exit_DP_Mode_Req:1,
			    HPD_State:1,
			    HPD_Interrupt:1,
			    Reserved:23;
	} BITS;
} UND_VDO_DP_STATUS_UPDATES_Type;

typedef union {
	unsigned int        DATA;
	struct {
		unsigned char     BDATA[4];
	} BYTES;
	struct {
		unsigned int	SEL_Configuration:2,
				Select_DP_V1p3:1,
				Select_USB_Gen2:1,
				Select_Reserved_1:2,
				Select_Reserved_2:2,
				DFP_D_PIN_Assign_A:1,
				DFP_D_PIN_Assign_B:1,
				DFP_D_PIN_Assign_C:1,
				DFP_D_PIN_Assign_D:1,
				DFP_D_PIN_Assign_E:1,
				DFP_D_PIN_Assign_F:1,
				DFP_D_PIN_Reserved:2,
				UFP_D_PIN_Assign_A:1,
				UFP_D_PIN_Assign_B:1,
				UFP_D_PIN_Assign_C:1,
				UFP_D_PIN_Assign_D:1,
				UFP_D_PIN_Assign_E:1,
				UFP_D_PIN_Assign_F:1,
				UFP_D_PIN_Reserved:2,
				DP_MODE_Reserved:8;
	} BITS;
} UND_DP_CONFIG_UPDATE_Type;


#define VDM_RESPONDER_ACK	0x1
#define VDM_RESPONDER_NAK	0x2
#define VDM_RESPONDER_BUSY	0x3


enum VDM_MSG_IRQ_State {
	VDM_DISCOVER_ID		=	(1 << 0),
	VDM_DISCOVER_SVIDS	=	(1 << 1),
	VDM_DISCOVER_MODES	=	(1 << 2),
	VDM_ENTER_MODE		=	(1 << 3),
	VDM_DP_STATUS_UPDATE = (1 << 4),
	VDM_DP_CONFIGURE	=	(1 << 5),
	VDM_ATTENTION		=	(1 << 6),
	VDM_EXIT_MODE		=	(1 << 7),
};

typedef enum{
	OPCODE_ID_VDM_DISCOVER_IDENTITY = 0x1,
	OPCODE_ID_VDM_DISCOVER_SVIDS = 0x2,
	OPCODE_ID_VDM_DISCOVER_MODES = 0x3,
	OPCODE_ID_VDM_ENTER_MODE = 0x4,
	OPCODE_ID_VDM_EXIT_MODE = 0x5,
	OPCODE_ID_VDM_ATTENTION = 0x6,
	OPCODE_ID_VDM_SVID_DP_STATUS = 0x10,
	OPCODE_ID_VDM_SVID_DP_CONFIGURE = 0x11,
} max77705_vdm_list;


#define CCIC_NOTIFY_LOW     0x0
#define CCIC_NOTIFY_HIGH    0x1
#define CCIC_NOTIFY_IRQ     0x2

#define OPCODE_CMD_REQUEST	0x1

#define QUEUESIZE   50

#define UFP                     0x0
#define DFP                     0x1

/*
 * F/W update
 */
#define FW_CMD_READ			0x3
#define FW_CMD_READ_SIZE	6	/* cmd(1) + len(1) + data(4) */

#define FW_CMD_WRITE		0x1
#define FW_CMD_WRITE_SIZE	36	/* cmd(1) + len(1) + data(34) */

#define FW_CMD_END			0x0

#define FW_HEADER_SIZE		8
#define FW_VERIFY_DATA_SIZE 3

#define FW_VERIFY_TRY_COUNT 10

#define FW_WAIT_TIMEOUT			(1000 * 5) /* 5 sec */

struct max77958_fw_header {
	unsigned char data0;
	unsigned char data1;
	unsigned char data2;
	unsigned char data3;
	unsigned char data4; /* FW version LSB */
	unsigned char data5;
	unsigned char data6;
	unsigned char data7; /* FW version MSB */
};

#define FW_UPDATE_START 0x0
#define FW_UPDATE_WAIT_RESP_START 0x1
#define FW_UPDATE_WAIT_RESP_STOP 0x2
#define FW_UPDATE_DOING 0x3
#define FW_UPDATE_END 0x4



#define FW_UPDATE_FAIL      0xF0
#define FW_UPDATE_I2C_FAIL  0xF1
#define FW_UPDATE_TIMEOUT_FAIL  0xF2
#define FW_UPDATE_VERIFY_FAIL   0xF3
#define FW_UPDATE_CMD_FAIL  0xF4
#define FW_UPDATE_MAX_LENGTH_FAIL   0xF5


#define CUSTMINFOSIZE    6

#define DISABLED 0
#define ENABLED 1

#define SRC	0
#define SNK	1
#define DRP	2

#define MoistureDetectionDisable 0
#define MoistureDetectionEnable 1

#define MemoryUpdateRAM 0
#define MemoryUpdateMTP 1


#define PD_VID 0x1004
#define PD_PID 0x6860

#define I2C_SID0 0x67
#define I2C_SID1 0x67
#define I2C_SID2 0x67
#define I2C_SID3 0x28



typedef struct {
	UND_DATA_MSG_HEADER_Type			MSG_HEADER;
	UND_DATA_MSG_VDM_HEADER_Type		DATA_MSG_VDM_HEADER;
	UND_VDO_MODE_DP_CAPABILITY_Type		DATA_MSG_MODE_VDO_DP;
} DIS_MODE_DP_CAPA_Type;

typedef struct {
	UND_DATA_MSG_HEADER_Type			MSG_HEADER;
	UND_DATA_MSG_VDM_HEADER_Type		DATA_MSG_VDM_HEADER;
	UND_VDO_DP_STATUS_UPDATES_Type		DATA_DP_STATUS_UPDATE;
} DP_STATUS_UPDATE_Type;

typedef struct {
	UND_DATA_MSG_HEADER_Type			MSG_HEADER;
	UND_DATA_MSG_VDM_HEADER_Type		DATA_MSG_VDM_HEADER;
	UND_VDO_DP_STATUS_UPDATES_Type		DATA_MSG_DP_STATUS;
} DIS_ATTENTION_MESSAGE_DP_STATUS_Type;


struct DP_DP_DISCOVER_IDENTITY { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;

} __attribute__((aligned(1), packed));

struct DP_DP_DISCOVER_ENTER_MODE { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;

} __attribute__((aligned(1), packed));

struct DP_DP_STATUS { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;
	UND_VDO_DP_STATUS_UPDATES_Type vdo_status;
} __attribute__((aligned(1), packed));

struct DP_DP_CONFIGURE { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;
	UND_DP_CONFIG_UPDATE_Type vdo_config;
} __attribute__((aligned(1), packed));

typedef struct {
	uint32_t	VDO[7];
} VDO_MESSAGE_Type;

struct max77958_opcode {
	unsigned char opcode;
	unsigned char data[OPCODE_DATA_LENGTH];
	int read_length;
	int write_length;
};


struct max77958_apcmd_data {
	unsigned char	opcode;
	unsigned char  prev_opcode;
	unsigned char	response;
	unsigned char	read_data[OPCODE_DATA_LENGTH];
	unsigned char	write_data[OPCODE_DATA_LENGTH];
	int read_length;
	int write_length;
	unsigned char	reg;
	unsigned char	val;
	unsigned char	mask;
	unsigned char  seq;
	unsigned char	is_uvdm;
	unsigned char  is_chg_int;
	unsigned char  is_chg_notify;
};


struct max77958_apcmd_data_queue {
    unsigned char head;
    unsigned char tail;    
    unsigned char prev_tail;        
    struct max77958_apcmd_data queue[QUEUESIZE];
};

union max77958_custm_info {
	unsigned int CustmInfoArry[CUSTMINFOSIZE];

	struct {
		unsigned int debugSRC : 1;
		unsigned int debugSNK : 1;
		unsigned int AudioAcc : 1;
		unsigned int TrySNK : 1;
		unsigned int TypeCstate : 2;
		unsigned int notdef1 : 2; //8

		unsigned int VID : 16;
		unsigned int PID_H : 8;


		unsigned int PID_L : 8;
		unsigned int srcPDOtype : 2;
		unsigned int srcPDOpeakCurrent : 2;
		unsigned int notdef2 : 4; //8

		unsigned int srcPDOVoltage : 16;
        
		unsigned int srcPDOMaxCurrent : 16;
		unsigned int srcPDOOverIDebounce : 16;
        
		unsigned int srcPDOOverIThreshold : 16;
		unsigned int snkPDOtype : 2;
		unsigned int notdef3 : 6;
		unsigned int snkPDOVoltage_H : 8;
        
		unsigned int snkPDOVoltage_L : 8;
		unsigned int snkPDOOperatingI : 16;
		unsigned int i2cSID1: 8;
        
		unsigned int i2cSID2: 8;
		unsigned int i2cSID3: 8;
		unsigned int i2cSID4: 8;
		unsigned short spare: 8;        
	} custm_bits;
};


struct typec_pd_channel_state {

	/* interrupt information */
	unsigned char reg_uic_int;
	unsigned char reg_cc_int;
	unsigned char reg_pd_int;
	unsigned char reg_vdm_int;

	/* register information */
	unsigned char usbc_status1;
	unsigned char usbc_status2;
	unsigned char bc_status;
	unsigned char cc_status0;
	unsigned char cc_status1;
	unsigned char pd_status0;
	unsigned char pd_status1;


	/* F/W state */
	unsigned char HW_Revision;
	unsigned char FW_Revision;
	unsigned char FW_Minor_Revision;
	unsigned char plug_attach_done;


    unsigned char prev_ccstat;
	//CC_NO_CON / CC_SNK.
    unsigned char cur_ccstat;

	//CCISTAT_500MA / CCISTAT_1500MA
    unsigned char ccistat;

	//SDP / DCP / CDP
    unsigned char chgtype;

	//NIKON
    unsigned char prchgtype;

	//SINK POWER LIST
	int sink_status_power_list[PDO_LIST_NUM];

	//CURRENT PDO NUM
	int current_pdo_num;

	//VSAFE5V or VSAFE0V
	int vsafe5V;
	int vbus_on;    
    int vadc;

	//MAX POWER
	unsigned int max_power;
	//MAX CURRENT    
	unsigned char max_current;
	//MAX VOLTAGE        
	unsigned char max_voltage;

	unsigned char current_dr;
	unsigned char previous_dr;


    unsigned char boot_complete;

	unsigned int Vendor_ID;
	unsigned int Product_ID;
	unsigned int Device_Version;
	unsigned int SVID_0;
	unsigned int SVID_1;
	unsigned int SVID_DP;
	unsigned int dp_is_connect;
	unsigned int dp_hs_connect;
	unsigned int dp_selected_pin;
	unsigned char pin_assignment;
	unsigned int is_sent_pin_configuration;    

	unsigned char Device_Revision;
	int fw_update_state;
    unsigned char last_alternate;
    unsigned char received_discover_mode;
    unsigned char selected_pdo_num;    
};

struct typec_pd_channel_state max77958_data;

#endif /* PRJ_DEVICE_MAX77958_H_ */
