#ifndef CONTROLCAN_H
#define CONTROLCAN_H

////文件版本：v2.00 20150920
//#include <cvidef.h>	//使用CVI平台开发，请使用该语句。
#include "windows.h"

//接口卡类型定义

#define VCI_USBCAN1		3
#define VCI_USBCAN2		4
#define VCI_USBCAN2A		4

#define VCI_USBCAN_E_U 		20
#define VCI_USBCAN_2E_U 	21

//CAN错误码
#define	ERR_CAN_OVERFLOW			0x0001	//CAN控制器内部FIFO溢出
#define	ERR_CAN_ERRALARM			0x0002	//CAN控制器错误报警
#define	ERR_CAN_PASSIVE				0x0004	//CAN控制器消极错误
#define	ERR_CAN_LOSE				0x0008	//CAN控制器仲裁丢失
#define	ERR_CAN_BUSERR				0x0010	//CAN控制器总线错误
#define	ERR_CAN_REG_FULL			0x0020	//CAN接收寄存器满
#define	ERR_CAN_REG_OVER			0x0040	//CAN接收寄存器溢出
#define	ERR_CAN_ZHUDONG	    		0x0080	//CAN控制器主动错误

//通用错误码
#define	ERR_DEVICEOPENED			0x0100	//设备已经打开
#define	ERR_DEVICEOPEN				0x0200	//打开设备错误
#define	ERR_DEVICENOTOPEN			0x0400	//设备没有打开
#define	ERR_BUFFEROVERFLOW			0x0800	//缓冲区溢出
#define	ERR_DEVICENOTEXIST			0x1000	//此设备不存在
#define	ERR_LOADKERNELDLL			0x2000	//装载动态库失败
#define ERR_CMDFAILED				0x4000	//执行命令失败错误码
#define	ERR_BUFFERCREATE			0x8000	//内存不足

//函数调用返回状态值
#define	STATUS_OK					1
#define STATUS_ERR					0
	
/*------------------------------------------------兼容ZLG的函数及数据类型------------------------------------------------*/
//1.ZLGCAN系列接口卡信息的数据类型。
typedef  struct  _VCI_BOARD_INFO{
		USHORT	hw_Version;
		USHORT	fw_Version;
		USHORT	dr_Version;
		USHORT	in_Version;
		USHORT	irq_Num;
		BYTE	can_Num;
		CHAR	str_Serial_Num[20];
		CHAR	str_hw_Type[40];
		USHORT	Reserved[4];
} VCI_BOARD_INFO,*PVCI_BOARD_INFO; 

//2.定义CAN信息帧的数据类型。
typedef  struct  _VCI_CAN_OBJ{
	UINT	ID;
	UINT	TimeStamp;
	BYTE	TimeFlag;
	BYTE	SendType;
	BYTE	RemoteFlag;//是否是远程帧
	BYTE	ExternFlag;//是否是扩展帧
	BYTE	DataLen;
	BYTE	Data[8];
	BYTE	Reserved[3];
}VCI_CAN_OBJ,*PVCI_CAN_OBJ;

//3.定义初始化CAN的数据类型
typedef struct _VCI_INIT_CONFIG{
	DWORD	AccCode;
	DWORD	AccMask;
	DWORD	Reserved;
	UCHAR	Filter;
	UCHAR	Timing0;	
	UCHAR	Timing1;	
	UCHAR	Mode;
}VCI_INIT_CONFIG,*PVCI_INIT_CONFIG;

//4.定义错误信息的数据类型。
typedef struct _VCI_ERR_INFO{
        UINT	ErrCode;
        BYTE	Passive_ErrData[3];
        BYTE	ArLost_ErrData;
} VCI_ERR_INFO,*PVCI_ERR_INFO;

///////// new add struct for filter /////////
typedef struct _VCI_FILTER_RECORD{
	DWORD ExtFrame;	//是否为扩展帧
	DWORD Start;
	DWORD End;
}VCI_FILTER_RECORD,*PVCI_FILTER_RECORD;
 
#define EXTERNC		extern "C"

EXTERNC DWORD __stdcall VCI_OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
EXTERNC DWORD __stdcall VCI_CloseDevice(DWORD DeviceType,DWORD DeviceInd);
EXTERNC DWORD __stdcall VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

EXTERNC DWORD __stdcall VCI_ReadBoardInfo(DWORD DeviceType,DWORD DeviceInd,PVCI_BOARD_INFO pInfo);
EXTERNC DWORD __stdcall VCI_ReadErrInfo(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_ERR_INFO pErrInfo);

EXTERNC DWORD __stdcall VCI_SetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);

EXTERNC ULONG __stdcall VCI_GetReceiveNum(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
EXTERNC DWORD __stdcall VCI_ClearBuffer(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

EXTERNC DWORD __stdcall VCI_StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
EXTERNC DWORD __stdcall VCI_ResetCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

EXTERNC ULONG __stdcall VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,ULONG Len);
EXTERNC ULONG __stdcall VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,ULONG Len,INT WaitTime);


/*------------------------------------------------其他补充函数及数据结构描述------------------------------------------------*/

//USB-CAN总线适配器板卡信息的数据类型1，该类型为VCI_FindUsbDevice函数的返回参数。
typedef  struct  _VCI_BOARD_INFO1{
	USHORT	hw_Version;
	USHORT	fw_Version;
	USHORT	dr_Version;
	USHORT	in_Version;
	USHORT	irq_Num;
	BYTE	can_Num;
	BYTE	Reserved;
	CHAR	str_Serial_Num[8];
	CHAR	str_hw_Type[16];
	CHAR	str_Usb_Serial[4][4];
} VCI_BOARD_INFO1,*PVCI_BOARD_INFO1;

//USB-CAN总线适配器板卡信息的数据类型2，该类型为VCI_FindUsbDevice函数的返回参数。为扩展更多的设备
typedef  struct  _VCI_BOARD_INFO2{
	USHORT	hw_Version;
	USHORT	fw_Version;
	USHORT	dr_Version;
	USHORT	in_Version;
	USHORT	irq_Num;
	BYTE	can_Num;
	BYTE	Reserved;
	CHAR	str_Serial_Num[8];
	CHAR	str_hw_Type[16];
	CHAR	str_Usb_Serial[10][4];
} VCI_BOARD_INFO2,*PVCI_BOARD_INFO2;


#define EXTERNC		extern "C"

EXTERNC DWORD __stdcall VCI_GetReference2(DWORD DevType,DWORD DevIndex,DWORD CANIndex,DWORD Reserved,BYTE *pData);
EXTERNC DWORD __stdcall VCI_SetReference2(DWORD DevType,DWORD DevIndex,DWORD CANIndex,DWORD RefType,BYTE *pData);


EXTERNC DWORD __stdcall VCI_ConnectDevice(DWORD DevType,DWORD DevIndex);
EXTERNC DWORD __stdcall VCI_UsbDeviceReset(DWORD DevType,DWORD DevIndex,DWORD Reserved);
EXTERNC DWORD __stdcall VCI_FindUsbDevice(PVCI_BOARD_INFO1 pInfo);
EXTERNC DWORD __stdcall VCI_FindUsbDevice2(PVCI_BOARD_INFO2 pInfo);



#endif
