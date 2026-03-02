#ifndef _EVOC_BPI_DLL_H
#define _EVOC_BPI_DLL_H
#include <windows.h>


#define BPI_EXPORT_DLL _declspec(dllexport)

#ifdef __cplusplus
extern "C"{
#endif

    typedef struct _WDT_INFO_STRUCT
    {
        unsigned char  ucTimeout;     //overtime count  ,can be 1-255
        unsigned char  ucUnit;        //time unit,can be 0(second),1(minute)
    } WDT_INFO_STRUCT,*PWDT_INFO_STRUCT;


    typedef void (_stdcall * CALLBACK_FUNC)(void * pContext);    //callback function type

//define return value
#define BPI_STATUS_SUCCESSFUL                           0x00
#define BPI_STATUS_INVALID_PARAMETER                    0x01
#define BPI_STATUS_TIMEOUT                                                          0x02
#define BPI_STATUS_FAILED                               0x03
#define BPI_STATUS_UNSUPPORT                            0xFF

#define BPI_MAX_GPIONUM                     128     // The max number of GPIO 

#define BPI_HWM_TEMP_SYSTEM                 0x00    // System device
#define BPI_HWM_TEMP_CPU1                   0x01    // CPU1 device
#define BPI_HWM_TEMP_CPU2                   0x02    // CPU2 device
#define BPI_HWM_TEMP_CPU3                   0x03    // CPU3 device
#define BPI_HWM_TEMP_CPU4                   0x04    // CPU4 device

#define BPI_HWM_FAN_SYSTEM1                 0x00    // System Fan1
#define BPI_HWM_FAN_SYSTEM2                 0x01    // System Fan2
#define BPI_HWM_FAN_SYSTEM3                 0x05    // System Fan1
#define BPI_HWM_FAN_SYSTEM4                 0x06    // System Fan2
#define BPI_HWM_FAN_CPU1                    0x02    // CPU Fan1
#define BPI_HWM_FAN_CPU2                    0x03    // CPU Fan2
#define BPI_HWM_FAN_CPU3                    0x04    // CPU Fan1
#define BPI_HWM_FAN_CPU4                    0x07    // CPU Fan2


#define BPI_HWM_VOLTAGE_CPU                 0x00    // CPU voltage
#define BPI_HWM_VOLTAGE_3_3V                0x01    // 3.3V voltage
#define BPI_HWM_VOLTAGE_5V                  0x02    // 5.0V voltage
#define BPI_HWM_VOLTAGE_12V                 0x03    // 12.0V voltage
#define BPI_HWM_VOLTAGE_BAT                 0x04    // Bat voltage
#define BPI_HWM_VOLTAGE_1_5V                0x05    // 1.5V voltage
#define BPI_HWM_VOLTAGE_1_8V                0x06    // 1.8V voltage
#define BPI_HWM_VOLTAGE_2_5V                0x07    // 2.5V voltage
#define BPI_HWM_VOLTAGE_CPU2                0x08    // CPU2 voltage
#define BPI_HWM_VOLTAGE_CPU3                0x09    // CPU3 voltage
#define BPI_HWM_VOLTAGE_CPU4                0x0A    // CPU4 voltage

#define WDT_RESET                   0x00   //WDT reset mode
#define WDT_INTERRUPT               0x01   //WDT interrupt mode
#define WDT_UNIT_SECOND             0x00   //WDT time unit(second)
#define WDT_UNIT_MINUTE             0x01   //WDT time unit(minute)

#define BPI_GPIO_OUTPUTMODE                 0x00   // GPIO out mode
#define BPI_GPIO_INPUTMODE                  0x01   // GPIO in mode
#define BPI_GPIO_LOWLEVEL                   0x00   // GPIO low level
#define BPI_GPIO_HIGHLEVEL                  0x01   // GPIO hight level


#define INTERRUPT_MSG  RegisterWindowMessage("INTERRUPT_OCCUR")   //define the message use to send when interrupt occur

        //add
#define  FIRST_BOOT_TIME  0      //第一次启动
#define  TESTFINISHTIME 1        //测试完成时间
#define  OUTFACTORYTIME 2        //出厂时间(整机测试完成时间)
#define  CUSTOMERFIRSTTIME 3     //用户第一次启用时间


typedef struct _STRUCT_TIME 
{
        unsigned int second:6;  //bit0-bit5表示秒
        unsigned int minute:6;  //bit6-bit11表示分
        unsigned int hour:5;    //bit12-bit16表示时
        unsigned int day:5;     //bit17-bit21表示日
        unsigned int month:4;   //bit22-bit25表示月
        unsigned int year:6;    //bit26-bit31表示年
}STRUCT_TIME,*PSTRUCT_TIME;


typedef struct _USER_TIME
{
        STRUCT_TIME reBootTime;
        STRUCT_TIME reShutDownTime;
        struct _USER_TIME *pNextTime;
}USER_TIME,*PUSER_TIME;




/*
Function         : BPI_Init
Description          : request resource and initial BPI
Parameters       : null
Return Values    :
                   BPI_BPI_FAILED            --- function fail
                   BPI_BPI_SUCCESSFUL        --- function success
                               BPI_BPI_STATUS_UNSUPPORT  --- not support BPI
Explain          ：must use BPI_Deinit release resource
*/
BPI_EXPORT_DLL int _stdcall BPI_Init();

/*
Function         : BPI_Deinit
Description      : release resource,BPI exit
Parameters       : null      
Return Values    :
                   BPI_BPI_FAILED       ---release resource fail
                   BPI_BPI_SUCCESSFUL   ---release resource success
*/
BPI_EXPORT_DLL int _stdcall BPI_Deinit();


/*
Function        : BPI_HWM_Get_Temperature
Description             : get the temperature of device
Parameters      : 
                          [in] ucDeviceTemperature  - device type,can be 0(system),1(CPU1),2(CPU2)
                          [out] pTemperatureValue - Pointer to an unsigned char variable that receive the temperature
                                                        if the value is 0xff mean temperature below zero, the unit is ℃
Return Values   :
                          BPI_STATUS_UNSUPPORT    ----- not support
                          BPI_STATUS_SUCCESSFUL   ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_HWM_Get_Temperature(unsigned char ucDeviceTemperature,unsigned char *pTemperatureValue);


/*
Function        : BPI_HWM_Get_FanSpeed
Description             : get the fan speed
Parameters      : 
                          [in] ucFanSpeed   -  fan type ,can be 0(system fan1),1(system fan2),2(CPU fan1),3(CPU fan2)
                  [out] pSpeedValue -  Pointer to an unsigned short variable that receive the speed of fan
Return Values   :
                          BPI_STATUS_UNSUPPORT    ----- not support
                  BPI_STATUS_SUCCESSFUL   ----- function success
*/
BPI_EXPORT_DLL int _stdcall BPI_HWM_Get_FanSpeed(unsigned char ucFanSpeed,unsigned short *pSpeedValue);


/*
Function        : BPI_HWM_Get_Voltage
Description     : get the real voltage
Parameters      : 
                              [in] ucVoltage      - voltage type ,can be 0(core voltage ),1(3.3V voltage),2(5.0V voltage),3(12.0V voltage),4(bat voltage)
                  [out] pVoltageValue - Pointer to a float variable that receive the voltage
Return Values   :
                          BPI_STATUS_UNSUPPORT         ----- not support
                  BPI_STATUS_SUCCESSFUL        ----- function success
*/
BPI_EXPORT_DLL int _stdcall BPI_HWM_Get_Voltage(unsigned char ucVoltage,float *pVoltageValue);

//////////////////////////////////////////////////////////////////////////
// WDT

/*
Function        : BPI_WDT_Open
Description         : open WDT，request resource from input parameters
Parameters      : 
                  [in] pWdtInfo  - Pointer to the PWDT_INFO_STRUCT structure  ,the structure parameters mean see PWDT_INFO_STRUCT explain
Return Values   :              
                  BPI_STATUS_SUCCESSFUL          ----- function success
                  BPI_STATUS_FAILED              ----- function fail
                                  BPI_STATUS_UNSUPPORT                   ----- don't support interrupt or reset mode.
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Open(PWDT_INFO_STRUCT pWdtInfo);

/*
Function       : BPI_WDT_Close
Description        : release resource which request in BPI_WDT_Open
Parameters     : 
                 null
Return Values  :              
                 BPI_STATUS_SUCCESSFUL          ----- function success
                 BPI_STATUS_FAILED              ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Close();

/*
Function       : BPI_WDT_Begin
Description        : set the overtime count and begin to counting ,function is the same with  BPI_WDT_Feeding
Parameters     : 
                 null
Return Values  :              
                 BPI_STATUS_SUCCESSFUL          ----- function success
                 BPI_STATUS_FAILED              ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Begin();

/*
Function       : BPI_WDT_Stop
Description        : stop to count the time
Parameters     : 
null
Return Values  :              
BPI_STATUS_SUCCESSFUL          ----- function success
BPI_STATUS_FAILED              ----- function fail
*/
BPI_EXPORT_DLL INT _stdcall BPI_WDT_Stop();

/*
Function       : BPI_WDT_Feeding
Description        : the same function as  BPI_WDT_Begin
Parameters     : 
                 无
Return Values  :              
                 BPI_STATUS_SUCCESSFUL          ----- function success
                 BPI_STATUS_FAILED              ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Feeding();

/*
Function       : BPI_WDT_Get_CurrentTime
Description        : get current overtime count
Parameters     : 
                 [out] pCurrentTime        - Pointer to an unsigned char variable that receive the current overtime count 
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Get_CurrentTime(unsigned char *pCurrentTime);


/*
Function        : BPI_WDT_Set_Callback
Description         : set the callback function
Parameters      : 
                  [in] pFunc     - Pointer to the callback function address
                              [in] pContext  -  Pointer to the callback function parameters address
Return Values   :              
                  BPI_STATUS_SUCCESSFUL          ----- function success
                  BPI_STATUS_FAILED              ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Set_Callback(CALLBACK_FUNC pFunc, void *pContext);


/*
Function       : BPI_WDT_Get_EnableIRQ
Description        : get enable irq
Parameters     : 
                 [out] pIRQArray        - Pointer to an unsigned char array that receive the enable irq
                                 [in][out] pArrayLen    - input value is the pIRQArray length,output is the enable irq array length
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_WDT_Get_EnableIRQ(unsigned char *pIRQArray,unsigned char *pArrayLen);


//////////////////////////////////////////////////////////////////////////
// GPIO
/*
Function       : BPI_GPIO_Init
Description        : Init GPIO funtion, and get valid GPIO count
Parameters     : 
                 [out] pucGpioNum   - the number of GPIO
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
                 BPI_STATUS_UNSUPPORT         ----- function fail ,no GPIO
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_Init(PUCHAR pucGpioNum);

/*
Function       : BPI_GPIO_Deinit
Description        : Deinit GPIO funtion, and reset all the pins
Parameters     : 
                 
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
                 BPI_STATUS_UNSUPPORT         ----- function fail ,no GPIO
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_Deinit();

/*
Function       : BPI_GPIO_SetIoMode
Description        : Set the GPIO input/output mode
Parameters     : 
                 [in] pbGpioMode  - Pointer to the byte array buff for Input/Output mode
                 each bit corresponding to a pin 0:output,      1:input
                 [in] ucNum       - Number of GPIO
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_SetIoMode(PBYTE pbGpioMode, UCHAR ucBuffLen);

/*
Function       : BPI_GPIO_GetIoMode
Description        : Get the GPIO input/output mode
Parameters     : 
                 [out]pbGpioMode  - Pointer to the byte array buff for Input/Output mode
                 each bit corresponding to a pin 0:output,      1:input
                 [in] ucBuffLen   - Length of the pbGpioMode Buff
                 [out]pucNum      - Number of GPIO
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_GetIoMode(PBYTE pbGpioMode, UCHAR ucBuffLen);

/*
Function       : BPI_GPIO_SetLevels
Description        : Set all GPO(output) level
Parameters     : 
                 [in] pbLevels     - Pointer to the byte array buff for GPO Levels
                 each bit corresponding to a level 0:low,       1:high
                 [in] ucValidNum   - Length of the pbGpioMode Buff
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_SetLevels(PBYTE pbLevels, UCHAR ucBuffLen);

/*
Function       : BPI_GPIO_GetLevels
Description        : Get all GPO(output) level
Parameters     : 
                 [out]pbLevels     - Pointer to the byte array buff for GPI Levels
                 each bit corresponding to a level 0:low,       1:high
                 [in] ucValidNum   - Length of the pbGpioMode Buff
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_GetLevels(PBYTE pbLevels, UCHAR ucBuffLen);


/*
Function      : BPI_GPIO_SetPinLevel
Description       : Set GPIO Pin level
Parameters    :
                [in] ucPinIndex    - Pin index which want to set
                [in] ucPinLevel    - Pin level value
Return Values      :
                BPI_STATUS_SUCCESSFUL        ----- function success
                BPI_STATUS_FAILED            ----- function fail
Explain       ：only output mode can set level
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_SetPinLevel(UCHAR ucGpioId, UCHAR ucPinLevel);

/*
Function       : BPI_GPIO_GetPinLevel
Description        : Get the Pin level
Parameters     : 
[in] ucPinIndex    - Pin index 
[out] pPinLevel    - Pointer to unsigned char that get the level,
0 is low level,1 is hight level
Return Values  :              
BPI_STATUS_SUCCESSFUL        ----- function success
BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL INT WINAPI BPI_GPIO_GetPinLevel(UCHAR ucGpioId, PUCHAR pucPinLevel);



/*
Function      : BPI_LED_SET_State
Description       : SET LED STATE
Parameters    :
                                [in] ucPinIndex    - Pin index which want to set
                                [in] ucPinLevel    - Pin level value
Return Values      :
BPI_STATUS_SUCCESSFUL        ----- function success
BPI_STATUS_FAILED            ----- function fail
Explain       ：only output mode can set level
*/
BPI_EXPORT_DLL INT WINAPI BPI_LED_Get_State(UCHAR ucPinIndex,PUCHAR pucPinLevel);


/*
Function      : BPI_LED_Get_State
Description       : get LED STATE
Parameters    :
                                [in] ucPinIndex    - Pin index which want to set
                                [out] ucPinLevel    - Pin level value
Return Values      :
BPI_STATUS_SUCCESSFUL        ----- function success
BPI_STATUS_FAILED            ----- function fail

*/

BPI_EXPORT_DLL INT WINAPI BPI_LED_Set_State(UCHAR ucPinIndex,UCHAR ucPinLevel);



/*

Function      : BPI_Get_STATUS
Description       : BPI_Get_STATUS
Parameters    :
[out] ucPinLevel    -  RETURN STATUS
Return Values      :
BPI_STATUS_SUCCESSFUL        ----- function success
BPI_STATUS_FAILED            ----- function fail

*/
//BPI_EXPORT_DLL INT WINAPI BPI_Get_STATUS(NTSTATUS *STATUS);


//////////////////////////////////////////////////////////////////////////
// Old Interface

/*
Function       : BPI_GPIO_Init_Config
Description        : set GPIO Input/Output mode and level
Parameters        : 
                 [in] ucStartPin      - the begin Pin index
                             [in] ucEndPin        - the end Pin index
                             [in] pPinMode        - Pointer to an unsigned char array,each bit in the array mean the Pin Input/Output mode，
                                                   
                             [in] pPinLevel       - Pointer to an unsigned char array,each bit in the array mean the Pin level
                                                                          Exp:  ucStartPin=1,
                                                                                ucEndPin = 8
                                                                                        pPinMode = {0x0f,...}
                                                                                        pPinLevel = {0x0f,...}
                                                                          mean the Pin1 to Pin4 is input mode,Pin5-Pin8 is output mode, the Pin1 to Pin4 
                                                                          is hight level,pin5-Pin8 is low level
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
                 BPI_STATUS_UNSUPPORT         ----- function fail ,not GPIO
*/
BPI_EXPORT_DLL int _stdcall BPI_GPIO_Init_Config(UCHAR ucStartPin,UCHAR  ucEndPin,PUCHAR pPinMode,PUCHAR pPinLevel);

/*
Function       : BPI_GPIO_Get_Config
Description        : get the GPIO config
Parameters     : 
                 [out] pGPIOPinMode   - Pointer to the unsigned char array that receive Input/Output mode
                                 [out] pGPIOPinLevel  - Pointer to the unsigned char array that receive Level
                                 [in][out] pPintArrayLen - Pointer to the unsigned char ,input value is the length of the array,
                                                           output value is the real length of the efficacious
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_GPIO_Get_Config(PUCHAR pGPIOPinMode,PUCHAR pGPIOPinLevel,PUCHAR pPintArrayLen);

/*
Function       : BPI_GPIO_Get_PinLevel
Description        : get the Pin level
Parameters     : 
                 [in] ucPinIndex    - Pin index 
                 [out] pPinLevel    - Pointer to unsigned char that get the level, 0 is low level,1 is hight level
Return Values  :              
                 BPI_STATUS_SUCCESSFUL        ----- function success
                 BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_GPIO_Get_PinLevel(unsigned char ucPinIndex,unsigned char *pPinLevel);

/*
Function      : BPI_GPIO_Set_PinLevel
Description       : set GPIO Pin level
Parameters    :
                [in] ucPinIndex    - Pin index which want to set
                [in] ucPinLevel    - Pin level value
Return Values      :
                BPI_STATUS_SUCCESSFUL        ----- function success
                BPI_STATUS_FAILED            ----- function fail
Explain       ：only output mode can set level
*/
BPI_EXPORT_DLL int _stdcall BPI_GPIO_Set_PinLevel(unsigned char ucPinIndex,unsigned char ucPinLevel);
/*
Function      : BPI_GPIO_Set_PinMode
Description       : set GPIO Pin input/Output mode
Parameters    :
                [in] ucPinindex     - GPIO Pin index which want to set
                [in] ucPinmode      - input/Output mode value 
Return Values :
                BPI_STATUS_SUCCESSFUL        ----- function success
                            BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_GPIO_Set_PinMode(unsigned char ucPinIndex,unsigned char ucPinMode);

/*
Function      : BPI_GPIO_Get_PinCount
Description       : get GPIO pin count
Parameters    :
                [out] pPinCount     - Pointer to unsigned char that get the pin count
                [in] ucPinmode      - input/Output mode value 
Return Values :
                BPI_STATUS_SUCCESSFUL        ----- function success
                            BPI_STATUS_FAILED            ----- function fail
*/
BPI_EXPORT_DLL int _stdcall BPI_GPIO_Get_PinCount(unsigned char *pPinCount);


//////////////////////////////////////////////////////////////////////////
// WinIo test
/*
BPI_EXPORT_DLL BOOL WINAPI GetPhysLong(PBYTE pbPhysAddr, PDWORD pdwPhysVal);
BPI_EXPORT_DLL BOOL WINAPI SetPhysLong(PBYTE pbPhysAddr, DWORD dwPhysVal);
BPI_EXPORT_DLL BOOL WINAPI GetPortVal(WORD wPortAddr, PDWORD pdwPortVal, BYTE bSize);
BPI_EXPORT_DLL BOOL WINAPI SetPortVal(WORD wPortAddr, DWORD dwPortVal, BYTE bSize);
*/





/*FMI 模块*/


/*
Function      : BPI_WriteSN
Description       : SN写入Flash
Parameters    :
                [in] dwType        - SN类型，包括主板序列号MAINBOARD_SN(0x01),客户序列号CUSTOMER_SN(0x02)
                                [in] pData[]       - 存储SN数据的数组
                                [in] dwSNlen       - SN长度
Return Values :
                BPI_STATUS_SUCCESSFUL        ----- function success
                BPI_STATUS_FAILED            ----- function fail
                                BPI_STATUS_UNSUPPORT         ----- function support
*/
BPI_EXPORT_DLL int _stdcall BPI_WriteSN(unsigned char dwType ,unsigned char pData[] ,unsigned char dwSNlen);

/*
Function      : BPI_ReadSN
Description       : 读取SN
Parameters    :
                [in] dwType        - SN类型，包括主板序列号MAINBOARD_SN(0x01),客户序列号CUSTOMER_SN(0x02)
                                [out] pData[]      - 用于存储SN数据的数组地址
Return Values :
                BPI_STATUS_SUCCESSFUL        ----- function success
                BPI_STATUS_FAILED            ----- function fail
                                BPI_STATUS_UNSUPPORT         ----- function support
*/
BPI_EXPORT_DLL int _stdcall BPI_ReadSN(unsigned char dwType ,unsigned char pData[]);

/*
Function      : BPI_WriteCustomSN
Description       : 写入客户SN
Parameters    :
                [in] pData[]        - SN的数组指针
                                [in] dwSNlen        - 要写入的SN长度，最大32位
Return Values :
                BPI_STATUS_SUCCESSFUL        ----- function success
                BPI_STATUS_FAILED            ----- function fail
                                BPI_STATUS_UNSUPPORT         ----- function support
*/
BPI_EXPORT_DLL int _stdcall BPI_WriteCustomSN(unsigned char pData[] ,unsigned char dwSNlen);


//获取特殊时间
BPI_EXPORT_DLL int _stdcall BPI_FMI_ReadSpecialTime(int TimeType,PSTRUCT_TIME pstSpecTime);

//读取记录的开关机记录
BPI_EXPORT_DLL int _stdcall BPI_FMI_ReadTimeRecord(PUSER_TIME *pUserTime,DWORD *dwTimeNum);

//读取记录的开关机记录
BPI_EXPORT_DLL int _stdcall BPI_FMI_FreeMemory(PUSER_TIME pUserTime);

//读取引导设备
BPI_EXPORT_DLL int _stdcall BPI_FMI_GetBootDevice(unsigned char ucBootData[]);

//设置引导设备顺序
BPI_EXPORT_DLL int _stdcall BPI_FMI_SetBootDevice(unsigned char ucBootData[]);

//获取启动次数
BPI_EXPORT_DLL unsigned short _stdcall BPI_FMI_GetBootNum();

//获取非法关机次数
BPI_EXPORT_DLL unsigned short _stdcall BPI_FMI_GetShutDownFailNum();

//获取运行总时间
BPI_EXPORT_DLL unsigned __int32 _stdcall BPI_FMI_GetRunTime();

/*
Function      : BPI_FMI_CheckeDisk
Description       : 检测是否支持eDisk
Parameters    :
                [out] bRes        - 检测结果
Return Values :
                BPI_STATUS_SUCCESSFUL        ----- function success
                BPI_STATUS_FAILED            ----- function fail
                                BPI_STATUS_UNSUPPORT         ----- function support
*/
BPI_EXPORT_DLL int _stdcall BPI_FMI_CheckeDisk(BOOL *bRes);


BPI_EXPORT_DLL int _stdcall BPI_SAVE_FMI(char * filename);

BPI_EXPORT_DLL int _stdcall BPI_Load_FMI(char * filename);









//////////////////////////////////////////////////////////////////////////
// WinIo test
/*
BPI_EXPORT_DLL BOOL WINAPI GetPhysLong(PBYTE pbPhysAddr, PDWORD pdwPhysVal);
BPI_EXPORT_DLL BOOL WINAPI SetPhysLong(PBYTE pbPhysAddr, DWORD dwPhysVal);
BPI_EXPORT_DLL BOOL WINAPI GetPortVal(WORD wPortAddr, PDWORD pdwPortVal, BYTE bSize);
BPI_EXPORT_DLL BOOL WINAPI SetPortVal(WORD wPortAddr, DWORD dwPortVal, BYTE bSize);
*/







INT IoctlDebugFun();

#ifdef __cplusplus
}
#endif

// gansai add
extern HANDLE hDriver;
extern BOOL IsBPIIoInitialized;
extern BOOL g_Is64BitOS;

#endif
