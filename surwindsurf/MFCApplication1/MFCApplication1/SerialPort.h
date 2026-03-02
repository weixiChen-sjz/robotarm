#pragma once
#include <Windows.h>
#include <string>

class CSerialPort
{
public:
    CSerialPort();
    ~CSerialPort();

    // 打开串口
    BOOL Open(int nPort, int nBaudRate = 9600, int nDataBits = 8, 
              int nStopBits = 1, int nParity = 0);
    
    // 关闭串口
    void Close();
    
    // 发送数据
    BOOL SendData(const char* pData, DWORD dwSize);
    
    // 发送数据（支持自动添加终止符）
    BOOL SendData(const char* pData, DWORD dwSize, BOOL bAppendTerminator);
    
    // 接收数据
    BOOL ReceiveData(char* pBuffer, DWORD dwSize, DWORD& dwBytesRead);
    
    // 清空缓冲区
    void ClearBuffer();
    
    // 检查串口是否打开
    BOOL IsOpen() const { return m_hComm != INVALID_HANDLE_VALUE; }
    
    // 获取串口句柄
    HANDLE GetHandle() const { return m_hComm; }

    // 解析IEEE 754单精度浮点数（4字节）
    static float ParseIEEE754Float(const BYTE* pData);
    
    // 从激光测距仪返回数据中提取距离值
    static BOOL ParseLaserDistance(const char* pData, DWORD dwSize, float& fDistance);

private:
    HANDLE m_hComm;           // 串口句柄
    OVERLAPPED m_ovRead;      // 读重叠结构
    OVERLAPPED m_ovWrite;     // 写重叠结构
    
    // 初始化重叠结构
    BOOL InitOverlapped();
    
    // 清理重叠结构
    void CleanupOverlapped();
};
