#include "pch.h"
#include "SerialPort.h"

CSerialPort::CSerialPort()
    : m_hComm(INVALID_HANDLE_VALUE)
{
    ZeroMemory(&m_ovRead, sizeof(OVERLAPPED));
    ZeroMemory(&m_ovWrite, sizeof(OVERLAPPED));
}

CSerialPort::~CSerialPort()
{
    Close();
}

BOOL CSerialPort::Open(int nPort, int nBaudRate, int nDataBits, int nStopBits, int nParity)
{
    // 如果已经打开，先关闭
    if (IsOpen())
    {
        Close();
    }

    // 构造串口名称
    CString strPort;
    strPort.Format(_T("\\\\.\\COM%d"), nPort);

    // 打开串口
    m_hComm = CreateFile(strPort,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL);

    if (m_hComm == INVALID_HANDLE_VALUE)
    {
        return FALSE;
    }

    // 初始化重叠结构
    if (!InitOverlapped())
    {
        Close();
        return FALSE;
    }

    // 设置缓冲区大小
    SetupComm(m_hComm, 4096, 4096);

    // 清空缓冲区
    PurgeComm(m_hComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

    // 设置超时
    COMMTIMEOUTS TimeOuts;
    TimeOuts.ReadIntervalTimeout = MAXDWORD;
    TimeOuts.ReadTotalTimeoutMultiplier = 0;
    TimeOuts.ReadTotalTimeoutConstant = 0;
    TimeOuts.WriteTotalTimeoutMultiplier = 50;
    TimeOuts.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(m_hComm, &TimeOuts);

    // 配置串口参数
    DCB dcb;
    GetCommState(m_hComm, &dcb);
    dcb.BaudRate = nBaudRate;
    dcb.ByteSize = nDataBits;
    dcb.Parity = nParity;
    dcb.StopBits = nStopBits;
    dcb.fBinary = TRUE;
    dcb.fParity = (nParity != NOPARITY);

    if (!SetCommState(m_hComm, &dcb))
    {
        Close();
        return FALSE;
    }

    return TRUE;
}

void CSerialPort::Close()
{
    if (IsOpen())
    {
        CleanupOverlapped();
        CloseHandle(m_hComm);
        m_hComm = INVALID_HANDLE_VALUE;
    }
}

BOOL CSerialPort::SendData(const char* pData, DWORD dwSize)
{
    if (!IsOpen() || pData == NULL || dwSize == 0)
    {
        return FALSE;
    }

    DWORD dwBytesWritten = 0;
    DWORD dwError = 0;
    COMSTAT comStat;

    ClearCommError(m_hComm, &dwError, &comStat);

    if (!WriteFile(m_hComm, pData, dwSize, &dwBytesWritten, &m_ovWrite))
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            // 等待写操作完成
            if (!GetOverlappedResult(m_hComm, &m_ovWrite, &dwBytesWritten, TRUE))
            {
                return FALSE;
            }
        }
        else
        {
            return FALSE;
        }
    }

    return (dwBytesWritten == dwSize);
}

BOOL CSerialPort::SendData(const char* pData, DWORD dwSize, BOOL bAppendTerminator)
{
    if (!IsOpen() || pData == NULL || dwSize == 0)
    {
        return FALSE;
    }

    if (!bAppendTerminator)
    {
        // 不添加终止符，直接调用原始发送函数
        return SendData(pData, dwSize);
    }

    // 创建带终止符的缓冲区
    // 终止符为: 0x0D 0x0A 0x0D 0x0A (CR LF CR LF)
    const BYTE terminator[] = { 0x0D, 0x0A, 0x0D, 0x0A };
    DWORD dwTotalSize = dwSize + sizeof(terminator);
    char* pBuffer = new char[dwTotalSize];

    if (pBuffer == NULL)
    {
        return FALSE;
    }

    // 复制原始数据
    memcpy(pBuffer, pData, dwSize);
    // 添加终止符
    memcpy(pBuffer + dwSize, terminator, sizeof(terminator));

    // 发送完整数据
    BOOL bResult = SendData(pBuffer, dwTotalSize);

    // 释放缓冲区
    delete[] pBuffer;

    return bResult;
}

BOOL CSerialPort::ReceiveData(char* pBuffer, DWORD dwSize, DWORD& dwBytesRead)
{
    if (!IsOpen() || pBuffer == NULL || dwSize == 0)
    {
        dwBytesRead = 0;
        return FALSE;
    }

    DWORD dwError = 0;
    COMSTAT comStat;
    ClearCommError(m_hComm, &dwError, &comStat);

    if (comStat.cbInQue == 0)
    {
        dwBytesRead = 0;
        return TRUE;
    }

    DWORD dwBytesToRead = min(dwSize, comStat.cbInQue);
    dwBytesRead = 0;

    if (!ReadFile(m_hComm, pBuffer, dwBytesToRead, &dwBytesRead, &m_ovRead))
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            if (!GetOverlappedResult(m_hComm, &m_ovRead, &dwBytesRead, TRUE))
            {
                return FALSE;
            }
        }
        else
        {
            return FALSE;
        }
    }

    return TRUE;
}

void CSerialPort::ClearBuffer()
{
    if (IsOpen())
    {
        PurgeComm(m_hComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
    }
}

BOOL CSerialPort::InitOverlapped()
{
    m_ovRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (m_ovRead.hEvent == NULL)
    {
        return FALSE;
    }

    m_ovWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (m_ovWrite.hEvent == NULL)
    {
        CloseHandle(m_ovRead.hEvent);
        m_ovRead.hEvent = NULL;
        return FALSE;
    }

    return TRUE;
}

void CSerialPort::CleanupOverlapped()
{
    if (m_ovRead.hEvent != NULL)
    {
        CloseHandle(m_ovRead.hEvent);
        m_ovRead.hEvent = NULL;
    }

    if (m_ovWrite.hEvent != NULL)
    {
        CloseHandle(m_ovWrite.hEvent);
        m_ovWrite.hEvent = NULL;
    }
}

// 解析IEEE 754单精度浮点数（4字节）
float CSerialPort::ParseIEEE754Float(const BYTE* pData)
{
    if (pData == NULL)
    {
        return 0.0f;
    }
    
    // IEEE 754单精度浮点数：大端序（Big-Endian）
    // 将4个字节组合成32位整数
    union
    {
        BYTE bytes[4];
        float value;
    } data;
    
    // 按大端序排列
    data.bytes[3] = pData[0];  // 最高字节
    data.bytes[2] = pData[1];
    data.bytes[1] = pData[2];
    data.bytes[0] = pData[3];  // 最低字节
    
    return data.value;
}

// 从激光测距仪返回数据中提取距离值
// 数据格式示例: 01 04 04 C1 6E E0 69 2E 4B
// 其中 C1 6E E0 69 是IEEE 754格式的浮点数
//
// 重要说明：
// - 激光测距仪返回的是相对于零点的偏移值
// - 测量范围：120-280mm
// - 200mm是零点
// - 真实距离 = 200mm - 激光返回值
// - 例如：返回50.00mm → 真实距离 = 200 - 50 = 150mm
BOOL CSerialPort::ParseLaserDistance(const char* pData, DWORD dwSize, float& fDistance)
{
    if (pData == NULL || dwSize < 9)
    {
        return FALSE;
    }
    
    BYTE* pBytes = (BYTE*)pData;
    
    // 验证数据格式
    // 第1字节: 设备地址 (01)
    // 第2字节: 功能码 (04)
    // 第3字节: 数据长度 (04)
    if (pBytes[1] != 0x04 || pBytes[2] != 0x04)
    {
        return FALSE;
    }
    
    // 获取4字节浮点数数据 (字节3-6)
    // C1 6E E0 69
    BYTE floatData[4];
    floatData[0] = pBytes[3];  // C1
    floatData[1] = pBytes[4];  // 6E
    floatData[2] = pBytes[5];  // E0
    floatData[3] = pBytes[6];  // 69
    
    // 解析为浮点数（这是激光返回的原始偏移值）
    float fRawValue = ParseIEEE754Float(floatData);
    
    // ========== 距离换算公式 ==========
    // 测量范围: 120-280mm
    // 零点位置: 200mm
    // 真实距离 = 200mm - 激光返回值
    // 
    // 示例：
    // 激光返回 50.00mm → 真实距离 = 200 - 50 = 150mm
    // 激光返回 -30.00mm → 真实距离 = 200 - (-30) = 230mm
    // 激光返回 80.00mm → 真实距离 = 200 - 80 = 120mm
    fDistance = 200.0f - fRawValue;
    
    return TRUE;
}
