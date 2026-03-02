#pragma once
#include <array>
#include <string>
#include <memory>
#include <mutex>
#include <afxsock.h>

#include "IRobotArmController.h"

namespace rokae { class BaseRobot; }

class CRobotArmController : public IRobotArmController
{
public:
    CRobotArmController();
    ~CRobotArmController();

    // 连接机械臂（TCP/IP）
    BOOL Connect(const CString& strIP, int nPort) override;
    
    // 断开连接
    void Disconnect() override;
    
    // 初始化机械臂
    BOOL Initialize() override;
    
    // 归零
    BOOL GoHome() override;
    
    // 移动到指定位置
    BOOL MoveTo(const RobotArmPosition& pos, int nSpeed = 50) override;
    
    // 停止
    BOOL Stop() override;
    
    // 急停
    BOOL EmergencyStop() override;
    
    // 获取当前位置
    RobotArmPosition GetCurrentPosition() const override;
    
    // 获取当前状态
    RobotArmState GetState() const override { return m_state; }
    
    // 设置速度 (0-100)
    void SetSpeed(int nSpeed) override;
    
    // 获取速度
    int GetSpeed() const override { return m_nSpeed; }
    
    // 检查是否连接
    BOOL IsConnected() const override;

    // 仅旋转第7轴(J7)，单位：度
    BOOL RotateJ7(double thetaDeg, int nSpeed = 50) override;

    BOOL JogJoint(int jointIndex, double deltaDeg, int nSpeed = 50);

    BOOL GetJointAnglesDeg(std::array<double, 7>& outDeg) const;

private:
    CSocket m_socket;                 // TCP/IP Socket对象
    RobotArmState m_state;            // 当前状态
    mutable RobotArmPosition m_currentPos;    // 当前位置
    int m_nSpeed;                     // 当前速度
    CString m_strServerIP;            // 服务器IP地址
    int m_nServerPort;                // 服务器端口

    std::unique_ptr<rokae::BaseRobot> m_sdkRobot;
    mutable std::mutex m_sdkMutex;
    std::string m_sdkRemoteIp;
    std::string m_sdkLocalIp;
    
    // 发送命令并等待响应
    BOOL SendCommand(const CString& strCmd, CString& strResponse, DWORD dwTimeout = 1000);
    
    // 解析位置响应
    BOOL ParsePosition(const CString& strResponse, RobotArmPosition& pos);
};
