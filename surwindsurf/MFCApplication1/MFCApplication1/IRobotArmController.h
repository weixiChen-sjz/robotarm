#pragma once

// »úÐµ±Û×´Ì¬
enum RobotArmState
{
    STATE_DISCONNECTED = 0,
    STATE_CONNECTED,
    STATE_INITIALIZED,
    STATE_MOVING,
    STATE_ERROR
};

// »úÐµ±ÛÎ»ÖÃ
struct RobotArmPosition
{
    double X;
    double Y;
    double Z;
    double Theta;

    RobotArmPosition() : X(0), Y(0), Z(0), Theta(0) {}
    RobotArmPosition(double x, double y, double z, double theta = 0) : X(x), Y(y), Z(z), Theta(theta) {}
};

class IRobotArmController
{
public:
    virtual ~IRobotArmController() = default;

    virtual BOOL Connect(const CString& strIP, int nPort) = 0;
    virtual void Disconnect() = 0;
    virtual BOOL Initialize() = 0;
    virtual BOOL GoHome() = 0;

    virtual BOOL MoveTo(const RobotArmPosition& pos, int nSpeed = 50) = 0;
    virtual BOOL Stop() = 0;
    virtual BOOL EmergencyStop() = 0;

    virtual RobotArmPosition GetCurrentPosition() const = 0;
    virtual RobotArmState GetState() const = 0;
    virtual BOOL IsConnected() const = 0;

    virtual void SetSpeed(int nSpeed) = 0;
    virtual int GetSpeed() const = 0;

    virtual BOOL RotateJ7(double thetaDeg, int nSpeed = 50) = 0;
};

