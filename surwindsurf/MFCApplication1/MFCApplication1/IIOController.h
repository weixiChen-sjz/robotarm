#pragma once

#include <Windows.h>

class IIOController
{
public:
	virtual ~IIOController() = default;
	virtual BOOL Initialize() = 0;
	virtual void ProcessGPIO() = 0;
	virtual void Shutdown() = 0;
	virtual BOOL IsOpen() const = 0;
	virtual BOOL IsInitialized() const { return IsOpen(); }
	virtual BOOL IsHardwareConnected() const { return IsOpen(); }
	virtual BYTE GetGPIOLevels() = 0;
	virtual BOOL OpenInfusionSwitch() = 0;
	virtual BOOL CloseInfusionSwitch() = 0;
	virtual BOOL OpenDisinfectSwitch() = 0;
	virtual BOOL CloseDisinfectSwitch() = 0;
	virtual BOOL OpenLaserSwitch() = 0;
	virtual BOOL CloseLaserSwitch() = 0;
	virtual BOOL GetInfusionSensorState() = 0;
	virtual BOOL GetDisinfectSensorState() = 0;
	virtual BOOL GetLaserSensorState() = 0;
	virtual BOOL GetInfusionSwitchState() const = 0;
	virtual BOOL GetDisinfectSwitchState() const = 0;
	virtual BOOL GetLaserSwitchState() const = 0;
	virtual CString GetHardwareStateText() const = 0;
};
