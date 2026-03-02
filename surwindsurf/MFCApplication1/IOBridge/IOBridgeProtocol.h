#pragma once

#include <stdint.h>

namespace IOBridge
{
	static constexpr uint32_t kMagic = 0x4F494F42; // 'BIOO'
	static constexpr uint16_t kVersion = 1;
	static constexpr wchar_t kPipeName[] = L"\\\\.\\pipe\\MFCApplication1.IOBridge";

	enum class Command : uint32_t
	{
		Ping = 1,
		Initialize = 2,
		Shutdown = 3,
		ProcessGPIO = 4,
		OpenInfusionSwitch = 10,
		CloseInfusionSwitch = 11,
		OpenDisinfectSwitch = 12,
		CloseDisinfectSwitch = 13,
		OpenLaserSwitch = 14,
		CloseLaserSwitch = 15,
		GetInfusionSensorState = 20,
		GetDisinfectSensorState = 21,
		GetLaserSensorState = 22,
		GetGPIOLevels = 30,
		GetSwitchStates = 31,
	};

	#pragma pack(push, 1)
	struct Request
	{
		uint32_t magic;
		uint16_t version;
		uint16_t reserved;
		uint32_t command; // Command
		uint32_t payloadSize;
		uint8_t payload[64];
	};

	struct Response
	{
		uint32_t magic;
		uint16_t version;
		uint16_t reserved;
		int32_t status; // 0 ok, negative error
		uint32_t payloadSize;
		uint8_t payload[128];
	};

	struct SwitchStates
	{
		uint8_t infusionOpen;
		uint8_t disinfectOpen;
		uint8_t laserOpen;
		uint8_t reserved;
	};
	#pragma pack(pop)
}
