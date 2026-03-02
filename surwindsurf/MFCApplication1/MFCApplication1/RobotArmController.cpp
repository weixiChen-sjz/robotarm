#include "pch.h"
#include "RobotArmController.h"

#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <Shlwapi.h>
#include <iphlpapi.h>

#include "rokae/robot.h"
#include "LogManager.h"

namespace {
    constexpr double kPi = 3.1415926535897932384626433832795;
    constexpr double kHalfPi = 1.5707963267948966192313216916398;
    constexpr int kSdkTimeoutEcValue = 263;

    void LogSdkEc(const wchar_t* action, const ::error_code& ec)
    {
        if (!ec)
        {
            return;
        }

        CStringA msgA(ec.message().c_str());
        CString msgW(msgA);

        CString out;
        out.Format(_T("%s 失败: (%d) %s"), action, static_cast<int>(ec.value()), msgW.GetString());
        CLogManager::GetInstance().AddLog(_T("机械臂"), LOG_ERROR, out);
    }

    void LogSdkText(const CString& text, LogLevel level = LOG_INFO)
    {
        CLogManager::GetInstance().AddLog(_T("机械臂"), level, text);
    }

    bool IsSdkTimeoutEc(const ::error_code& ec)
    {
        return ec && ec.value() == kSdkTimeoutEcValue;
    }

    CString PowerStateToString(rokae::PowerState ps)
    {
        switch (ps)
        {
        case rokae::PowerState::on: return _T("on");
        case rokae::PowerState::off: return _T("off");
        case rokae::PowerState::estop: return _T("estop");
        case rokae::PowerState::gstop: return _T("gstop");
        default: return _T("unknown");
        }
    }

    CString OperateModeToString(rokae::OperateMode mode)
    {
        switch (mode)
        {
        case rokae::OperateMode::manual: return _T("manual");
        case rokae::OperateMode::automatic: return _T("automatic");
        default: return _T("unknown");
        }
    }

    CString GetConfigIniPath()
    {
        TCHAR szPath[MAX_PATH];
        GetModuleFileName(NULL, szPath, MAX_PATH);
        PathRemoveFileSpec(szPath);
        PathAppend(szPath, _T("config.ini"));
        return CString(szPath);
    }

    std::string ReadIniStringA(const wchar_t* section, const wchar_t* key)
    {
        CString iniPath = GetConfigIniPath();
        TCHAR buf[256] = { 0 };
        GetPrivateProfileString(section, key, _T(""), buf, 256, iniPath);
        CString v(buf);
        v.Trim();
        CStringA a(v);
        return std::string(a.GetString());
    }

    bool EnsureWsaStarted()
    {
        static std::once_flag once;
        static bool ok = false;
        std::call_once(once, []() {
            WSADATA wsaData{};
            ok = (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0);
        });
        return ok;
    }

    bool ParseIpv4(const std::string& ip, IN_ADDR& out)
    {
        if (!EnsureWsaStarted())
        {
            return false;
        }
        return inet_pton(AF_INET, ip.c_str(), &out) == 1;
    }

    std::string SockaddrToIpv4String(const sockaddr* sa)
    {
        if (!sa || sa->sa_family != AF_INET)
        {
            return std::string();
        }

        const sockaddr_in* sin = reinterpret_cast<const sockaddr_in*>(sa);
        char buf[INET_ADDRSTRLEN] = { 0 };
        if (!EnsureWsaStarted())
        {
            return std::string();
        }
        if (inet_ntop(AF_INET, const_cast<IN_ADDR*>(&sin->sin_addr), buf, INET_ADDRSTRLEN) == nullptr)
        {
            return std::string();
        }
        return std::string(buf);
    }

    bool IsLoopbackOrApipa(const IN_ADDR& a)
    {
        const unsigned long host = ntohl(a.S_un.S_addr);
        const unsigned long first = (host >> 24) & 0xFF;
        const unsigned long second = (host >> 16) & 0xFF;
        if (first == 127)
        {
            return true;
        }
        if (first == 169 && second == 254)
        {
            return true;
        }
        return false;
    }

    std::string PickLocalIpForRemote(const std::string& remoteIp)
    {
        IN_ADDR remoteAddr{};
        if (!ParseIpv4(remoteIp, remoteAddr))
        {
            return std::string();
        }
        const unsigned long remoteHost = ntohl(remoteAddr.S_un.S_addr);

        ULONG size = 0;
        ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
        DWORD ret = GetAdaptersAddresses(AF_INET, flags, nullptr, nullptr, &size);
        if (ret != ERROR_BUFFER_OVERFLOW || size == 0)
        {
            return std::string();
        }

        std::vector<unsigned char> buf(size);
        IP_ADAPTER_ADDRESSES* addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buf.data());
        ret = GetAdaptersAddresses(AF_INET, flags, nullptr, addrs, &size);
        if (ret != NO_ERROR)
        {
            return std::string();
        }

        std::string fallback;
        for (IP_ADAPTER_ADDRESSES* aa = addrs; aa != nullptr; aa = aa->Next)
        {
            for (IP_ADAPTER_UNICAST_ADDRESS* ua = aa->FirstUnicastAddress; ua != nullptr; ua = ua->Next)
            {
                if (!ua->Address.lpSockaddr || ua->Address.lpSockaddr->sa_family != AF_INET)
                {
                    continue;
                }

                const sockaddr_in* sin = reinterpret_cast<const sockaddr_in*>(ua->Address.lpSockaddr);
                IN_ADDR localAddr = sin->sin_addr;
                if (IsLoopbackOrApipa(localAddr))
                {
                    continue;
                }

                std::string localIp = SockaddrToIpv4String(ua->Address.lpSockaddr);
                if (fallback.empty())
                {
                    fallback = localIp;
                }

                ULONG prefix = ua->OnLinkPrefixLength;
                if (prefix > 32)
                {
                    prefix = 32;
                }
                const unsigned long mask = (prefix == 0) ? 0UL : (0xFFFFFFFFUL << (32 - prefix));
                const unsigned long localHost = ntohl(localAddr.S_un.S_addr);
                if ((remoteHost & mask) == (localHost & mask))
                {
                    return localIp;
                }
            }
        }

        return fallback;
    }

    bool ParseDoubles(const CString& text, std::vector<double>& out)
    {
        out.clear();
        CStringA a(text);
        std::string s = a.GetString();
        for (char& ch : s)
        {
            if (ch == ',' || ch == ';' || ch == '\t')
            {
                ch = ' ';
            }
        }
        std::stringstream ss(s);
        double v = 0.0;
        while (ss >> v)
        {
            out.push_back(v);
        }
        return !out.empty();
    }

    void UpdateCachedPositionLocked(rokae::BaseRobot* robot, RobotArmPosition& ioPos)
    {
        if (!robot)
        {
            return;
        }

        ::error_code ec;
        try
        {
            auto p = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            if (!ec)
            {
                ioPos.X = p[0] * 1000.0;
                ioPos.Y = p[1] * 1000.0;
                ioPos.Z = p[2] * 1000.0;
                // Theta 使用第7轴(J7)角度（单位：度）
                auto joints = robot->jointPos(ec);
                if (!ec && joints.size() >= 7)
                {
                    ioPos.Theta = joints[6] * 180.0 / kPi;
                }
            }
        }
        catch (const std::exception&)
        {
            // keep last cached value
        }
    }
}

CRobotArmController::CRobotArmController()
    : m_state(STATE_DISCONNECTED)
    , m_nSpeed(50)
    , m_nServerPort(0)
{
}

CRobotArmController::~CRobotArmController()
{
    Disconnect();
}

BOOL CRobotArmController::Connect(const CString& strIP, int nPort)
{
    (void)nPort;

    Disconnect();

    CStringA ipA(strIP);
    m_sdkRemoteIp = std::string(ipA.GetString());
    m_sdkLocalIp.clear();

    {
        HMODULE hSdk = GetModuleHandle(_T("xCoreSDK.dll"));
        if (hSdk)
        {
            TCHAR sdkPath[MAX_PATH] = { 0 };
            GetModuleFileName(hSdk, sdkPath, MAX_PATH);
            CString m;
            m.Format(_T("xCoreSDK.dll: %s"), sdkPath);
            LogSdkText(m, LOG_INFO);
        }
    }

    std::string localIpFromIni = ReadIniStringA(L"RobotArm", L"LocalIP");
    if (!localIpFromIni.empty())
    {
        m_sdkLocalIp = localIpFromIni;
    }
    else
    {
        std::string autoIp = PickLocalIpForRemote(m_sdkRemoteIp);
        if (!autoIp.empty())
        {
            m_sdkLocalIp = autoIp;
        }
    }

    {
        CStringA localIpA(m_sdkLocalIp.c_str());
        CString localIpW(localIpA);
        CString msg;
        msg.Format(_T("创建SDK：RemoteIP=%s, LocalIP=%s"), strIP.GetString(), localIpW.GetString());
        LogSdkText(msg, LOG_INFO);
    }

    std::string createErr;

    try
    {
        std::lock_guard<std::mutex> lk(m_sdkMutex);
        try
        {
            m_sdkRobot = std::make_unique<rokae::xMateErProRobot>(m_sdkRemoteIp, m_sdkLocalIp);
        }
        catch (const std::exception& ex)
        {
            createErr = ex.what();
            m_sdkRobot = std::make_unique<rokae::xMateRobot>(m_sdkRemoteIp, m_sdkLocalIp);
        }
    }
    catch (const std::exception& ex)
    {
        m_state = STATE_DISCONNECTED;
        CStringA whatA(ex.what());
        CString whatW(whatA);
        CStringA createErrA(createErr.c_str());
        CString createErrW(createErrA);
        CString msg;
        if (!createErr.empty())
        {
            msg.Format(_T("机械臂连接失败(异常)：创建SDK机器人实例失败。ErPro失败原因: %s。最终异常: %s"), createErrW.GetString(), whatW.GetString());
        }
        else
        {
            msg.Format(_T("机械臂连接失败(异常)：创建SDK机器人实例失败。异常: %s"), whatW.GetString());
        }
        LogSdkText(msg, LOG_ERROR);
        return FALSE;
    }

    m_strServerIP = strIP;
    m_nServerPort = nPort;
    m_state = STATE_CONNECTED;
    LogSdkText(_T("机械臂已连接(SDK)：已创建机器人实例"), LOG_INFO);
    return TRUE;
}

BOOL CRobotArmController::JogJoint(int jointIndex, double deltaDeg, int nSpeed)
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("关节点动失败：未连接SDK机器人"), LOG_ERROR);
        return FALSE;
    }

    if (m_state != STATE_INITIALIZED)
    {
        LogSdkText(_T("关节点动失败：请先点击【初始化】（上电/自动模式）"), LOG_ERROR);
        return FALSE;
    }

    if (jointIndex < 1)
    {
        jointIndex = 1;
    }

    if (jointIndex > 7)
    {
        jointIndex = 7;
    }

    if (nSpeed < 0) nSpeed = 0;
    if (nSpeed > 100) nSpeed = 100;

    ::error_code ec;

    try
    {
        auto ps = m_sdkRobot->powerState(ec);
        if (ec) { LogSdkEc(L"关节点动前查询上电状态(powerState)", ec); return FALSE; }
        if (ps != rokae::PowerState::on)
        {
            CString psMsg;
            psMsg.Format(_T("关节点动失败：当前上电状态=%s"), PowerStateToString(ps).GetString());
            LogSdkText(psMsg, LOG_ERROR);
            return FALSE;
        }

        auto om = m_sdkRobot->operateMode(ec);
        if (ec) { LogSdkEc(L"关节点动前查询手自动模式(operateMode)", ec); return FALSE; }
        if (om != rokae::OperateMode::automatic)
        {
            CString omMsg;
            omMsg.Format(_T("关节点动失败：当前手自动模式=%s"), OperateModeToString(om).GetString());
            LogSdkText(omMsg, LOG_ERROR);
            return FALSE;
        }
    }
    catch (const std::exception&)
    {
        LogSdkText(_T("关节点动失败(异常)：读取机器人状态失败"), LOG_ERROR);
        return FALSE;
    }

    auto joints = m_sdkRobot->jointPos(ec);
    if (ec)
    {
        LogSdkEc(L"关节点动读取当前关节(jointPos)", ec);
        return FALSE;
    }

    if (joints.size() < static_cast<size_t>(jointIndex))
    {
        LogSdkText(_T("关节点动失败：当前关节数量不足"), LOG_ERROR);
        return FALSE;
    }

    joints[static_cast<size_t>(jointIndex - 1)] += deltaDeg * kPi / 180.0;

    int speed_mm_s = nSpeed * 10;
    if (speed_mm_s < 10) speed_mm_s = 10;

    rokae::JointPosition targetJ(joints);
    rokae::MoveAbsJCommand cmd(targetJ, speed_mm_s, 0);

    std::string id;
    m_sdkRobot->moveReset(ec);
    if (ec) { LogSdkEc(L"关节点动(moveReset)", ec); return FALSE; }
    m_sdkRobot->moveAppend({ cmd }, id, ec);
    if (ec) { LogSdkEc(L"关节点动(moveAppend)", ec); return FALSE; }
    m_sdkRobot->moveStart(ec);
    if (ec) { LogSdkEc(L"关节点动(moveStart)", ec); return FALSE; }

    m_state = STATE_MOVING;
    auto start = std::chrono::steady_clock::now();
    int timeoutCount = 0;
    bool seenNonIdle = false;
    while (true)
    {
        auto st = m_sdkRobot->operationState(ec);
        if (IsSdkTimeoutEc(ec))
        {
            ++timeoutCount;
            ec = ::error_code{};
            if (timeoutCount >= 5)
            {
                LogSdkText(_T("关节点动等待失败：operationState() 连续超时，请检查网络/交换机/防火墙。"), LOG_ERROR);
                return FALSE;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (ec) { LogSdkEc(L"关节点动等待(operationState)", ec); return FALSE; }

        if (st == rokae::OperationState::idle)
        {
            if (!seenNonIdle)
            {
                if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2))
                {
                    LogSdkText(_T("关节点动失败：moveStart后运行状态持续为idle（运动未启动）。请检查机器人报警/安全互锁/目标可达性。"), LOG_ERROR);
                    m_state = STATE_ERROR;
                    return FALSE;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            break;
        }
        seenNonIdle = true;

        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(60))
        {
            LogSdkText(_T("关节点动等待超时(60s)：请检查是否卡在安全限制/目标不可达。"), LOG_ERROR);
            return FALSE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    m_state = STATE_INITIALIZED;
    UpdateCachedPositionLocked(m_sdkRobot.get(), m_currentPos);
    return TRUE;
}

BOOL CRobotArmController::GetJointAnglesDeg(std::array<double, 7>& outDeg) const
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        return FALSE;
    }

    ::error_code ec;
    auto joints = m_sdkRobot->jointPos(ec);
    if (ec || joints.size() < 7)
    {
        return FALSE;
    }

    for (size_t i = 0; i < 7; ++i)
    {
        outDeg[i] = joints[i] * 180.0 / kPi;
    }
    return TRUE;
}

void CRobotArmController::Disconnect()
{
    {
        std::lock_guard<std::mutex> lk(m_sdkMutex);
        if (m_sdkRobot)
        {
            ::error_code ec;
            m_sdkRobot->disconnectFromRobot(ec);
            LogSdkEc(L"断开连接", ec);
            m_sdkRobot.reset();
        }
    }

    if (m_socket.m_hSocket != INVALID_SOCKET)
    {
        m_socket.Close();
    }
    m_state = STATE_DISCONNECTED;
}

BOOL CRobotArmController::Initialize()
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("初始化失败：未连接机械臂"), LOG_ERROR);
        return FALSE;
    }

    ::error_code ec;
    try
    {
        m_sdkRobot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        if (ec) { LogSdkEc(L"设置运动控制模式(NrtCommand)", ec); return FALSE; }
        m_sdkRobot->setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec) { LogSdkEc(L"设置手自动模式(automatic)", ec); return FALSE; }
        m_sdkRobot->setPowerState(true, ec);
        if (ec) { LogSdkEc(L"上电(setPowerState true)", ec); return FALSE; }

        auto ps = m_sdkRobot->powerState(ec);
        if (ec) { LogSdkEc(L"查询上电状态(powerState)", ec); return FALSE; }
        CString psMsg;
        psMsg.Format(_T("当前上电状态: %s"), PowerStateToString(ps).GetString());
        LogSdkText(psMsg, LOG_INFO);
        if (ps != rokae::PowerState::on)
        {
            LogSdkText(_T("初始化失败：机器人未处于上电状态(可能急停/安全门/外接使能未满足)"), LOG_ERROR);
            return FALSE;
        }

        auto om = m_sdkRobot->operateMode(ec);
        if (ec) { LogSdkEc(L"查询手自动模式(operateMode)", ec); return FALSE; }
        CString omMsg;
        omMsg.Format(_T("当前手自动模式: %s"), OperateModeToString(om).GetString());
        LogSdkText(omMsg, LOG_INFO);
        if (om != rokae::OperateMode::automatic)
        {
            LogSdkText(_T("初始化失败：机器人未处于自动模式"), LOG_ERROR);
            return FALSE;
        }

        m_sdkRobot->moveReset(ec);
        if (ec) { LogSdkEc(L"运动队列复位(moveReset)", ec); return FALSE; }

        auto joints = m_sdkRobot->jointPos(ec);
        if (!ec && !joints.empty())
        {
            CString js(_T("当前关节角(rad): "));
            for (size_t i = 0; i < joints.size(); ++i)
            {
                CString part;
                part.Format(_T("J%u=%.6f "), static_cast<unsigned>(i + 1), joints[i]);
                js += part;
            }
            LogSdkText(js, LOG_INFO);
        }
        else if (ec)
        {
            LogSdkEc(L"读取当前关节角(jointPos)", ec);
            ec = ::error_code{};
        }
    }
    catch (const std::exception&)
    {
        m_state = STATE_ERROR;
        LogSdkText(_T("初始化失败(异常)：请检查急停/安全门/权限/机器人状态"), LOG_ERROR);
        return FALSE;
    }

    m_state = STATE_INITIALIZED;
    UpdateCachedPositionLocked(m_sdkRobot.get(), m_currentPos);
    LogSdkText(_T("机械臂初始化成功"), LOG_INFO);
    return TRUE;
}

BOOL CRobotArmController::GoHome()
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("归零失败：未连接机械臂"), LOG_ERROR);
        return FALSE;
    }

    if (m_state != STATE_INITIALIZED)
    {
        LogSdkText(_T("归零失败：请先点击【初始化】（上电/自动模式）"), LOG_ERROR);
        return FALSE;
    }

    ::error_code ec;
    try
    {
        // 仅支持7轴机型：xMateErProRobot
        if (dynamic_cast<rokae::xMateErProRobot*>(m_sdkRobot.get()) == nullptr)
        {
            LogSdkText(_T("归零失败：当前连接的不是7轴 xMateErProRobot"), LOG_ERROR);
            return FALSE;
        }

        // 从 config.ini 读取 Home 关节角(rad)
        CString iniPath = GetConfigIniPath();
        TCHAR buf[512] = { 0 };
        GetPrivateProfileString(_T("RobotArm"), _T("HomeJointsRad"), _T(""), buf, 512, iniPath);
        CString homeText(buf);
        if (homeText.IsEmpty())
        {
            LogSdkText(_T("已禁用归零动作：未在config.ini配置[RobotArm] HomeJointsRad（7个弧度值）。"), LOG_WARNING);
            return FALSE;
        }

        std::vector<double> home;
        if (!ParseDoubles(homeText, home) || home.size() != 7)
        {
            LogSdkText(_T("归零失败：HomeJointsRad格式错误，必须是7个数字（单位rad，空格/逗号/分号分隔）。"), LOG_ERROR);
            return FALSE;
        }

        // 归零前检查上电/自动模式（对应手册 powerState/operateMode）
        auto ps = m_sdkRobot->powerState(ec);
        if (ec) { LogSdkEc(L"归零前查询上电状态(powerState)", ec); return FALSE; }
        if (ps != rokae::PowerState::on)
        {
            CString psMsg;
            psMsg.Format(_T("归零失败：当前上电状态=%s"), PowerStateToString(ps).GetString());
            LogSdkText(psMsg, LOG_ERROR);
            return FALSE;
        }

        auto om = m_sdkRobot->operateMode(ec);
        if (ec) { LogSdkEc(L"归零前查询手自动模式(operateMode)", ec); return FALSE; }
        if (om != rokae::OperateMode::automatic)
        {
            CString omMsg;
            omMsg.Format(_T("归零失败：当前手自动模式=%s"), OperateModeToString(om).GetString());
            LogSdkText(omMsg, LOG_ERROR);
            return FALSE;
        }

        // 安全限幅：若与当前关节角差异过大，拒绝执行（避免误动作）
        auto curr = m_sdkRobot->jointPos(ec);
        if (!ec && curr.size() >= 7)
        {
            double maxAbsDelta = 0.0;
            for (size_t i = 0; i < 7; ++i)
            {
                double d = std::fabs(curr[i] - home[i]);
                if (d > maxAbsDelta) maxAbsDelta = d;
            }
            if (maxAbsDelta > 2.5)
            {
                LogSdkText(_T("归零已拒绝：目标Home与当前姿态差异过大（>2.5rad）。请确认Home关节角与现场安装姿态一致。"), LOG_ERROR);
                return FALSE;
            }
        }
        else if (ec)
        {
            LogSdkEc(L"归零前读取当前关节角(jointPos)", ec);
            ec = ::error_code{};
        }

        std::string id;
        m_sdkRobot->moveReset(ec);
        if (ec) { LogSdkEc(L"归零(moveReset)", ec); return FALSE; }

        rokae::MoveAbsJCommand cmd({ home[0], home[1], home[2], home[3], home[4], home[5], home[6] });
        m_sdkRobot->moveAppend({ cmd }, id, ec);
        if (ec) { LogSdkEc(L"归零(moveAppend)", ec); return FALSE; }

        m_sdkRobot->moveStart(ec);
        if (ec) { LogSdkEc(L"归零(moveStart)", ec); return FALSE; }

        LogSdkText(_T("归零指令已下发(moveStart)：等待完成..."), LOG_INFO);

        auto start = std::chrono::steady_clock::now();
        int timeoutCount = 0;
        while (true)
        {
            auto st = m_sdkRobot->operationState(ec);
            if (IsSdkTimeoutEc(ec))
            {
                ++timeoutCount;
                ec = ::error_code{};
                if (timeoutCount >= 5)
                {
                    LogSdkText(_T("归零等待失败：operationState() 连续超时，请检查网络/交换机/防火墙。"), LOG_ERROR);
                    return FALSE;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            if (ec) { LogSdkEc(L"归零等待(operationState)", ec); return FALSE; }

            if (st == rokae::OperationState::idle || st == rokae::OperationState::unknown)
            {
                break;
            }

            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(60))
            {
                LogSdkText(_T("归零等待超时(60s)：请检查是否卡在安全限制/目标不可达。"), LOG_ERROR);
                return FALSE;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        UpdateCachedPositionLocked(m_sdkRobot.get(), m_currentPos);
        LogSdkText(_T("机械臂归零完成"), LOG_INFO);
        return TRUE;
    }
    catch (const std::exception&)
    {
        m_state = STATE_ERROR;
        LogSdkText(_T("归零失败(异常)：可能是目标点不可达/安全限制/机器人状态不允许运动"), LOG_ERROR);
        return FALSE;
    }
}

BOOL CRobotArmController::MoveTo(const RobotArmPosition& pos, int nSpeed)
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("移动失败：未连接机械臂"), LOG_ERROR);
        return FALSE;
    }

    if (m_state != STATE_INITIALIZED)
    {
        LogSdkText(_T("移动失败：请先点击【初始化】（上电/自动模式）"), LOG_ERROR);
        return FALSE;
    }

    if (nSpeed < 0) nSpeed = 0;
    if (nSpeed > 100) nSpeed = 100;

    ::error_code ec;
    try
    {
        auto ps = m_sdkRobot->powerState(ec);
        if (ec) { LogSdkEc(L"移动前查询上电状态(powerState)", ec); return FALSE; }
        if (ps != rokae::PowerState::on)
        {
            CString psMsg;
            psMsg.Format(_T("移动失败：当前上电状态=%s"), PowerStateToString(ps).GetString());
            LogSdkText(psMsg, LOG_ERROR);
            return FALSE;
        }

        auto om = m_sdkRobot->operateMode(ec);
        if (ec) { LogSdkEc(L"移动前查询手自动模式(operateMode)", ec); return FALSE; }
        if (om != rokae::OperateMode::automatic)
        {
            CString omMsg;
            omMsg.Format(_T("移动失败：当前手自动模式=%s"), OperateModeToString(om).GetString());
            LogSdkText(omMsg, LOG_ERROR);
            return FALSE;
        }

        auto curr = m_sdkRobot->posture(rokae::CoordinateType::flangeInBase, ec);
        if (ec) { LogSdkEc(L"读取当前位姿(posture)", ec); return FALSE; }

        std::array<double, 6> target = curr;
        target[0] = pos.X / 1000.0;
        target[1] = pos.Y / 1000.0;
        target[2] = pos.Z / 1000.0;

        int speed_mm_s = nSpeed * 10;
        if (speed_mm_s < 10) speed_mm_s = 10;
        rokae::MoveLCommand cmd(target, speed_mm_s, 0);

        std::string id;
        m_sdkRobot->moveReset(ec);
        if (ec) { LogSdkEc(L"移动(moveReset)", ec); return FALSE; }
        m_sdkRobot->moveAppend({ cmd }, id, ec);
        if (ec) { LogSdkEc(L"移动(moveAppend)", ec); return FALSE; }
        m_sdkRobot->moveStart(ec);
        if (ec) { LogSdkEc(L"移动(moveStart)", ec); return FALSE; }
        LogSdkText(_T("移动指令已下发(moveStart)"), LOG_INFO);

        m_state = STATE_MOVING;
        auto start = std::chrono::steady_clock::now();
        int timeoutCount = 0;
        bool seenNonIdle = false;
        while (true)
        {
            auto st = m_sdkRobot->operationState(ec);
            if (IsSdkTimeoutEc(ec))
            {
                ++timeoutCount;
                ec = ::error_code{};
                if (timeoutCount >= 5)
                {
                    LogSdkText(_T("查询运行状态(operationState) 连续超时：请检查网络/网口/交换机/防火墙"), LOG_ERROR);
                    return FALSE;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            if (ec) { LogSdkEc(L"查询运行状态(operationState)", ec); return FALSE; }
            if (st == rokae::OperationState::idle)
            {
                if (!seenNonIdle)
                {
                    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2))
                    {
                        LogSdkText(_T("移动失败：moveStart后运行状态持续为idle（运动未启动）。请检查机器人报警/安全互锁/目标可达性。"), LOG_ERROR);
                        m_state = STATE_ERROR;
                        return FALSE;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                break;
            }
            seenNonIdle = true;

            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(60))
            {
                m_state = STATE_ERROR;
                return FALSE;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        auto after = m_sdkRobot->posture(rokae::CoordinateType::flangeInBase, ec);
        if (ec) { LogSdkEc(L"移动后读取位姿(posture)", ec); return FALSE; }
        const double ax = after[0] * 1000.0;
        const double ay = after[1] * 1000.0;
        const double az = after[2] * 1000.0;
        const double dx = fabs(ax - pos.X);
        const double dy = fabs(ay - pos.Y);
        const double dz = fabs(az - pos.Z);
        if (dx > 5.0 || dy > 5.0 || dz > 5.0)
        {
            CString msg;
            msg.Format(_T("移动到位校验失败：目标(%.2f, %.2f, %.2f) 实际(%.2f, %.2f, %.2f)"),
                pos.X, pos.Y, pos.Z, (float)ax, (float)ay, (float)az);
            LogSdkText(msg, LOG_ERROR);
            m_state = STATE_ERROR;
            return FALSE;
        }
    }
    catch (const std::exception&)
    {
        m_state = STATE_ERROR;
        LogSdkText(_T("移动失败(异常)：可能是目标点不可达/安全限制/机器人状态不允许运动"), LOG_ERROR);
        return FALSE;
    }

    m_state = STATE_INITIALIZED;
    UpdateCachedPositionLocked(m_sdkRobot.get(), m_currentPos);
    return TRUE;
}

BOOL CRobotArmController::RotateJ7(double thetaDeg, int nSpeed)
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("旋转J7失败：未连接机械臂"), LOG_ERROR);
        return FALSE;
    }

    if (m_state != STATE_INITIALIZED)
    {
        LogSdkText(_T("旋转J7失败：请先点击【初始化】（上电/自动模式）"), LOG_ERROR);
        return FALSE;
    }

    if (nSpeed < 0) nSpeed = 0;
    if (nSpeed > 100) nSpeed = 100;

    // 软限位：7轴±360°（按手册）
    if (thetaDeg > 360.0) thetaDeg = 360.0;
    if (thetaDeg < -360.0) thetaDeg = -360.0;

    ::error_code ec;
    try
    {
        auto ps = m_sdkRobot->powerState(ec);
        if (ec) { LogSdkEc(L"旋转J7前查询上电状态(powerState)", ec); return FALSE; }
        if (ps != rokae::PowerState::on)
        {
            CString psMsg;
            psMsg.Format(_T("旋转J7失败：当前上电状态=%s"), PowerStateToString(ps).GetString());
            LogSdkText(psMsg, LOG_ERROR);
            return FALSE;
        }

        auto om = m_sdkRobot->operateMode(ec);
        if (ec) { LogSdkEc(L"旋转J7前查询手自动模式(operateMode)", ec); return FALSE; }
        if (om != rokae::OperateMode::automatic)
        {
            CString omMsg;
            omMsg.Format(_T("旋转J7失败：当前手自动模式=%s"), OperateModeToString(om).GetString());
            LogSdkText(omMsg, LOG_ERROR);
            return FALSE;
        }

        auto joints = m_sdkRobot->jointPos(ec);
        if (ec) { LogSdkEc(L"旋转J7读取当前关节角(jointPos)", ec); return FALSE; }
        if (joints.size() < 7)
        {
            LogSdkText(_T("旋转J7失败：当前机型关节数不足7轴"), LOG_ERROR);
            return FALSE;
        }

        joints[6] = thetaDeg * kPi / 180.0;

        int speed_mm_s = nSpeed * 10;
        if (speed_mm_s < 10) speed_mm_s = 10;

        rokae::JointPosition targetJ(joints);
        rokae::MoveAbsJCommand cmd(targetJ, speed_mm_s, 0);

        std::string id;
        m_sdkRobot->moveReset(ec);
        if (ec) { LogSdkEc(L"旋转J7(moveReset)", ec); return FALSE; }
        m_sdkRobot->moveAppend({ cmd }, id, ec);
        if (ec) { LogSdkEc(L"旋转J7(moveAppend)", ec); return FALSE; }
        m_sdkRobot->moveStart(ec);
        if (ec) { LogSdkEc(L"旋转J7(moveStart)", ec); return FALSE; }

        m_state = STATE_MOVING;
        auto start = std::chrono::steady_clock::now();
        int timeoutCount = 0;
        bool seenNonIdle = false;
        while (true)
        {
            auto st = m_sdkRobot->operationState(ec);
            if (IsSdkTimeoutEc(ec))
            {
                ++timeoutCount;
                ec = ::error_code{};
                if (timeoutCount >= 5)
                {
                    LogSdkText(_T("旋转J7等待失败：operationState() 连续超时，请检查网络/交换机/防火墙。"), LOG_ERROR);
                    return FALSE;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            if (ec) { LogSdkEc(L"旋转J7等待(operationState)", ec); return FALSE; }
            if (st == rokae::OperationState::idle)
            {
                if (!seenNonIdle)
                {
                    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2))
                    {
                        LogSdkText(_T("旋转J7失败：moveStart后运行状态持续为idle（运动未启动）。请检查机器人报警/安全互锁/目标可达性。"), LOG_ERROR);
                        m_state = STATE_ERROR;
                        return FALSE;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                break;
            }
            seenNonIdle = true;

            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(60))
            {
                LogSdkText(_T("旋转J7等待超时(60s)：请检查是否卡在安全限制/目标不可达。"), LOG_ERROR);
                return FALSE;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        auto afterJ = m_sdkRobot->jointPos(ec);
        if (ec) { LogSdkEc(L"旋转J7后读取关节角(jointPos)", ec); return FALSE; }
        if (afterJ.size() >= 7)
        {
            const double j7Deg = afterJ[6] * 180.0 / kPi;
            if (fabs(j7Deg - thetaDeg) > 2.0)
            {
                CString msg;
                msg.Format(_T("旋转J7到位校验失败：目标=%.2f° 实际=%.2f°"), (float)thetaDeg, (float)j7Deg);
                LogSdkText(msg, LOG_ERROR);
                m_state = STATE_ERROR;
                return FALSE;
            }
        }
    }
    catch (const std::exception&)
    {
        m_state = STATE_ERROR;
        LogSdkText(_T("旋转J7失败(异常)：可能是安全限制/机器人状态不允许运动"), LOG_ERROR);
        return FALSE;
    }

    m_state = STATE_INITIALIZED;
    UpdateCachedPositionLocked(m_sdkRobot.get(), m_currentPos);
    return TRUE;
}

BOOL CRobotArmController::Stop()
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("停止失败：未连接机械臂"), LOG_ERROR);
        return FALSE;
    }

    ::error_code ec;
    try
    {
        m_sdkRobot->stop(ec);
        if (ec) { LogSdkEc(L"停止(stop)", ec); return FALSE; }
        m_sdkRobot->moveReset(ec);
        if (ec) { LogSdkEc(L"停止(moveReset)", ec); return FALSE; }
    }
    catch (const std::exception&)
    {
        return FALSE;
    }

    if (ec)
    {
        return FALSE;
    }

    m_state = STATE_INITIALIZED;
    return TRUE;
}

BOOL CRobotArmController::EmergencyStop()
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        LogSdkText(_T("急停失败：未连接机械臂"), LOG_ERROR);
        return FALSE;
    }

    ::error_code ec;
    try
    {
        m_sdkRobot->stop(ec);
        if (ec) { LogSdkEc(L"急停(stop)", ec); return FALSE; }
        m_sdkRobot->moveReset(ec);
        if (ec) { LogSdkEc(L"急停(moveReset)", ec); return FALSE; }
        m_sdkRobot->setPowerState(false, ec);
        if (ec) { LogSdkEc(L"急停(下电 setPowerState false)", ec); return FALSE; }
    }
    catch (const std::exception&)
    {
        return FALSE;
    }

    if (ec)
    {
        m_state = STATE_ERROR;
        return FALSE;
    }

    m_state = STATE_INITIALIZED;
    return TRUE;
}

void CRobotArmController::SetSpeed(int nSpeed)
{
    if (nSpeed < 0)
        nSpeed = 0;
    if (nSpeed > 100)
        nSpeed = 100;
    
    m_nSpeed = nSpeed;
}

BOOL CRobotArmController::IsConnected() const
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    return m_sdkRobot != nullptr && m_state != STATE_DISCONNECTED;
}

RobotArmPosition CRobotArmController::GetCurrentPosition() const
{
    std::lock_guard<std::mutex> lk(m_sdkMutex);
    if (!m_sdkRobot)
    {
        return m_currentPos;
    }

    ::error_code ec;
    try
    {
        auto p = m_sdkRobot->posture(rokae::CoordinateType::flangeInBase, ec);
        if (!ec)
        {
            m_currentPos.X = p[0] * 1000.0;
            m_currentPos.Y = p[1] * 1000.0;
            m_currentPos.Z = p[2] * 1000.0;

            auto joints = m_sdkRobot->jointPos(ec);
            if (!ec && joints.size() >= 7)
            {
                m_currentPos.Theta = joints[6] * 180.0 / kPi;
            }
        }
    }
    catch (const std::exception&)
    {
        // keep last cached value
    }

    return m_currentPos;
}

BOOL CRobotArmController::SendCommand(const CString& strCmd, CString& strResponse, DWORD dwTimeout)
{
    (void)strCmd;
    (void)strResponse;
    (void)dwTimeout;
    return FALSE;

    if (m_socket.m_hSocket == INVALID_SOCKET)
    {
        return FALSE;
    }

    // 转换为 ANSI 字符串发送
    CStringA strCmdA(strCmd);
    
    // 发送命令
    int nSent = m_socket.Send(strCmdA.GetString(), strCmdA.GetLength());
    if (nSent == SOCKET_ERROR)
    {
        return FALSE;
    }

    // 等待响应
    DWORD dwStartTime = GetTickCount();
    char szBuffer[256] = {0};
    
    strResponse.Empty();
    
    while (GetTickCount() - dwStartTime < dwTimeout)
    {
        // 设置接收超时
        int nTimeout = dwTimeout;
        m_socket.SetSockOpt(SO_RCVTIMEO, &nTimeout, sizeof(nTimeout));
        
        int nReceived = m_socket.Receive(szBuffer, sizeof(szBuffer) - 1);
        if (nReceived > 0)
        {
            szBuffer[nReceived] = '\0';
            strResponse += CString(szBuffer);
            
            // 检查是否收到完整响应（以\r\n结尾）
            if (strResponse.Find(_T("\r\n")) >= 0)
            {
                return TRUE;
            }
        }
        else if (nReceived == SOCKET_ERROR)
        {
            int nError = WSAGetLastError();
            if (nError != WSAETIMEDOUT)
            {
                // 真正的错误
                return FALSE;
            }
        }
        
        Sleep(10);
    }

    // 超时，但如果有部分响应也返回TRUE
    return !strResponse.IsEmpty();
}

BOOL CRobotArmController::ParsePosition(const CString& strResponse, RobotArmPosition& pos)
{
    (void)strResponse;
    (void)pos;
    return FALSE;
}
