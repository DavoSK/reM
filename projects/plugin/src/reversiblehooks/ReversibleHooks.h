#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <Windows.h>


template<typename T>
void* FunctionPointerToVoidP(T func)
{
    union
    {
        T a;
        void* b;
    } c = { func };
    return c.b;
}


#define VALIDATE_SIZE(struc, size) static_assert(sizeof(struc) == size, "Invalid structure size of " #struc)

const std::uint32_t JUMP_OPCODE = 0xE9;
const std::uint32_t NOP_OPCODE = 0x90;

enum class eReversibleHookType {
    Simple,
    Virtual
};

#pragma pack(push, 1)
struct SHookContent {
    unsigned char jumpOpCode = JUMP_OPCODE;
    unsigned int jumpLocation = 0;
    unsigned char possibleNops[52 - sizeof(jumpOpCode) - sizeof(jumpLocation)] = { 0 };
};
#pragma pack(pop)
VALIDATE_SIZE(SHookContent, 0x34);

struct SReversibleHook {
    bool m_bIsHooked = false;
    bool m_bImguiHooked = false;
    std::string m_sIdentifier;
    std::string m_sFunctionName;
    eReversibleHookType m_eHookType;

    virtual void Switch() = 0;
};

struct SSimpleReversibleHook : SReversibleHook {
    SHookContent m_HookContent;
    unsigned char m_OriginalFunctionContent[sizeof(m_HookContent)];
    unsigned int m_iHookedBytes;
    unsigned int m_iRealHookedAddress;

    SHookContent m_LibHookContent;
    unsigned char m_LibOriginalFunctionContent[sizeof(m_LibHookContent)];
    unsigned int m_iLibHookedBytes;
    unsigned int m_iLibFunctionAddress;

    static std::shared_ptr<SSimpleReversibleHook> InstallHook(uint32_t installAddress, void* addressToJumpTo, int iJmpCodeSize = 5);
    virtual void Switch() override;
};

struct SVirtualReversibleHook : SReversibleHook {
    std::vector<uint32_t> m_vecHookedAddresses;
    uint32_t m_OriginalFunctionAddress;
    uint32_t m_LibFunctionAddress;

    static std::shared_ptr<SVirtualReversibleHook> InstallHook(void* libFuncAddress, const std::vector<uint32_t>& vecAddressesToHook);
    virtual void Switch() override;
};

class ReversibleHooks {
public:
    static ReversibleHooks& GetInstance() {
        static ReversibleHooks instance;
        return instance;
    }

    template <typename T>
    static void Install(const std::string& sIdentifier, const std::string& sFuncName, DWORD installAddress, T addressToJumpTo, bool bDisableByDefault = false, int iJmpCodeSize = 5) {
        auto ptr = FunctionPointerToVoidP(addressToJumpTo);
        ReversibleHooks::GetInstance().HookInstall(sIdentifier, sFuncName, installAddress, ptr, iJmpCodeSize, bDisableByDefault);
    }

    template <typename T>
    static void InstallVirtual(const std::string& sIdentifier, const std::string& sFuncName, T libVTableAddress, const std::vector<uint32_t>& vecAddressesToHook) {
        auto ptr = FunctionPointerToVoidP(libVTableAddress);
        ReversibleHooks::GetInstance().HookInstallVirtual(sIdentifier, sFuncName, ptr, vecAddressesToHook);
    }

    static void Switch(std::shared_ptr<SReversibleHook> pHook) {
        ReversibleHooks::GetInstance().HookSwitch(pHook);
    }
    static std::map<std::string, std::vector<std::shared_ptr<SReversibleHook>>>& GetAllHooks() {
        return ReversibleHooks::GetInstance().m_HooksMap;
    }

    static void UnHook(const std::string& className, const char* functionName = nullptr);
private:
    void HookInstall(const std::string& sIdentifier, const std::string& sFuncName, unsigned int installAddress, void* addressToJumpTo, int iJmpCodeSize = 5, bool bDisableByDefault = false);
    void HookInstallVirtual(const std::string& sIdentifier, const std::string& sFuncName, void* libVTableAddress, const std::vector<uint32_t>& vecAddressesToHook);
    void HookSwitch(std::shared_ptr<SReversibleHook> pHook) const;
    bool IsFunctionHooked(const std::string& sIdentifier, const std::string& sFuncName);
    std::shared_ptr<SReversibleHook> GetHook(const std::string& sIdentifier, const std::string& sFuncName);

public:
    static constexpr unsigned int x86JMPSize = 5U;
    static unsigned int GetJMPLocation(unsigned int dwFrom, unsigned int dwTo);
    static unsigned int GetFunctionLocationFromJMP(unsigned int dwJmpLoc, unsigned int dwJmpOffset);

private:
    std::map<std::string, std::vector<std::shared_ptr<SReversibleHook>>> m_HooksMap;
    ReversibleHooks() = default;
    ReversibleHooks(ReversibleHooks const&) = delete;
    void operator=(ReversibleHooks const&) = delete;
};
