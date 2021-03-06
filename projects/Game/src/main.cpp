#include <iostream>
#include <Windows.h>

#include "C_Vehicle.h"

#include "I3d_math.h"
#include "I3d_frame.h"

void Init();

BOOL WINAPI DllMain(
    HINSTANCE hinstDLL,  // handle to DLL module
    DWORD fdwReason,     // reason for calling function
    LPVOID lpReserved)  // reserved
{
    // Perform actions based on the reason for calling.
    switch (fdwReason)
    {
    case DLL_PROCESS_ATTACH:
        Init();
        break;

    case DLL_THREAD_ATTACH:
        break;

    case DLL_THREAD_DETACH:
        break;

    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;  // Successful DLL_PROCESS_ATTACH.
}

void Init() {
    AllocConsole();
    freopen("CONOUT$", "w", stdout);

    I3D_math::InitHooks();
    I3D_frame::InitHooks();
    C_Vehicle::InitHooks();
}