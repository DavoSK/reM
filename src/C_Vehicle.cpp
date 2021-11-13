#include "C_Vehicle.h"

#include <cstdint>
#include <cstdio>

#include <Windows.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include <reversiblehooks/ReversibleHooks.h>
#include "plugin.h"

S_vector* C_Vehicle::GetWheelCamPos(S_vector* outPos, int wheelIdx, S_vector* offset)
{
    float offsetX = 0.0f;
    float* wheel = (float*)m_pWheels[wheelIdx];

    if (wheelIdx % 2)
        offsetX = offset->x + wheel[3];
    else
        offsetX = -offset->x - wheel[3];
    
    outPos->x = offsetX + wheel[4];
    outPos->y = offset->y + wheel[5];
    outPos->z = offset->z + wheel[6];
    return outPos;
}

bool C_Vehicle::SetGear(int32_t gear)
{
    //printf("sizeof C_Vehicle: 0x%X\n", sizeof(C_Vehicle));
    //printf("C_Vehicle::SetGear want: %d, current: %d  max: %d | m_iGear: 0x%X\n", gear, m_iGear, m_iMaxGear, offsetof(C_Vehicle, m_iGear));

    if (gear >= -1 && gear <= m_iMaxGear)
    {
        if (m_iGear == gear)
            return true;

        // if (*(uint32_t*)(this + 0xB8) != 1 || gear >= 0)
        // {
        //     if ((*(uint32_t*)(this + 0xC68) & 0x4000) == 0
        //         || (v3 = *(float*)(this + 0x1EC), v4 = v3 < 0.0, v5 = 0, v6 = v3 == 0.0, (v2 & 0x4100) != 0))
        //     {
        //         if ((*(uint32_t*)(this + 0xC68) & 0x4000) != 0)
        //             return 1;
        //     }
        //     if (gear == -1
        //         || (v8 = *(float*)(this + 4 * gear + 0x570)
        //             / *(float*)(this + 4 * *(uint32_t*)(this + 0x55C) + 0x570)
        //             * *(float*)(this + 0x548),
        //             v9 = v8 < *(float*)(this + 0x1F9C),
        //             v10 = 0,
        //             v11 = v8 == *(float*)(this + 0x1F9C),
        //             (v7 & 0x4100) != 0))
        //     {
        //         v12 = *(uint32_t*)(this + 0x5E8);
        //         *(uint32_t*)(this + 0x494) = 0;
        //         if ((v12 & 0x20) != 0)
        //         {
        //             v12 = v12 & 0xFC | 2;
        //             *(uint32_t*)(this + 0x5E8) = v12;
        //         }

        //         m_iGear = gear;
        //         return 1;
        //     }
        // }
    }


    return 0;
}

bool C_Vehicle::SetSpeedLimit(float limit)
{
    m_fSpeedLimit = limit;
    if(m_fSpeedLimit > 0.0f)
        m_fSpeedLimit = limit + 0.5f;
    return true;
}

bool C_Vehicle::SetClutch(float clutch)
{
    clutch = std::clamp(clutch, 0.0f, 1.0f);
    
    if ( m_bDontInterpolateSteering )
    {
        m_fClutch = ((1.0f - clutch) * m_fClutchLinearity + clutch) * clutch;
    }
    else
    {
        m_fClutch = clutch;
    }

    return true;
}

bool C_Vehicle::SetSteer(float steer)
{
    float steerChange = steer;
    if ( m_bDontInterpolateSteering )
    {
        if ( steer > 0.0f )
        {
            steerChange = ((1.0f - steer) * m_fSteeringLinearity + steer) * steer;
        }
        else
        {
            steerChange = -((steer - (steer + 1.0f) * m_fSteeringLinearity) * steer);
        }
    }

    m_fSteerAngle = steerChange * m_fMaxSteerAngle;
    return true;
}

bool C_Vehicle::SetSteeringLinearity(float lin)
{
    if ( lin >= 0.0f )
    {
        if ( lin <= 1.0f )
        {
            m_fSteeringLinearity = lin;
            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

bool C_Vehicle::SoundOff() 
{
    return plugin::CallMethodAndReturn<bool, 0x4EEC80>(this);
}

bool C_Vehicle::SetFuel(float fuel)
{   
    if ( fuel > m_fMaxFuel )
        fuel = m_fMaxFuel;
    
    m_fFuel = fuel;
    return true;
}

bool C_Vehicle::SetHandbrake(bool handbrake)
{
    if (handbrake)
    {
        float v2 = *(float*)((uint32_t)this + 0x4C8) + *(float*)((uint32_t)this + 0x3B8);
        *(float*)((uint32_t)this + 0x3B8) = v2;

        if (v2 > *(float*)((uint32_t)this + 0x3B4))
            *(uint32_t*)((uint32_t)this + 0x3B8) = *(uint32_t*)((uint32_t)this + 0x3B4);
        
        *(float*)((uint32_t)this + 0x434) = *(float*)((uint32_t)this + 0x3B8) / *(float*)((uint32_t)this + 0x3B4) * *(float*)((uint32_t)this + 0x3B0);
    }
    else
    {
        *(uint32_t*)((uint32_t)this + 0x434) = 0;
        *(uint32_t*)((uint32_t)this + 0x3B8) = 0;
    }

    return true;
}

bool C_Vehicle::EnableSounds(bool enable)
{
    bool result = false;
    if ( !enable )
    {
        result = m_SoundEnabled;
        if ( result )
            result = SoundOff();
    }

    m_SoundEnabled = enable;
    return result;
}

void C_Vehicle::InitHooks()
{
    //ReversibleHooks::Install("C_Vehicle", "SetGear", 0x4CB070, &C_Vehicle::SetGear);
    ReversibleHooks::Install("C_Vehicle", "GetWheelCamPos", 0x4C6010, &C_Vehicle::GetWheelCamPos);
    ReversibleHooks::Install("C_Vehicle", "SetSpeedLimit", 0x4CB6A0, &C_Vehicle::SetSpeedLimit);
    ReversibleHooks::Install("C_Vehicle", "SetClutch", 0x4CB460, &C_Vehicle::SetClutch);
    ReversibleHooks::Install("C_Vehicle", "SetSteer", 0x4CB4D0, &C_Vehicle::SetSteer);
    ReversibleHooks::Install("C_Vehicle", "SetSteeringLinearity", 0x4CB570, &C_Vehicle::SetSteeringLinearity);
    ReversibleHooks::Install("C_Vehicle", "SetFuel", 0x4CB6E0, &C_Vehicle::SetFuel);
    ReversibleHooks::Install("C_Vehicle", "SetHandbrake", 0x4CB710, &C_Vehicle::SetHandbrake);
    ReversibleHooks::Install("C_Vehicle", "EnableSounds", 0x4CB780, &C_Vehicle::EnableSounds);
}
