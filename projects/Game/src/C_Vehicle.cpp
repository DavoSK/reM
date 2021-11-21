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
    if (gear >= -1 && gear <= m_iMaxGear)
    {
        if (m_iGear == gear)
            return true;

        uint32_t v1 = *(uint32_t*)((uint32_t)this + 0xB8);
        if (v1 != 1 || gear >= 0)
        {
            float v3 = *(float*)((uint32_t)this + 0x1EC);
            if ((*(uint32_t*)((uint32_t)this + 0xC68) & 0x4000) == 0 || v3 < 0.0f)
            {
                if ((*(uint32_t*)((uint32_t)this + 0xC68) & 0x4000) != 0)
                    return true;
            }

            float v8 = m_GearRatios[gear] /
                       m_GearRatios[m_iLastGear] * *(float*)((uint32_t)this + 0x548);

            float v9 = *(float*)((uint32_t)this + 0x1F9C);
            if (gear == -1 || v8 <= v9)
            {
                uint32_t v12 = *(uint32_t*)((uint32_t)this + 0x5E8);
                *(uint32_t*)((uint32_t)this + 0x494) = 0;
                if ((v12 & 0x20) != 0)
                {
                    v12 = v12 & 0xFC | 2;
                    *(uint32_t*)((uint32_t)this + 0x5E8) = v12;
                }

                m_iGear = gear;
                return 1;
            }
        }
    }

    return 0;
}

bool C_Vehicle::SetBrake(float brake)
{
    if (brake > 1.0f || brake < 0.0f || *(uint32_t*)((uint32_t)this + 0xB0) == 1)
        return false;

    if (!*(uint8_t*)((uint32_t)this + 0x4CC))
    {
        if ( m_bDontInterpolateBrake )
        {
            m_fBrake = brake;
        }
        else
        {
            m_fBrakeCurrent = m_fDeltaUpdateTime + m_fBrakeCurrent;
              
            if (m_fBrakeCurrent > m_fBrakeMax)
                m_fBrakeCurrent = m_fBrakeMax;

            m_fBrake = m_fBrakeCurrent / m_fBrakeMax * brake;
        }

        if (brake > 0.0f)
        {
            *(uint32_t*)((uint32_t)this + 0x3A8) = 0;
            *(uint32_t*)((uint32_t)this + 0x3A4) = 0;
            return true;
        }
        return true;
    }

    if (*(int*)((uint32_t)this + 0x55C) < 0)
    {
        float v25 = *(uint8_t*)((uint32_t)this + 0xC2C);
        *(uint32_t*)((uint32_t)this + 0x3A4) = 0;
        *(uint32_t*)((uint32_t)this + 0x3A8) = 0;
        if (v25)
            *(float*)((uint32_t)this + 0x3A4) = brake;
        return true;
    }

    if (m_bDontInterpolateBrake)
    {
        m_fBrake = brake;
    }
    else
    {
        m_fBrakeCurrent = m_fDeltaUpdateTime + m_fBrakeCurrent;
        
        if (m_fBrakeCurrent > m_fBrakeMax)
            m_fBrakeCurrent = m_fBrakeMax;

        m_fBrake = m_fBrakeCurrent / m_fBrakeMax * brake;
    }

 
    if (brake > 0.0f)
    {
        *(uint32_t*)((uint32_t)this + 0x3A8) = 0;
        *(uint32_t*)((uint32_t)this + 0x3A4) = 0;
    }

    *(uint32_t*)((uint32_t)this + 0x1A4) &= ~0x1000000u;
    return true;
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

bool C_Vehicle::SetHandbrake(bool doBrake)
{
    if (doBrake)
    {
        float val = m_fDeltaUpdateTime + m_fHandbrakeCurrent;
        m_fHandbrakeCurrent = val;

        if (val > m_fHandbrakeMax)
            m_fHandbrakeCurrent = m_fHandbrakeMax;
        
        m_fHandbrake = m_fHandbrakeCurrent / m_fHandbrakeMax * m_fMaxHandbrakeForce;
    }
    else
    {
        m_fHandbrake = 0.0f;
        m_fHandbrakeCurrent = 0.0f;
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
    ReversibleHooks::Install("C_Vehicle", "SetBrake", 0x4CB2D0, &C_Vehicle::SetBrake);
    ReversibleHooks::Install("C_Vehicle", "SetGear", 0x4CB070, &C_Vehicle::SetGear);
    ReversibleHooks::Install("C_Vehicle", "GetWheelCamPos", 0x4C6010, &C_Vehicle::GetWheelCamPos);
    ReversibleHooks::Install("C_Vehicle", "SetSpeedLimit", 0x4CB6A0, &C_Vehicle::SetSpeedLimit);
    ReversibleHooks::Install("C_Vehicle", "SetClutch", 0x4CB460, &C_Vehicle::SetClutch);
    ReversibleHooks::Install("C_Vehicle", "SetSteer", 0x4CB4D0, &C_Vehicle::SetSteer);
    ReversibleHooks::Install("C_Vehicle", "SetSteeringLinearity", 0x4CB570, &C_Vehicle::SetSteeringLinearity);
    ReversibleHooks::Install("C_Vehicle", "SetFuel", 0x4CB6E0, &C_Vehicle::SetFuel);
    ReversibleHooks::Install("C_Vehicle", "SetHandbrake", 0x4CB710, &C_Vehicle::SetHandbrake);
    ReversibleHooks::Install("C_Vehicle", "EnableSounds", 0x4CB780, &C_Vehicle::EnableSounds);
}
