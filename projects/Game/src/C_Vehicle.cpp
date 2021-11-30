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

bool C_Vehicle::Engine(float a2, float a3, float a4)
{
    __int16 v12; // fps
    //float* v16; // edi
    int v17; // edx
    int v18; // eax
    int v19; // eax
    int v21; // esi
    int i; // eax
    int v23; // ecx
    double v24; // st7
    double v25; // st6
    double v26; // st5
    double v27; // st5
    double v28; // st4
    double v29; // rt1
    double v30; // st6
    double v31; // st7
    bool v35; // c0
    char v36; // c2
    bool v37; // c3
    int v38; // eax
    int v39; // ecx
    int v40; // edx
    
    float v45; // [esp+10h] [ebp-1Ch]
    float v49; // [esp+20h] [ebp-Ch]
    float v50; // [esp+24h] [ebp-8h]
    float v51; // [esp+28h] [ebp-4h]

    ///constexpr auto test = offsetof(C_Vehicle, m_fWIP_Accel);
    S_vector v42(0.0f, 0.0f, 0.0f);
    if (m_fSpeed > 0.0f)
    {
        float speedPow2 = m_fSpeed * m_fSpeed;
        v42.z = *(float*)((uint32_t)this + 0x53C) * *(float*)((uint32_t)this + 0x538) * speedPow2 * 0.64999998;
        v42.y = *(float*)((uint32_t)this + 0x3D4) * *(float*)((uint32_t)this + 0x3D0) * speedPow2 * 0.64999998;
        v42.x = *(float*)((uint32_t)this + 0x3DC) * *(float*)((uint32_t)this + 0x3D8) * speedPow2 * 0.64999998;
    }

    if (*(float*)((uint32_t)this + 0x5B8) == 0.0f)
        *(uint32_t*)((uint32_t)this + 0x3CC) = 0;
   
    *(uint32_t*)((uint32_t)this + 0x5B4) = 0;
    
    S_vector vForward = m_aForward;

    memcpy((void*)((uint32_t)this + 0xDE8), (const void*)((uint32_t)this + 0xCE8), 0x40u);
    memcpy((void*)((uint32_t)this + 0xE28), (const void*)((uint32_t)this + 0xD28), 0x40u);

    //v16 = (float*)((uint32_t)this + 0xC54);
    *(uint32_t*)((uint32_t)this + 0x68) = *(uint32_t*)((uint32_t)this + 0x320);
    v17 = *(uint32_t*)((uint32_t)this + 808);
    *(uint32_t*)((uint32_t)this + 0x6C) = *(uint32_t*)((uint32_t)this + 0x324);
    *(uint32_t*)((uint32_t)this + 0x70) = v17;
    *(uint32_t*)((uint32_t)this + 0x74) = *(uint32_t*)((uint32_t)this + 0xC3C);
    v18 = *(uint32_t*)((uint32_t)this + 0xC44);
    *(uint32_t*)((uint32_t)this + 0x78) = *(uint32_t*)((uint32_t)this + 3136);
    *(uint32_t*)((uint32_t)this + 0x7C) = v18;
    *(uint32_t*)((uint32_t)this + 0x80) = m_aUp.x;
    *(uint32_t*)((uint32_t)this + 0x84) = m_aUp.y;
    *(uint32_t*)((uint32_t)this + 0x88) = m_aUp.z;
    *(uint32_t*)((uint32_t)this + 0x8C) = *(uint32_t*)((uint32_t)this + 0xC48);
    v19 = *(uint32_t*)((uint32_t)this + 0xC50);
    *(uint32_t*)((uint32_t)this + 0x90) = *(uint32_t*)((uint32_t)this + 0xC4C);
    *(uint32_t*)((uint32_t)this + 0x94) = v19;
    
    bool singleMove = a2 <= *(float*)((uint32_t)this + 0xC24);
    if (singleMove)
    {
        *(uint16_t*)((uint32_t)this + 0x64) = 1;
        *(uint16_t*)((uint32_t)this + 0x66) = 0;
        this->Move(a2, v42.z, v42.y, v42.x, a3, a4);
    }
    else
    {
        uint32_t v20 = (uint32_t)(a2 / *(float*)((uint32_t)this + 0xC24)) + 1;
        *(uint16_t*)((uint32_t)this + 0x66) = 0;
        *(uint16_t*)((uint32_t)this + 0x64) = v20;
        v21 = 0;
        v45 = a2 / (double)v20;
        if (v20)
        {
            do
            {
                this->Move(v45, v42.z, v42.y, v42.x, a3, a4);
                ++v21;
            }       while (v21 < *(unsigned __int16*)((uint32_t)this + 0x64));
        }
    }
    
    for (i = *(uint32_t*)((uint32_t)this + 0x4C4); i; *(float*)((uint32_t)v23 + 0x1C8) = *(float*)((uint32_t)v23 + 0x1C8) + *(float*)((uint32_t)this + 0x5B4))
    {
        --i;
        v23 = *(uint32_t*)(*(uint32_t*)((uint32_t)this + 0xC38) + 4 * i);
    }
    
    v24 = vForward.z * m_aUp.y - vForward.y * m_aUp.z;
    v25 = vForward.x * m_aUp.z - vForward.z * m_aUp.x;
    v26 = vForward.y * m_aUp.x - vForward.x * m_aUp.y;
    v49 = v25 * m_aUp.z - v26 * m_aUp.y;
    v27 = v26 * m_aUp.x;
    v28 = v24 * m_aUp.z;
    vForward.x = v49;
    v50 = v27 - v28;
    v29 = v24 * m_aUp.y;
    v30 = v25 * m_aUp.x;
    vForward.y = v50;
    v51 = v29 - v30;
    vForward.z = v51;
    
    v31 = vForward.AngleTo(*(const S_vector*)((uint32_t)this + 0xC3C));
    *(float*)((uint32_t)this + 0x350) = v31;

    if (singleMove)
        *(float*)((uint32_t)this + 0x350) = *(float*)((uint32_t)this + 0x5B4) / v31;

    float v34 = *(float*)((uint32_t)this + 0x350);
    if (v34 <= 0.0f && *(float*)((uint32_t)this + 0x350) < (double)*(float*)((uint32_t)this + 0x354))
    {
        v38 = *(uint32_t*)((uint32_t)this + 0x4C4);
        if (v38)
        {
            v39 = *(uint32_t*)((uint32_t)this + 0xC38) + 4 * v38;
            while (1)
            {
                v40 = *(uint32_t*)(v39 - 4);
                v39 -= 4;
                --v38;
                if ((*(uint8_t*)(v40 + 0x120) & 0x90) != 0)
                    break;
                if (!v38)
                    goto LABEL_21;
            }
        }
        else
        {
        LABEL_21:
            *(uint32_t*)((uint32_t)this + 0x354) = *(uint32_t*)((uint32_t)this + 0x350);
        }
    }

    if (*(float*)((uint32_t)this + 0x628) != 0.0f
        && *(float*)((uint32_t)this + 0x59C) < 2.0f
        && fabs(*(float*)((uint32_t)this + 0x350)) < 0.001000000047497451f)
    {
        *(uint32_t*)((uint32_t)this + 0x350) = *(uint32_t*)((uint32_t)this + 0x354);
    }
    
    if (*(float*)((uint32_t)this + 0x628) < 0.0f)
        *(float*)((uint32_t)this + 0x350) = -*(float*)((uint32_t)this + 0x350);
   
    *(float*)((uint32_t)this + 0x4C8) = a2;
    return true;
}

bool C_Vehicle::Move(float a1, float a2, float a3, float a4, float a5, float a6)
{
    return plugin::CallMethodAndReturn<bool, 0x4E2090>(this, a1, a2, a3, a4, a5, a6);
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
    ReversibleHooks::Install("C_Vehicle", "Engine", 0x4E1CE0, &C_Vehicle::Engine);
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