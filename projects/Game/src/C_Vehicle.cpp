#include "C_Vehicle.h"

#include <cstdint>
#include <cstdio>

#include <Windows.h>
#include <cstdio>
#include <math.h>
#include <algorithm>

#include <reversiblehooks/ReversibleHooks.h>
#include "plugin.h"

#include "I3d_sound.h"

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

static C_Vehicle* initedFrom = nullptr;
static bool skipLoop = false;

bool C_Vehicle::Engine(float deltaTime, float a3, float a4)
{
     if (!skipLoop && initedFrom == this) {
         skipLoop = true;
         testVehicle->Engine(deltaTime, a3, a4);
         skipLoop = false;
         printf("pos: %f %f %f\n", testVehicle->m_aPosition.x, testVehicle->m_aPosition.y, testVehicle->m_aPosition.z);
     }


    //NOTE: this looks like its not vector  
    //just used as container for 3 floats
    S_vector v42(0.0f, 0.0f, 0.0f);
    if (m_fSpeed > 0.0f)
    {
        float speed2 = (m_fSpeed * m_fSpeed);
        const float magic = 0.64999998f;
        v42.z = *(float*)((uint32_t)this + 0x53C) * *(float*)((uint32_t)this + 0x538) * speed2 * magic;
        v42.y = *(float*)((uint32_t)this + 0x3D4) * *(float*)((uint32_t)this + 0x3D0) * speed2 * magic;
        v42.x = *(float*)((uint32_t)this + 0x3DC) * *(float*)((uint32_t)this + 0x3D8) * speed2 * magic;
    }

    if (m_fBrake == 0.0f)
        m_fBrakeCurrent = 0.0f;
   
    *(uint32_t*)((uint32_t)this + 0x5B4) = 0;
    S_vector vForward = m_aForward;

    memcpy((void*)((uint32_t)this + 0xDE8), (const void*)((uint32_t)this + 0xCE8), 0x40u);
    memcpy((void*)((uint32_t)this + 0xE28), (const void*)((uint32_t)this + 0xD28), 0x40u);

    m_aMovPosition = m_aPosition;
    m_aMovForward = m_aForward;
    m_aMovUp = m_aUp;
    m_aMovRight = m_aRight;

    bool isSingleMove = deltaTime <= m_fTimePerMoveFrame;
    if (isSingleMove)
    {
        m_iMoveFramesCnt = 1;
        m_iCurrentMoveFrame = 0;
        this->Move(deltaTime, v42.z, v42.y, v42.x, a3, a4);
    }
    else
    {
        m_iMoveFramesCnt = (uint16_t)(deltaTime / m_fTimePerMoveFrame) + 1;
        m_iCurrentMoveFrame = 0;
        float deltaTimePerMove = deltaTime / m_iMoveFramesCnt;
        for (uint16_t i = 0; i < m_iMoveFramesCnt; i++)
        {
            this->Move(deltaTimePerMove, v42.z, v42.y, v42.x, a3, a4);
        }
    }
    
    //NOTE: adds something to every wheel
    for (uint32_t i = 0; i < m_iWheelCnt; i++)
    {
        uint32_t currentWheel = (uint32_t)m_pWheels[i];
        *(float*)((uint32_t)currentWheel + 0x1C8) = *(float*)((uint32_t)currentWheel + 0x1C8) + *(float*)((uint32_t)this + 0x5B4);
    }
    
    float v24 = vForward.z * m_aUp.y - vForward.y * m_aUp.z;
    float v25 = vForward.x * m_aUp.z - vForward.z * m_aUp.x;
    float v26 = vForward.y * m_aUp.x - vForward.x * m_aUp.y;
    vForward.x = v25 * m_aUp.z - v26 * m_aUp.y;
    vForward.y = v26 * m_aUp.x - v24 * m_aUp.z;
    vForward.z = v24 * m_aUp.y - v25 * m_aUp.x;
    
    float angle = (float)vForward.AngleTo(m_aForward);
  
    *(float*)((uint32_t)this + 0x350) = angle;

    if (isSingleMove)
        *(float*)((uint32_t)this + 0x350) = *(float*)((uint32_t)this + 0x5B4) / angle;

    float v34 = *(float*)((uint32_t)this + 0x350);
    if (v34 <= 0.0f && *(float*)((uint32_t)this + 0x350) < (double)*(float*)((uint32_t)this + 0x354))
    {
        int wheelCnt = m_iWheelCnt;
        if (wheelCnt)
        {
            int32_t i = m_iWheelCnt;
            while (true)
            {
                if ((*(uint8_t*)((uint32_t)m_pWheels[i - 1] + 0x120) & 0x90) != 0)
                    break;
               
                if (!(--i))
                {
                    *(float*)((uint32_t)this + 0x354) = *(float*)((uint32_t)this + 0x350);
                    break;
                }
            }
        }
        else
        {
            *(float*)((uint32_t)this + 0x354) = *(float*)((uint32_t)this + 0x350);
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
   
    m_fDeltaUpdateTime = deltaTime;
    return true;
}

bool C_Vehicle::Move(float a1, float a2, float a3, float a4, float a5, float a6)
{
    return plugin::CallMethodAndReturn<bool, 0x4E2090>(this, a1, a2, a3, a4, a5, a6);
}

bool C_Vehicle::SetGear(int32_t iGear)
{
    if (iGear >= -1 && iGear <= m_iMaxGear)
    {
        if (m_iGear == iGear)
            return true;

        uint32_t v1 = *(uint32_t*)((uint32_t)this + 0xB8);
        if (v1 != 1 || iGear >= 0)
        {
            float v3 = *(float*)((uint32_t)this + 0x1EC);
            if ((*(uint32_t*)((uint32_t)this + 0xC68) & 0x4000) == 0 || v3 < 0.0f)
            {
                if ((*(uint32_t*)((uint32_t)this + 0xC68) & 0x4000) != 0)
                    return true;
            }

            float v8 = m_GearRatios[iGear] /
                       m_GearRatios[m_iLastGear] * *(float*)((uint32_t)this + 0x548);

            float v9 = *(float*)((uint32_t)this + 0x1F9C);
            if (iGear == -1 || v8 <= v9)
            {
                uint32_t v12 = *(uint32_t*)((uint32_t)this + 0x5E8);
                *(uint32_t*)((uint32_t)this + 0x494) = 0;
                if ((v12 & 0x20) != 0)
                {
                    v12 = v12 & 0xFC | 2;
                    *(uint32_t*)((uint32_t)this + 0x5E8) = v12;
                }

                m_iGear = iGear;
                return 1;
            }
        }
    }

    return 0;
}

bool C_Vehicle::SetBrake(float fBrake)
{
    if (fBrake > 1.0f || fBrake < 0.0f || *(uint32_t*)((uint32_t)this + 0xB0) == 1)
        return false;

    if (!*(uint8_t*)((uint32_t)this + 0x4CC))
    {
        if ( m_bDontInterpolateBrake )
        {
            m_fBrake = fBrake;
        }
        else
        {
            m_fBrakeCurrent = m_fDeltaUpdateTime + m_fBrakeCurrent;
              
            if (m_fBrakeCurrent > m_fBrakeMax)
                m_fBrakeCurrent = m_fBrakeMax;

            m_fBrake = m_fBrakeCurrent / m_fBrakeMax * fBrake;
        }

        if (fBrake > 0.0f)
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
            *(float*)((uint32_t)this + 0x3A4) = fBrake;
        return true;
    }

    if (m_bDontInterpolateBrake)
    {
        m_fBrake = fBrake;
    }
    else
    {
        m_fBrakeCurrent = m_fDeltaUpdateTime + m_fBrakeCurrent;
        
        if (m_fBrakeCurrent > m_fBrakeMax)
            m_fBrakeCurrent = m_fBrakeMax;

        m_fBrake = m_fBrakeCurrent / m_fBrakeMax * fBrake;
    }

 
    if (fBrake > 0.0f)
    {
        *(uint32_t*)((uint32_t)this + 0x3A8) = 0;
        *(uint32_t*)((uint32_t)this + 0x3A4) = 0;
    }

    m_uFlags &= ~0x1000000u;
    return true;
}

bool C_Vehicle::SetSpeedLimit(float fLimit)
{
    m_fSpeedLimit = fLimit;

    if(m_fSpeedLimit > 0.0f)
        m_fSpeedLimit = fLimit + 0.5f;
    return true;
}

bool C_Vehicle::SetClutch(float fClutch)
{
    fClutch = std::clamp(fClutch, 0.0f, 1.0f);
    
    if ( m_bDontInterpolateSteering )
    {
        m_fClutch = ((1.0f - fClutch) * m_fClutchLinearity + fClutch) * fClutch;
    }
    else
    {
        m_fClutch = fClutch;
    }

    return true;
}

bool C_Vehicle::SetSteer(float fSteer)
{
    float steerChange = fSteer;
    if ( m_bDontInterpolateSteering )
    {
        if (fSteer > 0.0f )
        {
            steerChange = ((1.0f - fSteer) * m_fSteeringLinearity + fSteer) * fSteer;
        }
        else
        {
            steerChange = -((fSteer - (fSteer + 1.0f) * m_fSteeringLinearity) * fSteer);
        }
    }

    m_fSteerAngle = steerChange * m_fMaxSteerAngle;
    return true;
}

bool C_Vehicle::SetSteeringLinearity(float fLin)
{
    if (fLin >= 0.0f )
    {
        if (fLin <= 1.0f )
        {
            m_fSteeringLinearity = fLin;
            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

bool C_Vehicle::SetFuel(float fFuel)
{   
    if (fFuel > m_fMaxFuel)
        fFuel = m_fMaxFuel;
    
    m_fFuel = fFuel;
    return true;
}

bool C_Vehicle::SetHandbrake(bool bBrake)
{
    if (bBrake)
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

bool C_Vehicle::EnableSounds(bool bEnable)
{
    bool bResult = false;

    if (!bEnable)
    {
        bResult = m_bSoundEnabled;
        if (bResult)
            bResult = SoundOff();
    }

    m_bSoundEnabled = bEnable;
    return bResult;
}

bool C_Vehicle::UpdateSteeringWheels(float fDeltaTime)
{
    for (size_t i = 0; i < m_iWheelCnt; i++) {
        auto* wheel = m_pWheels[i];
        printf("%s\n", wheel->m_pFrame->GetName());
    }
    return false;
}

bool C_Vehicle::SoundOff()
{    
    if (m_pEngineOnSound)
        m_pEngineOnSound->SetOn(false, true);

    if (m_pEngineOffSound)
        m_pEngineOffSound->SetOn(false, true);

    if (m_pEngineBadSound)
        m_pEngineBadSound->SetOn(false, true);

    if (m_pEngineNpcSound)
        m_pEngineNpcSound->SetOn(false, true);

    for (size_t i = 0; i < 5; i++)
    {
        if (m_pEngineForwardSounds[i])
            m_pEngineForwardSounds[i]->SetOn(false, true);

        if (m_pEngineReverseSounds[i])
            m_pEngineReverseSounds[i]->SetOn(false, true);
    }

    for (size_t i = 0; i < 2; i++) 
    {
        if (m_pEngineIdleSounds[i])
            m_pEngineIdleSounds[i]->SetOn(false, true);
    }

    if (m_pHornSound)
        m_pHornSound->SetOn(false, true);

    if (m_pSirenSound)
        m_pSirenSound->SetOn(false, true);

    if (m_pHandbrakeSound)
        m_pHandbrakeSound->SetOn(false, true);

    if (m_pGearNextSound)
        m_pGearNextSound->SetOn(false, true);

    if (m_pGearPrevSound)
        m_pGearPrevSound->SetOn(false, true);

    if (m_pDriftSound)
        m_pDriftSound->SetOn(false, true);

    if (m_pUnkSound)
        m_pUnkSound->SetOn(false, true);

    if (m_pCrashAbsorberSound)
        m_pCrashAbsorberSound->SetOn(false, true);

    if (m_pShotInWheelSound)
        m_pShotInWheelSound->SetOn(false, true);

    if (m_pCrashA1Sound)
        m_pCrashA1Sound->SetOn(false, true);

    if (m_pCrashK1Sound)
        m_pCrashK1Sound->SetOn(false, true);

    if (m_pCrashB1Sound)
        m_pCrashB1Sound->SetOn(false, true);

    if (m_pCrashC1Sound)
        m_pCrashC1Sound->SetOn(false, true);

    if (m_pCrashA2Sound)
        m_pCrashA2Sound->SetOn(false, true);

    if (m_pCrashK2Sound)
        m_pCrashK2Sound->SetOn(false, true);
    
    if (m_pCrashB2Sound)
        m_pCrashB2Sound->SetOn(false, true);

    if (m_pCrashC2Sound)
        m_pCrashC2Sound->SetOn(false, true);

    if (m_pDoorOpenSound)
        m_pDoorOpenSound->SetOn(false, true);

    if (m_pDoorCloseSound)
        m_pDoorCloseSound->SetOn(false, true);

    if (m_pWheelPunctureSound)
        m_pWheelPunctureSound->SetOn(false, true);

    return true;
}

I3D_sound* C_Vehicle::SoundInit(
    const char* szSound, 
    I3D_sound** pSoundOut, 
    uint32_t uType, 
    float fVal1, float fVal2, 
    float fVal3, float fVal4, 
    bool bDoRepeat, bool b2)
{
    return plugin::CallMethodAndReturn<I3D_sound*, 0x004EF110>(this, szSound, pSoundOut, uType, fVal1, fVal2, fVal3, fVal4, bDoRepeat, b2);
}

bool C_Vehicle::InitSounds(S_CARINIT* init)
{
    SoundInit(init->m_szSoundEngineOn, &m_pEngineOnSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, true);
    SoundInit(init->m_szSoundEngineOff, &m_pEngineOffSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, true);
    SoundInit(init->m_szSoundEngineBad, &m_pEngineBadSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, true);
    SoundInit(init->m_szSoundEngineNpc, &m_pEngineNpcSound, 8, 4.0f, 50.0f, 0.3f, 0.2f, true, true);

    for (size_t i = 0; i < 5; i++) 
    {
        SoundInit(init->m_szSoundEngineForward[i], &m_pEngineForwardSounds[i], 8, 4.0f, 50.0f, 0.3f, 0.2f, true, true);
        SoundInit(init->m_szSoundEngineReverse[i], &m_pEngineReverseSounds[i], 8, 4.0f, 50.0f, 0.3f, 0.2f, true, true);
    }

    SoundInit(init->m_szSoundEngineIdle[0], &m_pEngineIdleSounds[0], 0, 4.0f, 50.0f, 0.3f, 0.2f, true, true);
    SoundInit(init->m_szSoundEngineIdle[1], &m_pEngineIdleSounds[1], 0, 4.0f, 50.0f, 0.3f, 0.2f, true, false);
    SoundInit(init->m_szSoundHorn, &m_pHornSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, true, false);
    SoundInit(init->m_szSoundSiren, &m_pSirenSound, 0, 4.0f, 110.0f, 0.45f, 0.37f, true, false);
    SoundInit(init->m_szSoundHandbrake, &m_pHandbrakeSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundGearNext, &m_pGearNextSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundGearPrev, &m_pGearPrevSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundDrift, &m_pDriftSound, 8, 4.0f, 50.0f, 0.3f, 0.2f, true, false);
    SoundInit(init->m_szSoundUnk, &m_pUnkSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, true, false);
    SoundInit(init->m_szSoundCrashAbsorber, &m_pCrashAbsorberSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundShotInWheel, &m_pShotInWheelSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashA1, &m_pCrashA1Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashK1, &m_pCrashK1Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashB1, &m_pCrashB1Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashC1, &m_pCrashC1Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashA2, &m_pCrashA2Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashK2, &m_pCrashK2Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashB2, &m_pCrashB2Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundCrashC2, &m_pCrashC2Sound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundDoorOpen, &m_pDoorOpenSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit(init->m_szSoundDoorClose, &m_pDoorCloseSound, 0, 4.0f, 50.0f, 0.3f, 0.2f, false, false);
    SoundInit("PNEU_PUNC.WAV", &m_pWheelPunctureSound, 8, 6.0f, 30.0f, 0.3f, 0.2f, true, false);
    return 0;
}

int C_Vehicle::LockVehicle(bool bLock)
{     
    if (bLock)
        m_iLockCount++;
    else
        m_iLockCount--;

    if (m_iLockCount <= 0)
        m_uFlags = m_uFlags & 0xEFFFFFFF;
    else
        m_uFlags = m_uFlags | 0x10000000;

    return m_iLockCount;
}

int C_Vehicle::HornSnd(bool bHorn)
{
    if (bHorn)
    {
        I3D_sound* pMuteSound = nullptr;

        //NOTE: whats this ?
        if (*((uint8_t*)(uint32_t)this + 0x491))
        {
            if (m_pHornSound2 != nullptr && !m_pHornSound2->IsPlaying())
            {
                m_pHornSound2->SetOn(true);
                m_pHornSound2->SetVolume(1.0f);
            }
            
            pMuteSound = m_pHornSound;
        }
        else
        {
            if (m_pHornSound != nullptr && !m_pHornSound->IsPlaying())
            {
                m_pHornSound->SetOn(true);
                m_pHornSound->SetVolume(1.0f);
            }
            
            pMuteSound = m_pHornSound2;
        }

        if (pMuteSound)
            pMuteSound->SetOn(false);
    }
    else
    {
        if (m_pHornSound2 != nullptr)
            m_pHornSound2->SetOn(false);
       
        if (m_pHornSound != nullptr)
            m_pHornSound->SetOn(false);
    }

    return 0;
}

void C_Vehicle::InitHooks()
{
    ReversibleHooks::Install("C_Vehicle", "InitSounds",             0x4EAC70, &C_Vehicle::InitSounds);
    ReversibleHooks::Install("C_Vehicle", "HornSnd",                0x4EDA40, &C_Vehicle::HornSnd);

    ReversibleHooks::Install("C_Vehicle", "UpdateSteeringWheels", 0x4DDC60, &C_Vehicle::UpdateSteeringWheels);

    ReversibleHooks::Install("C_Vehicle", "LockVehicle",            0x4CD600, &C_Vehicle::LockVehicle);
    ReversibleHooks::Install("C_Vehicle", "Engine",                 0x4E1CE0, &C_Vehicle::Engine);
    ReversibleHooks::Install("C_Vehicle", "SetBrake",               0x4CB2D0, &C_Vehicle::SetBrake);
    ReversibleHooks::Install("C_Vehicle", "SetGear",                0x4CB070, &C_Vehicle::SetGear);
    ReversibleHooks::Install("C_Vehicle", "GetWheelCamPos",         0x4C6010, &C_Vehicle::GetWheelCamPos);
    ReversibleHooks::Install("C_Vehicle", "SetSpeedLimit",          0x4CB6A0, &C_Vehicle::SetSpeedLimit);
    ReversibleHooks::Install("C_Vehicle", "SetClutch",              0x4CB460, &C_Vehicle::SetClutch);
    ReversibleHooks::Install("C_Vehicle", "SetSteer",               0x4CB4D0, &C_Vehicle::SetSteer);
    ReversibleHooks::Install("C_Vehicle", "SetSteeringLinearity",   0x4CB570, &C_Vehicle::SetSteeringLinearity);
    ReversibleHooks::Install("C_Vehicle", "SetFuel",                0x4CB6E0, &C_Vehicle::SetFuel);
    ReversibleHooks::Install("C_Vehicle", "SetHandbrake",           0x4CB710, &C_Vehicle::SetHandbrake);
    ReversibleHooks::Install("C_Vehicle", "EnableSounds",           0x4CB780, &C_Vehicle::EnableSounds);
}