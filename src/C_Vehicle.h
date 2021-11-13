#pragma once
#include <cstdint>
#include <engine/I3d_math.h>

class C_Vehicle
{
public:
	S_vector* GetWheelCamPos(S_vector* outPos, int wheelIdx, S_vector* pos);
	bool SetGear(int32_t gear);
	bool SetSpeedLimit(float limit);
	bool SetClutch(float clutch);
	bool SetSteer(float steer);
	bool SetSteeringLinearity(float lin);
	bool SoundOff();
	bool SetFuel(float fuel);
	bool SetHandbrake(bool handbrake);
	bool EnableSounds(bool enable);
	
	static void InitHooks();
private:
   uint8_t _pad0[0x198];
   float m_fSteeringLinearity;			//0x198 - 0x19C
   float m_fClutchLinearity;			//0x19C - 0x1A0
   uint8_t _pad00[0x14];
   float m_fEngineHealth;
   uint8_t _pad1[0x34];
   float m_fHealth;
   uint8_t _pad2[0x28];
   void* m_pFirstMesh;
   void* m_pLastMesh;
   uint8_t _pad4[0x100];
   S_vector m_aPosition;
   uint8_t _pad5[0xF4];
   S_vector m_aAngularVelocity;
   uint8_t _pad6[0x4];
   uint8_t m_bHorn;
   uint8_t m_bSiren;
   uint8_t m_SoundEnabled;
   float m_fHandBreak;
   uint8_t _pad10[0x6C];
   float m_fSpeedLimit;					//0x4A4 - 0x4A8
   uint8_t m_bDontInterpolateSteering;	//0x4A8 - 0x4A9
   uint8_t _pad11[0x97];
   float m_fAccelerating;
   uint8_t _pad12[0x4];
   float m_fEngineRpm;
   uint8_t _pad13[0x14];
   int32_t m_iGear;
   uint8_t _m_pad23[4];
   int32_t m_iMaxGear; 
   uint8_t _pad14[0x4C];
   float m_fBreakValue;
   uint8_t _pad15[0x24];
   float m_fClutch;						//0x5E0 - 0x5E4
   uint8_t _pad16[0x2C];
   float m_fMaxSteerAngle;				//0x610 - 0x614
   uint8_t _pad161[0x10];
   float m_fSteerAngle;					//0x624 - 0x628
   uint8_t _pad17[0x604];
   uint8_t m_bEngineOn;
   float m_fFuel;
   uint8_t _pad19[0x4];
   void** m_pWheels;					//0xC38 - 0xC3C
   S_vector m_aForward;					//0xC3C - 0xC48
   S_vector m_aRight;					//0xC48 - 0xC54
   S_vector m_aUp;						//0xC54 - 0xC60
   uint8_t _pad22[0x1330];					
   S_vector m_aSpeed;
   uint8_t _pad23[4];
   float m_fMaxFuel;					//0x1FA0 - 0x1FA4
};

//static_assert(sizeof(C_Vehicle) == 0x21AC);