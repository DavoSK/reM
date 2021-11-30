#pragma once
#include <cstdint>
#include <I3d_math.h>

class C_Vehicle
{
public:
	S_vector* GetWheelCamPos(S_vector* outPos, int wheelIdx, S_vector* pos);
	bool Engine(float a1, float a2, float a3);
	bool Move(float a1, float a2, float a3, float a4, float a5, float a6);
	bool SetGear(int32_t gear);
	bool SetBrake(float brake);
	bool SetSpeedLimit(float limit);
	bool SetClutch(float clutch);
	bool SetSteer(float steer);
	bool SetSteeringLinearity(float lin);
	bool SoundOff();
	bool SetFuel(float fuel);
	bool SetHandbrake(bool doBrake);
	bool EnableSounds(bool enable);
	static void InitHooks();
private:
   uint8_t _pad0[0x64];
   uint16_t	m_iMoveFramesCnt; 			//0x64  -  0x66
   uint16_t m_iCurrentMoveFrame;		//0x66  -  0x68
   S_vector m_aMovPosition;				//0x68	-  0x74
   S_vector m_aMovForward;				//0x74	-  0x80
   S_vector m_aMovUp;					//0x80	-  0x8C
   S_vector m_aMovRight;				//0x8C	-  0x98
   uint8_t _pad003[0x100];				
   float m_fSteeringLinearity;			//0x198 - 0x19C
   float m_fClutchLinearity;			//0x19C - 0x1A0
   uint8_t _pad0001[0x4];				//0x1A0 - 0x1A4
   uint32_t m_uFlags;					//0x1A4 - 0x1A8
   uint8_t _pad00[0xC];
   float m_fEngineHealth;
   uint8_t _pad1[0x34];
   float m_fHealth;
   uint8_t _pad2[0x28];
   void* m_pFirstMesh;
   void* m_pLastMesh;
   uint8_t _pad4[0x100];
   S_vector m_aPosition;				//0x320	- 0x32C
   uint8_t _pad5[0x84];					//0x32C - 0x3B0
   float m_fMaxHandbrakeForce;			//0x3B0 - 0x3B4
   float m_fHandbrakeMax;				//0x3B4 - 0x3B8
   float m_fHandbrakeCurrent;			//0x3B8 - 0x3BC
   uint8_t _padxx[0xC];					//0x3BC - 0x3C8
   float m_fBrakeMax;					//0x3C8 - 0x3CC 
   float m_fBrakeCurrent;				//0x3CC - 0x3D0
   uint8_t _padx1[0x50];				//0x3D0 - 0x420
   S_vector m_aAngularVelocity;			//0x420 - 0x42C
   uint8_t _pad6[0x4];
   uint8_t m_bHorn;
   uint8_t m_bSiren;
   uint8_t m_SoundEnabled;
   float m_fHandbrake;					//0x434 - 0x438
   uint8_t _pad10[0x6C];				//0x438	- 0x4A4
   float m_fSpeedLimit;					//0x4A4 - 0x4A8
   uint8_t m_bDontInterpolateSteering;	//0x4A8 - 0x4A9
   uint8_t _pad11x;						//0x4A9 - 0x4AA 
   uint8_t m_bDontInterpolateBrake;		//0x4AA - 0x4AB
   uint8_t _pad11[0x19];				//0x4AB - 0x4C4
   uint32_t m_iWheelCnt;				//0x4C4 - 0x4C8
   float m_fDeltaUpdateTime;			//0x4C8 - 0x4CC
   uint8_t _pad12[0x74];				//0x4CC - 0x540
   float m_fAccelerating;				//0x540 - 0x544
   uint8_t _pad13[0xC];					//0x544 - 0x54C
   float m_fEngineRpm;					//0x550	- 0x554
   uint8_t _pad14[0x8];					//0x554 - 0x55C
   int32_t m_iLastGear;					//0x55C - 0x560
   int32_t m_iGear;						//0x560 - 0x564
   uint8_t _m_pad23[4];					//0x564 - 0x568
   int32_t m_iMaxGear;					//0x568 - 0x56C
   int32_t m_iUnk1;						//0x56C - 0x570
   float m_GearRatios[4];				//0x570 - 0x57C
   uint8_t _pad15[0x1C];				//0x57C - 0x59C
   float m_fSpeed;						//0x59C - 0x5A0
   uint8_t _pad167[0x18];				//0x5A0 - 0x5B8
   float m_fBrake;						//0x5B8 - 0x5BC
   uint8_t _pad16[0x24];				//0x5BC - 0x5E0
   float m_fClutch;						//0x5E0 - 0x5E4
   uint8_t _pad17[0x2C];				//0x5E4 - 0x610
   float m_fMaxSteerAngle;				//0x610 - 0x614
   uint8_t _pad161[0x10];				//0x614 - 0x624
   float m_fSteerAngle;					//0x624 - 0x628
   uint8_t _pad18[0x5FC];
   float m_fTimePerMoveFrame;			//0xC24 - 0xC28
   uint8_t _pad21[7];
   uint8_t m_bEngineOn;					//0xC2F - 0xC30
   float m_fFuel;						//0xC30 - 0xC34
   uint8_t _pad19[0x4];					//0xC34 - 0xC38
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