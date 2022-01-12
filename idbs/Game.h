/*
   This file has been generated by IDA.
   It contains local type definitions from
   the type library 'Game'
*/

#define __int8 char
#define __int16 short
#define __int32 int
#define __int64 long long

/* 18 */
struct S_vector
{
  float x;
  float y;
  float z;
};

/* 39 */
struct C_Vehicle
{
  uint8_t _pad1[100];
  uint16_t m_iMoveFramesCnt;
  uint16_t m_iCurrentMoveFrame;
  S_vector m_aMovPosition;
  S_vector m_aMovForward;
  S_vector m_aMovUp;
  S_vector m_aMovRight;
  __int16 _pad2[20];
  int m_iLockCount;
  char _pad2_1[212];
  float m_fSteeringLinearity;
  float m_fClutchLinearity;
  uint8_t _pad3[4];
  uint32_t m_uFlags;
  uint8_t _pad4[12];
  float m_fEngineHealth;
  uint8_t _pad5[52];
  float m_fHealth;
  uint8_t _pad6[40];
  void *m_pFirstMesh;
  void *m_pLastMesh;
  uint8_t _pad7[196];
  void *m_pCallbackCC;
  void *m_pCallbackCC1;
  void *m_pCallbackACI;
  void *m_pCallbackAF;
  void *m_pCallbackWC;
  void *m_pCallbackPD;
  void *m_pCallbackPDW;
  void *m_pCallbackBG;
  void *m_pCallbackUnk;
  void *m_pCallbackWU;
  void *m_pCallbackCFB;
  void *m_pCallbackCFW;
  void *m_pCallbackDVP;
  void *m_pCallbackVR;
  void *m_pCallbackLP;
  S_vector m_aPosition;
  uint8_t _pad8[132];
  float m_fMaxHandbrakeForce;
  float m_fHandbrakeMax;
  float m_fHandbrakeCurrent;
  uint8_t _pad9[12];
  float m_fBrakeMax;
  float m_fBrakeCurrent;
  uint8_t _pad10[80];
  S_vector m_aAngularVelocity;
  uint8_t _pad11[4];
  uint8_t m_bHorn;
  uint8_t m_bSiren;
  uint8_t m_SoundEnabled;
  char _pad12_f;
  float m_fHandbrake;
  uint8_t _pad12[108];
  float m_fSpeedLimit;
  uint8_t m_bDontInterpolateSteering;
  char _pad13;
  uint8_t m_bDontInterpolateBrake;
  uint8_t m_bDontInterpolateClutch;
  char _pad14[24];
  uint32_t m_iWheelCnt;
  float m_fDeltaUpdateTime;
  uint8_t _pad15[116];
  float m_fAccelerating;
  uint8_t _pad16[4];
  char m_fUnk0;
  char _pad16_2[7];
  float m_fEngineRpm;
  uint8_t _pad17[8];
  int32_t m_iLastGear;
  int32_t m_iGear;
  uint8_t _pad18[4];
  int32_t m_iMaxGear;
  int32_t m_iUnk1;
  float m_GearRatios[4];
  char _pad19[28];
  float m_fSpeed;
  char _pad20[24];
  float m_fBrake;
  char _pad21[36];
  float m_fClutch;
  uint8_t _pad22[4];
  int m_Punk0;
  uint8_t _pad22APol[36];
  float m_fMaxSteerAngle;
  char _pad23[16];
  float m_fSteerAngle;
  char _pad24[1532];
  float m_fTimePerMoveFrame;
  __int16 _pad25[2];
  uint8_t m_bIsEngineRunning;
  char _pad25_2[2];
  uint8_t m_bEngineOn;
  float m_fFuel;
  char _pad26[4];
  void **m_pWheels;
  S_vector m_aForward;
  S_vector m_aRight;
  S_vector m_aUp;
  char _pad27[4912];
  S_vector m_aSpeed;
  char field_1F9C[4];
  uint8_t _pad28[4];
  float m_fMaxFuel;
};

