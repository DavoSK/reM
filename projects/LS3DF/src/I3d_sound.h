#pragma once
#include "I3d_frame.h"

typedef uint32_t I3D_SOUNDTYPE;

class I3D_sound : public I3D_frame {
public:
	virtual void __stdcall Open(char const*, uint32_t, void (*)(LS3D_CALLBACK_MESSAGE, uint32_t, uint32_t, void*), void*) {}
	virtual bool __stdcall IsPlaying(bool bPlaying = false) const { return false; }
	virtual void __stdcall SetSoundType(I3D_SOUNDTYPE) {}
	virtual void __stdcall SetRange(float, float, float, float) {}
	virtual void __stdcall SetCone(float, float) {}
	virtual void __stdcall SetOutVol(float) {}
	virtual void __stdcall SetVolume(float) {}
	virtual void __stdcall SetLoop(bool) {}
	virtual void __stdcall SetCurrTime(uint32_t) {}
	virtual void __stdcall SetFrequency(float) {}
	virtual void __stdcall GetRange(float&, float&, float&, float&) {}
	virtual void __stdcall GetCone(float&, float&) {}
	virtual uint32_t __stdcall GetCurrTime(void) {}
	virtual uint32_t __stdcall GetPlayTime(void) {}
	virtual void __stdcall SetOn(bool bOn) override { SetOn(bOn, false); }
	virtual void __stdcall SetOn(bool arg1, bool arg2) {}
	virtual void __stdcall SetVelocity(S_vector) {}
};