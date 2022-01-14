#pragma once
#include "I3d_math.h"
#include <cstdint>

typedef uint32_t LS3D_CALLBACK_MESSAGE;
typedef uint32_t I3DENUMRET;
typedef uint32_t LS3D_RESULT;

class I3D_frame {
public:
	I3D_frame() {};
	~I3D_frame() {};

	virtual int __stdcall AddRef(void) {}
	virtual void __stdcall SetWorldPos(S_vector const&) {}
	virtual void __stdcall SetWorldPosDir(S_vector const&, S_vector const&, float) {}
	virtual void __stdcall SetDir(S_vector const&, float) {}
	virtual void __stdcall SetWorldDir(S_vector const&, float) {}
	virtual void __stdcall Tick(void) {};
	virtual void __stdcall Update(void) {}
	virtual void __stdcall SetCallback(uint32_t(*)(I3D_frame*, LS3D_CALLBACK_MESSAGE, uint32_t, uint32_t), uint32_t) {}
	virtual LS3D_RESULT __stdcall SetProperty(char const*) {}
	virtual void __stdcall SetOn(bool bOn) {}
	virtual void __stdcall SetName(char const* pName) {}
	virtual LS3D_RESULT __stdcall LinkTo(I3D_frame*, uint32_t) {}
	virtual I3D_frame* __stdcall GetChild(int idx) const {}
	virtual LS3D_RESULT __stdcall EnumFrames(I3DENUMRET(*)(I3D_frame*, uint32_t), uint32_t, uint32_t, char const*) const {}
	virtual I3D_frame* __stdcall FindChildFrame(char const*, uint32_t) const {}
	virtual LS3D_RESULT __stdcall Duplicate(I3D_frame* const) {}
	virtual void __stdcall ScalarDelete(uint32_t size) {}
	virtual void __stdcall nullsub_7() {}
	virtual void __stdcall nullsub_8() {}
	virtual int __stdcall LoadImageA(void) {}
	static void InitHooks();
private:
};