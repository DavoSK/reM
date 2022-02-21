#pragma once
#include "I3d_math.h"
#include "Common.h"

#include <cstdint>

typedef uint32_t LS3D_CALLBACK_MESSAGE;
typedef uint32_t I3DENUMRET;

#pragma pack(push,1)

struct I3D_bbox
{
	S_vector m_aMin;
	S_vector m_aMax;
};

struct I3D_bsphere
{
	S_vector m_aCenter;
	float m_fRadius;
};

enum I3D_FRAME_TYPE {
	FRAME_NULL, FRAME_VISUAL, FRAME_LIGHT, FRAME_CAMERA,
	FRAME_SOUND, FRAME_SECTOR, FRAME_DUMMY, FRAME_reserved,
	FRAME_USER, FRAME_MODEL, FRAME_JOINT, FRAME_VOLUME,
	FRAME_OCCLUDER,
	FRAME_LAST,
};

class I3D_frame 
{
public:
	I3D_frame() {};
	~I3D_frame() {};

	virtual int			__stdcall AddRef(void) {}
	virtual void		__stdcall SetWorldPos(S_vector const&) {}
	virtual void		__stdcall SetWorldPosDir(S_vector const&, S_vector const&, float) {}
	virtual void		__stdcall SetDir(S_vector const&, float) {}
	virtual void		__stdcall SetWorldDir(S_vector const&, float) {}
	virtual void		__stdcall Tick(void) {};
	virtual void		__stdcall Update(void) {}
	virtual void		__stdcall SetCallback(uint32_t(*)(I3D_frame*, LS3D_CALLBACK_MESSAGE, uint32_t, uint32_t), uint32_t) {}
	virtual LS3D_RESULT __stdcall SetProperty(char const*) {}
	virtual void		__stdcall SetOn(bool bOn) {}
	virtual void		__stdcall SetName(char const* pName) {}
	virtual LS3D_RESULT __stdcall LinkTo(I3D_frame*, uint32_t) {}
	virtual I3D_frame*	__stdcall GetChild(int idx) const {}
	virtual LS3D_RESULT __stdcall EnumFrames(I3DENUMRET(*)(I3D_frame*, uint32_t), uint32_t, uint32_t, char const*) const {}
	virtual I3D_frame*	__stdcall FindChildFrame(char const*, uint32_t) const {}
	virtual LS3D_RESULT __stdcall Duplicate(I3D_frame* const) {}
	virtual void		__stdcall ScalarDelete(uint32_t size) {}
	virtual void		__stdcall nullsub_7() {}
	virtual void		__stdcall nullsub_8() {}
	virtual int			__stdcall LoadImageA(void) {}

	void SetPos(const S_vector& aPos);
	const S_vector& GetPos() const { return *(S_vector*)&m_aLocalMat.e[12]; }

	void SetScale(const S_vector& aScale);
	const S_vector& GetScale() const { return m_aScale; }

	void SetRot(const S_quat& aRot);
	const S_quat& GetRot() const { return m_aRot;  }

	const char* GetName() { return m_pSzName; }
	const S_matrix& GetMatrix() const { return m_aLocalMat; }

	static void InitHooks();
private:
	uint32_t m_uVtable;
	int32_t m_iRefCount;
	char _pad0[4];
	char* m_pSzProperties;
	S_matrix m_aWorldMat;
	S_matrix m_aLocalMat;
	S_vector m_aScale;
	S_quat m_aRot;
	uint32_t m_uFlags;
	I3D_bbox m_aLocalBBOX;
	I3D_bsphere m_aLocalBSphere;
	I3D_bbox m_aWorldBBOX;
	I3D_bsphere m_aWorldBSphere;
	char* m_pSzName;
	char* m_pSzModelName;
	uint32_t _pad1;
	I3D_frame* m_pOwner;
	int32_t m_eFrameType;
};

#pragma pack(pop)