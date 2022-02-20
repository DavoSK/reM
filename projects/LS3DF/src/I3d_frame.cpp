#include "I3d_frame.h"

void I3D_frame::SetPos(const S_vector& aPos)
{
    if ((m_uFlags & 0x80u) == 0)
    {
        m_aLocalMat.e[12] = aPos.x;
        m_aLocalMat.e[13] = aPos.y;
        m_aLocalMat.e[14] = aPos.z;
        m_uFlags = m_uFlags & 0xBFFFFEDF | 0x40000000;
    }
}

void I3D_frame::SetScale(const S_vector& aScale)
{
    m_aScale = aScale;

    if ((m_uFlags & 8) == 0) 
        m_aRot.Make(m_aLocalMat);

    m_uFlags = m_uFlags & 0xBFFFFEC3 | 0x40000008;
}

void I3D_frame::SetRot(const S_quat& aRot)
{
    m_aRot = aRot;
    m_aRot.Normalize();
    m_uFlags = m_uFlags & 0xfffffecb | 0x40000008;
}


void I3D_frame::InitHooks()
{

}
