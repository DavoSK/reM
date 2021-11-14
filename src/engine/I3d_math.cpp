#include "I3d_math.h"
#include <cmath>
#include <cassert>
#include <algorithm>

#include <reversiblehooks/ReversibleHooks.h>
#include "plugin.h"

#pragma region VECTOR
/* -------------- [S_vector] -------------- */
S_vector::S_vector(float _x, float _y, float _z)
{
    x = _x;
    y = _y;
    z = _z;
}

S_vector::S_vector() 
{
    x = 0;
    y = 0;
    z = 0;
}

S_vector S_vector::operator*(float const& scalar)
{
    return S_vector(x * scalar, y * scalar, z * scalar);
}

void S_vector::operator*=(float const& scalar)
{
    x = x * scalar;
    y = y * scalar;
    z = z * scalar;
}

void S_vector::operator/=(float const& scalar)
{
    x = x / scalar;
    y = y / scalar;
    z = z / scalar;
}

S_vector S_vector::operator*(S_vector const& vec)
{
    return S_vector(x * vec.x, y * vec.y, z * vec.z);
}

void S_vector::operator*=(S_vector const& vec)
{
    x = x * vec.x;
    y = y * vec.y;
    z = z * vec.z;
}

S_vector S_vector::operator*(S_matrix const& mat)
{
    S_vector res;
    res.x = x * mat.m_fData[0] + mat.m_fData[12] + y * mat.m_fData[4] + z * mat.m_fData[8];
    res.y = x * mat.m_fData[1] + mat.m_fData[13] + y * mat.m_fData[5] + z * mat.m_fData[9];
    res.z = x * mat.m_fData[2] + mat.m_fData[14] + y * mat.m_fData[6] + z * mat.m_fData[10];
    return res;
}

void S_vector::operator*=(S_matrix const& mat)
{
    x = x * mat.m_fData[0] + mat.m_fData[12] + y * mat.m_fData[4] + z * mat.m_fData[8];
    y = x * mat.m_fData[1] + mat.m_fData[13] + y * mat.m_fData[5] + z * mat.m_fData[9];
    z = x * mat.m_fData[2] + mat.m_fData[14] + y * mat.m_fData[6] + z * mat.m_fData[10];
}

S_vector S_vector::operator+(S_vector const& vec)
{
    return S_vector(x + vec.x, y + vec.y, z + vec.z);
}

void S_vector::operator+=(S_vector const& vec)
{
    x += vec.x;
    y += vec.y;
    z += vec.z;
}

double S_vector::Dot(S_vector const& vec)
{
     return vec.z * z + vec.y * y + vec.x * x;
}

S_vector S_vector::operator-(S_vector const& vec)
{
    return S_vector(x - vec.x, y - vec.y, z - vec.z);
}

void S_vector::GetNormal(S_vector const& vec1, S_vector const& vec2, S_vector const& vec3)
{
    float v4 = vec3.x - vec2.x;
    float v5 = vec3.y - vec2.y;
    float v6 = vec3.z - vec2.z;

    float v8 = vec1.x - vec2.x;
    float v9 = vec1.y - vec2.y;
    float v7 = vec1.z - vec2.z;

    x = v7 * v5 - v9 * v6;
    y = v8 * v6 - v4 * v7;
    z = v4 * v9 - v8 * v5;
}

double S_vector::Magnitude2() const
{
    return (x * x) + (y * y) + (z * z);
}

double S_vector::Magniture() const
{
    return sqrt(Magnitude2());
}

void S_vector::SetNormalized(S_vector const& vec)
{
    float len = sqrt((float)vec.Magnitude2());
    if (len != 0.0f)
    {
        this->x = vec.x / len;
        this->y = vec.y / len;
        this->z = vec.z / len;
    }
}

S_vector S_vector::Cross(S_vector const& vec)
{
    return S_vector(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x);
}

double S_vector::CosAngleTo(S_vector const& vec)
{
    float val = (vec.z * z + vec.y * y + vec.x * x 
    / sqrt(
            (x * x
        + y * y
        + z *z)
        * (vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)));
    return sqrt((val + 1.0f) * 0.5f);
}

double S_vector::AngleTo(S_vector const& vec)
{   
    float len1 = x * x + y * y + z  *z;
    float len2 = vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
    
    float dot = x * vec.x + y * vec.y + z * vec.z;

    return acos(std::clamp(dot / sqrt(len1 * len2), -1.0f, 1.0f));
}

S_vector S_vector::RotateByMatrix(S_matrix const& mat)
{   
    S_vector res;
    res.x = x * mat.m_fData[0] + y * mat.m_fData[4] + z * mat.m_fData[8];
    res.y = x * mat.m_fData[1] + y * mat.m_fData[5] + z * mat.m_fData[9];
    res.z = x * mat.m_fData[2] + y * mat.m_fData[6] + z * mat.m_fData[10];
    return res;
}

S_vector S_vector::RotateByNormMatrix(S_matrix const& mat)
{
    S_vector res;
    res.RotateByMatrix(mat);
    res.SetNormalized(res);
    return res;
}

#pragma endregion

/* -------------- [S_quat] -------------- */
#pragma region QUAT

S_matrix S_quat::RotationMatrix()
{
    S_matrix result;

    if (fabs(w) < 1.0f)
    {
        result.m_fData[3] = 0.0f;
        result.m_fData[7] = 0.0f;

        float v4 = x + x;
        float v6 = y + y;
        float v19 = z + z;
        memset(&result.m_fData[11], 0, 16);
        result.m_fData[15] = 1.0f;
        float v17 = v4 * x;
        float v11 = v6 * x;
        float v13 = v19 * x;
        float v16 = v6 * y;
        float v15 = v19 * y;
        float v12 = v19 * z;
        float v14 = v4 * w;
        float v8 = v6 * w;
        float  v9 = v19 * w;
        result.m_fData[0] = 1.0f - (v12 + v16);
        result.m_fData[1] = v11 - v9;
        result.m_fData[2] = v8 + v13;
        result.m_fData[4] = v9 + v11;
        result.m_fData[5] = 1.0f - (v12 + v17);
        result.m_fData[6] = v15 - v14;
        result.m_fData[8] = v13 - v8;
        result.m_fData[9] = v14 + v15;
        result.m_fData[10] = 1.0f - (v16 + v17);
    }
    else
    {
        //NOTE: make identity rot mat
        result.m_fData[0] = 1.0f;
        result.m_fData[1] = 0.0f;
        result.m_fData[2] = 0.0f;
        result.m_fData[3] = 0.0f;
        result.m_fData[4] = 0.0f;
        result.m_fData[5] = 1.0f;
        result.m_fData[6] = 0.0f;
        result.m_fData[7] = 0.0f;
        result.m_fData[8] = 0.0f;
        result.m_fData[9] = 0.0f;
        result.m_fData[10] = 1.0f;
        result.m_fData[11] = 0.0f;
        result.m_fData[12] = 0.0f;
        result.m_fData[13] = 0.0f;
        result.m_fData[14] = 0.0f;
        result.m_fData[15] = 0.0f;
    }

    return result;
}

S_vector __stdcall S_quat::GetDir()
{
    S_vector result;

    if (fabs(w) < 1.0f)
    {
        float v4 = x + x;
        float v5 = y + y;
        float v6 = z + z;
        float v9 = v4 * x;
        float v8 = v5 * y;
        float v7 = v6 * z;
        float v10 = v4 * w;
        result.x = v6 * x - v5 * w;
        result.y = v10 + v7;
        result.z = 1.0f - (v8 + v9);
    }
    else
    {
        result.x = 0.0f;
        result.y = 0.0f;
        result.z = 1.0f;
    }
    return result;
}

S_quat S_quat::Slerp(S_quat const& quat, float t, bool unk)
{
    float mag2 = quat.y * y + quat.x * x + quat.w * w + quat.z * z;
    float v12;
    float v13;
    float v14;
    float v15;
    float v8;
    float v9;
    float v10;
    float v20;
    float v22;
    float v21 = mag2;

    if (mag2 >= 0.0f)
    {
        v12 = quat.x;
        v13 = quat.y;
        v14 = quat.z;
        v15 = quat.w;
    }
    else
    {
        v21 = -mag2;
        v12 = -quat.x;
        v13 = -quat.y;
        v14 = -quat.z;
        v15 = -quat.w;
    }

    if (1.0f - v21 <= 0.0001f)
    {
        v9 = 1.0f - t;
        v10 = t;
    }
    else
    {
        v8 = acos(v21);
        v22 = v8;
        v20 = 1.0f / sin(v8);
        v9 = sin((1.0f - t) * v22) * v20;
        v10 = sin(v22 * t) * v20;
    }

    S_quat res;
    res.x = v12 * v10 + v9 * x;
    res.y = v13 * v10 + v9 * y;
    res.z = v9 * z + v10 * v14;
    res.w = v9 * w + v10 * v15;
    return res;
}

void __stdcall S_quat::Make(S_vector const& axis, float angle)
{
    float halfAngle = angle * 0.5f;
    float v7 = sin(halfAngle);
    x = v7 * axis.x;
    y = v7 * axis.y;
    z = v7 * axis.z;
    w = cos(halfAngle);

    float mag2 = x * x + y * y + z * z + w * w;
    if (fabs(mag2 - 1.0f) >= 0.00000000999999993922529f)
    {
        if (mag2 >= 0.0000000099999999f)
        {
            float mag = 1.0f / sqrt(mag2);
            x = mag * x;
            y = mag * y;
            z = mag * z;
            w = mag * w;
        }
        else
        {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
            w = 1.0f;
        }
    }
}

void __stdcall S_quat::Make(S_matrix const& mat)
{
    assert(false);
}

S_quat S_quat::operator*(S_quat const& quat)
{    
    float v12 = quat.y * x - y * quat.x;
    float v11 = z * w;
    float v5 = quat.y * w;
    float v7 = quat.z * w;
    float v9 = quat.x * w + x * quat.w;
    float v10 = v7 + v11;
    float v4 = v9 + quat.z * y - quat.y * z;
    float v6 = v5 + y * w + quat.x * z - quat.z * x;
    float v8 = v10 + v12;
    
    S_quat res;
    res.w = quat.w * w - (z * quat.z + y * quat.y + x * quat.x);
    res.x = v4;
    res.y = v6;
    res.z = v8;
    return res;

}

void __stdcall S_quat::Normalize()
{
    float mag2 = w * w + x * x + y * y + z * z;
    float invSqrt = std::clamp(1.0f / sqrt(mag2), -1.0f, 1.0f);
    w = w * invSqrt;
    x = x * invSqrt;
    y = y * invSqrt;
    z = z * invSqrt;
}

#pragma endregion

void S_vector::InitHooks()
{
    uint32_t engineHandle = (uint32_t)GetModuleHandle("LS3DF.dll");
    auto rebase = [engineHandle](uint32_t adr) -> uint32_t {
        return (adr - 0x10000000) + engineHandle;
    };

    //S_vector
    ReversibleHooks::Install("S_vector", "CosAngleTo", rebase(0x1002E710), &S_vector::CosAngleTo);
    ReversibleHooks::Install("S_vector", "AngleTo", rebase(0x1002E7A0), &S_vector::AngleTo);
    ReversibleHooks::Install("S_vector", "RotateByMatrix", rebase(0x1002EA10), &S_vector::RotateByMatrix);
    ReversibleHooks::Install("S_vector", "RotateByNormMatrix", rebase(0x1002EAB0), &S_vector::RotateByNormMatrix);

    //S_quat
    ReversibleHooks::Install("S_quat", "RotationMatrix", rebase(0x1002EC00), &S_quat::RotationMatrix);
    ReversibleHooks::Install("S_quat", "GetDir", rebase(0x1002ED60), &S_quat::GetDir);
    //ReversibleHooks::Install("S_quat", "Make", rebase(0x1002EE00), &S_quat::Make);
    ReversibleHooks::Install("S_quat", "operator*", rebase(0x1002FA70), &S_quat::operator*);

    ReversibleHooks::Install("S_quat", "Slerp", rebase(0x1002FEE0), &S_quat::Slerp);
    ReversibleHooks::Install("S_quat", "Normalize*", rebase(0x10030140), &S_quat::Normalize);
}