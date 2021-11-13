#include "I3d_math.h"
#include <cmath>
#include <algorithm>

#include <reversiblehooks/ReversibleHooks.h>
#include "plugin.h"

S_vector::S_vector(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
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

void S_vector::InitHooks()
{
    uint32_t engineHandle = (uint32_t)GetModuleHandle("LS3DF.dll");
    auto rebase = [engineHandle](uint32_t adr) -> uint32_t {
        return (adr - 0x10000000) + engineHandle;
    };

    ReversibleHooks::Install("S_vector", "CosAngleTo", rebase(0x1002E710), &S_vector::CosAngleTo);
    ReversibleHooks::Install("S_vector", "AngleTo", rebase(0x1002E7A0), &S_vector::AngleTo);
    ReversibleHooks::Install("S_vector", "RotateByMatrix", rebase(0x1002EA10), &S_vector::RotateByMatrix);
    ReversibleHooks::Install("S_vector", "RotateByNormMatrix", rebase(0x1002EAB0), &S_vector::RotateByNormMatrix);
}