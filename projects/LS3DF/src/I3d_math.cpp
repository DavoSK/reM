#include "I3d_math.h"
#include <cmath>
#include <cassert>
#include <algorithm>

#include <reversiblehooks/ReversibleHooks.h>
#include "plugin.h"

#pragma region VECTOR
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

S_vector S_vector::operator-(S_vector const& vec) const
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

void __stdcall S_quat::Inverse(S_vector& vec, float& val)
{
    //NOTE: not used
    assert(false);
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

S_quat& __stdcall S_quat::RotateByMatrix(S_matrix const& mat)
{
    S_quat quat;
    quat.Make(mat);
   
    S_quat result;
    result.x = quat.x * w + quat.w * x + quat.z * y - quat.y * z;
    result.y = quat.y * w + quat.w * y + quat.x * z - quat.z * x;
    result.w = quat.w * w - (quat.y * y + quat.z * z + quat.x * x);
    result.z = (quat.z * w + quat.w * z) + quat.y * x - quat.x * y;
    return result;
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

#pragma region MATRIX
float gIdentityMatrix[] = { 1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f };

void __stdcall S_matrix::Identity()
{
    memcpy(m_fData, gIdentityMatrix, sizeof(S_matrix));
}

bool __stdcall S_matrix::Inverse(S_matrix const& mat)
{
    m_fData[0] = mat.m_fData[5] * mat.m_fData[10] - mat.m_fData[9] * mat.m_fData[6];
    m_fData[4] = mat.m_fData[10] * mat.m_fData[4] - mat.m_fData[8] * mat.m_fData[6];
    m_fData[8] = mat.m_fData[9] * mat.m_fData[4] - mat.m_fData[5] * mat.m_fData[8];

    float mag2 = mat.m_fData[0] * m_fData[0] - m_fData[4] * mat.m_fData[1] + m_fData[8] * mat.m_fData[2];
    if (fabs(mag2) >= 0.00000000999999993922529f)
    {
        float invMag = 1.0f / mag2;
        m_fData[0] = invMag * m_fData[0];;
        m_fData[1] = -((mat.m_fData[10] * mat.m_fData[1] - mat.m_fData[9] * mat.m_fData[2]) * invMag);
        m_fData[2] = (mat.m_fData[6] * mat.m_fData[1] - mat.m_fData[2] * mat.m_fData[5]) * invMag;
        m_fData[3] = 0.0f;
        m_fData[4] = -(invMag * m_fData[4]);
        m_fData[5] = (mat.m_fData[0] * mat.m_fData[10] - mat.m_fData[2] * mat.m_fData[8]) * invMag;
        m_fData[6] = -((mat.m_fData[0] * mat.m_fData[6] - mat.m_fData[2] * mat.m_fData[4]) * invMag);
        m_fData[7] = 0.0f;
        m_fData[8] = invMag * m_fData[8];
        m_fData[9] = -((mat.m_fData[9] * mat.m_fData[0] - mat.m_fData[8] * mat.m_fData[1]) * invMag);
        m_fData[11] = 0.0f;
        m_fData[10] = (mat.m_fData[5] * mat.m_fData[0] - mat.m_fData[4] * mat.m_fData[1]) * invMag;
        m_fData[12] = -(m_fData[0] * mat.m_fData[12] + m_fData[8] * mat.m_fData[14] + m_fData[4] * mat.m_fData[13]);
        m_fData[13] = -(m_fData[5] * mat.m_fData[13] + m_fData[9] * mat.m_fData[14] + mat.m_fData[12] * m_fData[1]);
        m_fData[14] = -((m_fData[6] * mat.m_fData[13] + m_fData[2] * mat.m_fData[12]) + mat.m_fData[14] * m_fData[10]);
        m_fData[15] = 1.0f;
    }
    else
    {
        memcpy(m_fData, &gIdentityMatrix, 0x40u);
        return false;
    }
    return true;
}

S_matrix __stdcall S_matrix::RotateByMatrix(S_matrix const& mat)
{
    S_matrix res;   
    res.m_fData[0] = m_fData[0] * mat.m_fData[0] + m_fData[1] * mat.m_fData[4] + m_fData[2] * mat.m_fData[8];
    res.m_fData[1] = m_fData[0] * mat.m_fData[1] + m_fData[1] * mat.m_fData[5] + m_fData[2] * mat.m_fData[9];
    res.m_fData[2] = m_fData[0] * mat.m_fData[2] + m_fData[1] * mat.m_fData[6] + m_fData[2] * mat.m_fData[10];
    res.m_fData[3] = 0.0f;

    res.m_fData[4] = m_fData[4] * mat.m_fData[0] + m_fData[5] * mat.m_fData[4] + m_fData[6] * mat.m_fData[8];
    res.m_fData[5] = m_fData[4] * mat.m_fData[1] + m_fData[5] * mat.m_fData[5] + m_fData[6] * mat.m_fData[9];
    res.m_fData[6] = m_fData[4] * mat.m_fData[2] + m_fData[5] * mat.m_fData[6] + m_fData[6] * mat.m_fData[10];
    res.m_fData[7] = 0.0f;

    res.m_fData[8] = m_fData[8] * mat.m_fData[0] + m_fData[9] * mat.m_fData[4] + m_fData[10] * mat.m_fData[8];
    res.m_fData[9] = m_fData[8] * mat.m_fData[1] + m_fData[9] * mat.m_fData[5] + m_fData[10] * mat.m_fData[9];
    res.m_fData[10] = m_fData[8] * mat.m_fData[2] + m_fData[9] * mat.m_fData[6] + m_fData[10] * mat.m_fData[10];
    res.m_fData[11] = 0.0f;
  
    res.m_fData[12] = m_fData[12] * mat.m_fData[0] + m_fData[13] * mat.m_fData[4] + m_fData[14] * mat.m_fData[8];
    res.m_fData[13] = m_fData[12] * mat.m_fData[1] + m_fData[13] * mat.m_fData[5] + m_fData[14] * mat.m_fData[9];
    res.m_fData[14] = m_fData[12] * mat.m_fData[2] + m_fData[13] * mat.m_fData[6] + m_fData[14] * mat.m_fData[10];
    res.m_fData[15] = 1.0f;
    return res;
}

S_matrix S_matrix::operator*(S_matrix const& mat)
{
    S_matrix res;
    res.m_fData[0] = m_fData[0] * mat.m_fData[0] + m_fData[1] * mat.m_fData[4] + m_fData[2] * mat.m_fData[8];
    res.m_fData[1] = m_fData[0] * mat.m_fData[1] + m_fData[1] * mat.m_fData[5] + m_fData[2] * mat.m_fData[9];
    res.m_fData[2] = m_fData[0] * mat.m_fData[2] + m_fData[1] * mat.m_fData[6] + m_fData[2] * mat.m_fData[10];
    res.m_fData[3] = 0.0f;

    res.m_fData[4] = m_fData[4] * mat.m_fData[0] + m_fData[5] * mat.m_fData[4] + m_fData[6] * mat.m_fData[8];
    res.m_fData[5] = m_fData[4] * mat.m_fData[1] + m_fData[5] * mat.m_fData[5] + m_fData[6] * mat.m_fData[9];
    res.m_fData[6] = m_fData[4] * mat.m_fData[2] + m_fData[5] * mat.m_fData[6] + m_fData[6] * mat.m_fData[10];
    res.m_fData[7] = 0.0f;

    res.m_fData[8] = m_fData[8] * mat.m_fData[0] + m_fData[9] * mat.m_fData[4] + m_fData[10] * mat.m_fData[8];
    res.m_fData[9] = m_fData[8] * mat.m_fData[1] + m_fData[9] * mat.m_fData[5] + m_fData[10] * mat.m_fData[9];
    res.m_fData[10] = m_fData[8] * mat.m_fData[2] + m_fData[9] * mat.m_fData[6] + m_fData[10] * mat.m_fData[10];
    res.m_fData[11] = 0.0f;

    res.m_fData[12] = m_fData[12] * mat.m_fData[0] + mat.m_fData[12] + m_fData[13] * mat.m_fData[4] + m_fData[14] * mat.m_fData[8];
    res.m_fData[13] = m_fData[12] * mat.m_fData[1] + mat.m_fData[13] + m_fData[13] * mat.m_fData[5] + m_fData[14] * mat.m_fData[9];
    res.m_fData[14] = m_fData[12] * mat.m_fData[2] + mat.m_fData[14] + m_fData[13] * mat.m_fData[6] + m_fData[14] * mat.m_fData[10];
    res.m_fData[15] = 1.0f;
    return res;
}

void S_matrix::operator*=(S_matrix const& mat)
{
    *this = *this * mat;
}

S_matrix __stdcall S_matrix::Mul4X4(S_matrix const& mat)
{
    S_matrix res;
    res.m_fData[0] = m_fData[2] * mat.m_fData[8] + m_fData[3] * mat.m_fData[12] + m_fData[1] * mat.m_fData[4] + m_fData[0] * mat.m_fData[0];
    res.m_fData[1] = m_fData[2] * mat.m_fData[9] + m_fData[3] * mat.m_fData[13] + m_fData[1] * mat.m_fData[5] + m_fData[0] * mat.m_fData[1];
    res.m_fData[2] = m_fData[2] * mat.m_fData[10] + m_fData[3] * mat.m_fData[14] + m_fData[1] * mat.m_fData[6] + m_fData[0] * mat.m_fData[2];
    res.m_fData[3] = m_fData[2] * mat.m_fData[11] + m_fData[3] * mat.m_fData[15] + m_fData[1] * mat.m_fData[7] + m_fData[0] * mat.m_fData[3];
    res.m_fData[4] = m_fData[6] * mat.m_fData[8] + m_fData[7] * mat.m_fData[12] + m_fData[5] * mat.m_fData[4] + m_fData[4] * mat.m_fData[0];
    res.m_fData[5] = m_fData[6] * mat.m_fData[9] + m_fData[7] * mat.m_fData[13] + m_fData[5] * mat.m_fData[5] + m_fData[4] * mat.m_fData[1];
    res.m_fData[6] = m_fData[6] * mat.m_fData[10] + m_fData[7] * mat.m_fData[14] + m_fData[5] * mat.m_fData[6] + m_fData[4] * mat.m_fData[2];
    res.m_fData[7] = m_fData[6] * mat.m_fData[11] + m_fData[7] * mat.m_fData[15] + m_fData[5] * mat.m_fData[7] + m_fData[4] * mat.m_fData[3];
    res.m_fData[8] = m_fData[10] * mat.m_fData[8] + m_fData[11] * mat.m_fData[12] + m_fData[9] * mat.m_fData[4] + m_fData[8] * mat.m_fData[0];
    res.m_fData[9] = m_fData[10] * mat.m_fData[9] + m_fData[11] * mat.m_fData[13] + m_fData[9] * mat.m_fData[5] + m_fData[8] * mat.m_fData[1];
    res.m_fData[10] = m_fData[10] * mat.m_fData[10] + m_fData[11] * mat.m_fData[14] + m_fData[9] * mat.m_fData[6] + m_fData[8] * mat.m_fData[2];
    res.m_fData[11] = m_fData[10] * mat.m_fData[11] + m_fData[11] * mat.m_fData[15] + m_fData[9] * mat.m_fData[7] + m_fData[8] * mat.m_fData[3];
    res.m_fData[12] = m_fData[14] * mat.m_fData[8] + m_fData[15] * mat.m_fData[12] + m_fData[13] * mat.m_fData[4] + m_fData[12] * mat.m_fData[0];
    res.m_fData[13] = m_fData[14] * mat.m_fData[9] + m_fData[15] * mat.m_fData[13] + m_fData[13] * mat.m_fData[5] + m_fData[12] * mat.m_fData[1];
    res.m_fData[14] = m_fData[14] * mat.m_fData[10] + m_fData[15] * mat.m_fData[14] + m_fData[13] * mat.m_fData[6] + m_fData[12] * mat.m_fData[2];
    res.m_fData[15] = m_fData[14] * mat.m_fData[11] + m_fData[15] * mat.m_fData[15] + m_fData[13] * mat.m_fData[7] + m_fData[12] * mat.m_fData[3];
    return res;
}

S_matrix& __thiscall S_matrix::Make4X4(S_matrix const& mat1, S_matrix const& mat2)
{
    m_fData[0] = mat1.m_fData[2] * mat2.m_fData[8] + mat1.m_fData[3] * mat2.m_fData[12] + mat1.m_fData[1] * mat2.m_fData[4] + *mat1.m_fData * *mat2.m_fData;
    m_fData[1] = mat1.m_fData[2] * mat2.m_fData[9] + mat1.m_fData[3] * mat2.m_fData[13] + mat1.m_fData[1] * mat2.m_fData[5] + *mat1.m_fData * mat2.m_fData[1];
    m_fData[2] = mat1.m_fData[2] * mat2.m_fData[10] + mat1.m_fData[3] * mat2.m_fData[14] + mat1.m_fData[1] * mat2.m_fData[6] + *mat1.m_fData * mat2.m_fData[2];
    m_fData[3] = mat1.m_fData[2] * mat2.m_fData[11] + mat1.m_fData[3] * mat2.m_fData[15] + mat1.m_fData[1] * mat2.m_fData[7] + *mat1.m_fData * mat2.m_fData[3];
    m_fData[4] = mat1.m_fData[6] * mat2.m_fData[8] + mat1.m_fData[7] * mat2.m_fData[12] + mat1.m_fData[5] * mat2.m_fData[4] + mat1.m_fData[4] * *mat2.m_fData;
    m_fData[5] = mat1.m_fData[6] * mat2.m_fData[9] + mat1.m_fData[7] * mat2.m_fData[13] + mat1.m_fData[5] * mat2.m_fData[5] + mat1.m_fData[4] * mat2.m_fData[1];
    m_fData[6] = mat1.m_fData[6] * mat2.m_fData[10] + mat1.m_fData[7] * mat2.m_fData[14] + mat1.m_fData[5] * mat2.m_fData[6] + mat1.m_fData[4] * mat2.m_fData[2];
    m_fData[7] = mat1.m_fData[6] * mat2.m_fData[11] + mat1.m_fData[7] * mat2.m_fData[15] + mat1.m_fData[5] * mat2.m_fData[7] + mat1.m_fData[4] * mat2.m_fData[3];
    m_fData[8] = mat1.m_fData[10] * mat2.m_fData[8] + mat1.m_fData[11] * mat2.m_fData[12] + mat1.m_fData[9] * mat2.m_fData[4] + mat1.m_fData[8] * *mat2.m_fData;
    m_fData[9] = mat1.m_fData[10] * mat2.m_fData[9] + mat1.m_fData[11] * mat2.m_fData[13] + mat1.m_fData[9] * mat2.m_fData[5] + mat1.m_fData[8] * mat2.m_fData[1];
    m_fData[10] = mat1.m_fData[10] * mat2.m_fData[10] + mat1.m_fData[11] * mat2.m_fData[14] + mat1.m_fData[9] * mat2.m_fData[6] + mat1.m_fData[8] * mat2.m_fData[2];
    m_fData[11] = mat1.m_fData[10] * mat2.m_fData[11] + mat1.m_fData[11] * mat2.m_fData[15] + mat1.m_fData[9] * mat2.m_fData[7] + mat1.m_fData[8] * mat2.m_fData[3];
    m_fData[12] = mat1.m_fData[14] * mat2.m_fData[8] + mat1.m_fData[15] * mat2.m_fData[12] + mat1.m_fData[13] * mat2.m_fData[4] + mat1.m_fData[12] * *mat2.m_fData;
    m_fData[13] = mat1.m_fData[14] * mat2.m_fData[9] + mat1.m_fData[15] * mat2.m_fData[13] + mat1.m_fData[13] * mat2.m_fData[5] + mat1.m_fData[12] * mat2.m_fData[1];
    m_fData[14] = mat1.m_fData[14] * mat2.m_fData[10] + mat1.m_fData[15] * mat2.m_fData[14] + mat1.m_fData[13] * mat2.m_fData[6] + mat1.m_fData[12] * mat2.m_fData[2];
    m_fData[15] = mat1.m_fData[14] * mat2.m_fData[11] + mat1.m_fData[15] * mat2.m_fData[15] + mat1.m_fData[13] * mat2.m_fData[7] + mat1.m_fData[12] * mat2.m_fData[3];
    return *this;
}

void __stdcall S_matrix::SetDir(S_vector const& dir)
{
    float v3; // st7
    float v4; // st7
    float v5; // st7
    float v6; // st7
    float v7; // st7
    float v8; // st6
    float v9; // st6
    float v10; // st7
    float v11; // st6
    float v17; // [esp+10h] [ebp-10h]
    float thisb; // [esp+24h] [ebp+4h]
    float thisc; // [esp+24h] [ebp+4h]
    float thisd; // [esp+24h] [ebp+4h]

    Identity();

    m_fData[8] = dir.x;
    m_fData[9] = dir.y;
    m_fData[10] = dir.z;

    double dirMag2 = dir.Magnitude2();
    if (fabs(dirMag2 - 1.0f) < 0.00000000999999993922529f)
    {
        m_fData[8] = dir.x;
        m_fData[9] = dir.y;
        m_fData[10] = dir.z;
        goto LABEL_17;
    }

    if (dirMag2 >= 0.0000000099999999f)
    {
        v4 = (float)(1.0f / sqrt(dirMag2));
        m_fData[8] = v4 * dir.x;
        m_fData[9] = v4 * dir.y;
        v3 = v4 * dir.z;
    LABEL_16:
        m_fData[10] = v3;
        goto LABEL_17;
    }
    
    if (dir.x != 0.0f)
    {
        if (dir.x >= 0.0f)
            m_fData[8] = 1.0f;
        else
            m_fData[8] = -1.0f;
        goto LABEL_17;
    }
    
    if (dir.z != 0.0f)
    {
        if (dir.z >= 0.0f)
            v3 = 1.0f;
        else
            v3 = -1.0f;
        goto LABEL_16;
    }
    
    if (dir.y >= 0.0f)
        m_fData[9] = 1.0f;
    else
        m_fData[9] = -1.0f;


LABEL_17:
    thisb = dir.z * dir.z + dir.x * dir.x;
    if (thisb > 0.0000000099999999f)
    {
        if (fabs(thisb - 1.0f) >= 0.00000000999999993922529f)
        {
            v6 = 1.0f / sqrt(thisb);
            m_fData[0] = v6 * dir.z;
            v5 = v6 * dir.x;
        }
        else
        {
            m_fData[0] = dir.z;
            v5 = dir.x;
        }
        m_fData[2] = -v5;
    }
    if (fabs(dir.z) + fabs(dir.x) > 0.00000000999999993922529f)
    {
        v7 = dir.z;
        v8 = -dir.x;
        m_fData[1] = 0;
        v17 = v8;
        m_fData[0] = v7;
        m_fData[2] = v17;
        v9 = v17 * v17 + v7 * v7;
        thisc = v9;
        if (fabs(v9 - 1.0f) < 0.00000000999999993922529f)
        {
            m_fData[0] = v7;
            m_fData[1] = 0.0f;
            m_fData[2] = v17;
            goto LABEL_37;
        }
        if (thisc >= 0.0000000099999999f)
        {
            v11 = 1.0f / sqrt(thisc);
            thisd = v11;
            m_fData[0] = v11 * v7;
            m_fData[1] = thisd * 0.0f;
            v10 = v17 * thisd;
        }
        else
        {
            if (v7 != 0.0f)
            {
                if (v7 >= 0.0f)
                    m_fData[0] = 1.0f;
                else
                    m_fData[0] = -1.0f;
                goto LABEL_37;
            }
            if (v17 == 0.0f)
            {
                m_fData[1] = 1.0f;
                goto LABEL_37;
            }
            if (v17 >= 0.0f)
                v10 = 1.0f;
            else
                v10 = -1.0f;
        }
        m_fData[2] = v10;
    }
LABEL_37:
    m_fData[4] = -(m_fData[1] * dir.z - m_fData[2] * dir.y);
    m_fData[5] = -(m_fData[2] * dir.x - m_fData[0] * dir.z );
    m_fData[6] = -(m_fData[0] * dir.y - m_fData[1] * dir.x);
}

void __stdcall S_matrix::SetDir3(S_vector const& v1, S_vector const& v2)
{
    float v4;
    float v5;

    m_fData[8] = v1.x;
    m_fData[9] = v1.y;
    m_fData[10] = v1.z;

    float mag2 = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
    if (fabs(mag2 - 1.0f) < 0.00000000999999993922529f)
    {
        m_fData[8] = v1.x;
        m_fData[9] = v1.y;
        m_fData[10] = v1.z;
        goto LABEL_17;
    }

    if (mag2 >= 0.0000000099999999f)
    {
        v5 = 1.0f / sqrt(mag2);
        m_fData[8] = v5 * v1.x;
        m_fData[9] = v5 * v1.y;
        v4 = v5 * v1.z;
    LABEL_16:
        m_fData[10] = v4;
        goto LABEL_17;
    }

    if (v1.x != 0.0f)
    {
        if (v1.x >= 0.0f)
            m_fData[8] = 1.0f;
        else
            m_fData[8] = -1.0f;
        goto LABEL_17;
    }

    if (v1.z != 0.0f)
    {
        if (v1.z >= 0.0f)
            v4 = 1.0f;
        else
            v4 = -1.0f;
        goto LABEL_16;
    }

    if (v1.y >= 0.0f)
        m_fData[9] = 1.0f;
    else
        m_fData[9] = -1.0f;
LABEL_17:
    float v6 = m_fData[10] * v2.z
        + m_fData[9] * v2.y
        + m_fData[8] * v2.x;


    S_vector v30 = S_vector(v6 * m_fData[8], v6 * m_fData[9], v6 * m_fData[10]);
    S_vector v27 = v2 - v30;

    if (v27.z * v27.z + v27.y * v27.y + v27.x * v27.x < 0.0000000099999999f)
    {
        S_vector v33 = S_vector( m_fData[8] * m_fData[10], 
                                m_fData[9] * m_fData[10], 
                                m_fData[10] * m_fData[10]);

        v30 = S_vector(-v33.x, -v33.y, 1.0f - v33.z);
        v27 = v30;

        if (v30.x * v30.x + v30.y * v30.y + v30.z * v30.z < 0.0000000099999999f)
        {
            v33 = S_vector( m_fData[8] * m_fData[9], 
                            m_fData[9] * m_fData[9], 
                            m_fData[10] * m_fData[9]);

            S_vector v30 = S_vector(-v33.x, 1.0f - v33.y, -v33.z);
            v27 = v30;

            if ( v33.Dot(v33) < 0.0000000099999999f )
            {
                v27 = S_vector(0.0f, 1.0f, 0.0f);
            }
        }
    }
    
    S_vector* secRow = (S_vector*)(&m_fData[4]);
    secRow->SetNormalized(v27);

    S_vector* firstRow = (S_vector*)(&m_fData[0]);
    S_vector v8 = v27.Cross(v1);
    firstRow->SetNormalized(v8);
}

void __stdcall S_matrix::SetDir(S_vector const& v1, S_vector const& v2)
{
    S_matrix::SetDir3(v1, v2);
    m_fData[3] = 0.0f;
    m_fData[7] = 0.0f;
    m_fData[11] = 0.0f;
    m_fData[12] = 0.0f;
    m_fData[13] = 0.0f;
    m_fData[14] = 0.0f;
    m_fData[15] = 1.0f;
}

void __stdcall S_matrix::SetRot3(S_quat const& rot)
{
    if (fabs(rot.w) < 0.9999999900000001)
    {
        float v3 = rot.x + rot.x;
        float v4 = rot.y + rot.y;
        float v14 = rot.z + rot.z;
        float v13 = (rot.x + rot.x) * rot.x;
        float v7 = v4 * rot.x;
        float v9 = v14 * rot.x;
        float v12 = v4 * rot.y;
        float v11 = v14 * rot.y;
        float v8 = v14 * rot.z;
        float v10 = v3 * rot.w;
        float v5 = v4 * rot.w;
        float v6 = v14 * rot.w;
        m_fData[0] = 1.0f - (v8 + v12);
        m_fData[1] = v7 - v6;
        m_fData[2] = v5 + v9;
        m_fData[4] = v6 + v7;
        m_fData[5] = 1.0f - (v8 + v13);
        m_fData[6] = v11 - v10;
        m_fData[8] = v9 - v5;
        m_fData[9] = v10 + v11;
        m_fData[10] = 1.0f - (v12 + v13);
    }
    else
    {   
        m_fData[0] = 1.0f;
        m_fData[1] = 0.0f;
        m_fData[2] = 0.0f;
        m_fData[4] = 0.0f;
        m_fData[5] = 1.0f;
        m_fData[6] = 0.0f;
        m_fData[8] = 0.0f;
        m_fData[9] = 0.0f;
        m_fData[10] = 1.0f;
    }
}

void __stdcall S_matrix::SetRot(S_quat const& rot)
{
    S_matrix::SetRot3(rot);
    m_fData[3] = 0.0f;
    m_fData[7] = 0.0f;
    m_fData[11] = 0.0f;
    m_fData[12] = 0.0f;
    m_fData[13] = 0.0f;
    m_fData[14] = 0.0f;
    m_fData[15] = 1.0f;
}

void __stdcall S_matrix::SetRot(S_matrix const& mat)
{
    m_fData[0] = mat.m_fData[0];
    m_fData[1] = mat.m_fData[1];
    m_fData[2] = mat.m_fData[2];
    m_fData[4] = mat.m_fData[4];
    m_fData[5] = mat.m_fData[5];
    m_fData[6] = mat.m_fData[6];
    m_fData[8] = mat.m_fData[8];
    m_fData[9] = mat.m_fData[9];
    m_fData[10] = mat.m_fData[10];

    float vMag2 = m_fData[0] * m_fData[0] + m_fData[4] * m_fData[4] + m_fData[8] * m_fData[8];
    if (fabs(vMag2 - 1.0f) > 0.00000000999999993922529f)
    {
        float invMag = 1.0f / sqrt(vMag2);
        m_fData[0] = invMag * m_fData[0];
        m_fData[4] = invMag * m_fData[4];
        m_fData[8] = invMag * m_fData[8];
    }

    vMag2 = m_fData[1] * m_fData[1] + m_fData[5] * m_fData[5] + m_fData[9] * m_fData[9];
    if (fabs(vMag2 - 1.0f) > 0.00000000999999993922529f)
    {
        float invMag = 1.0f / sqrt(vMag2);
        m_fData[1] = invMag * m_fData[1];
        m_fData[5] = invMag * m_fData[5];
        m_fData[9] = invMag * m_fData[9];
    }

    vMag2 = m_fData[2] * m_fData[2] + m_fData[6] * m_fData[6] + m_fData[10] * m_fData[10];
    if (fabs(vMag2 - 1.0f) > 0.00000000999999993922529f)
    {
        float invMag = 1.0f / sqrt(vMag2);
        m_fData[2] = invMag * m_fData[2];
        m_fData[6] = invMag * m_fData[6];
        m_fData[10] = invMag * m_fData[10];
    }
    
    m_fData[3] = 0.0f;
    m_fData[7] = 0.0f;
    m_fData[11] = 0.0f;
    m_fData[12] = 0.0f;
    m_fData[13] = 0.0f;
    m_fData[14] = 0.0f;
    m_fData[15] = 1.0f;
}

S_vector __stdcall S_matrix::GetScale()
{
    float x = m_fData[0] * m_fData[0] + m_fData[1] * m_fData[1] + m_fData[2] * m_fData[2];
    float y = m_fData[4] * m_fData[4] + m_fData[5] * m_fData[5] + m_fData[6] * m_fData[6];
    float z = m_fData[8] * m_fData[8] + m_fData[9] * m_fData[9] + m_fData[10] * m_fData[10];
    return S_vector(sqrt(x), sqrt(y), sqrt(z));
}

S_vector __stdcall S_matrix::GetScale2()
{
    S_vector ret;
    ret.x = m_fData[0] * m_fData[0] + m_fData[1] * m_fData[1] + m_fData[2] * m_fData[2];
    ret.y = m_fData[4] * m_fData[4] + m_fData[5] * m_fData[5] + m_fData[6] * m_fData[6];
    ret.z = m_fData[8] * m_fData[8] + m_fData[9] * m_fData[9] + m_fData[10] * m_fData[10];
    return ret;
}

double __stdcall S_matrix::GetUScale()
{
    float x = m_fData[0] * m_fData[0] + m_fData[1] * m_fData[1] + m_fData[2] * m_fData[2];
    float y = m_fData[4] * m_fData[4] + m_fData[5] * m_fData[5] + m_fData[6] * m_fData[6];
    float z = m_fData[8] * m_fData[8] + m_fData[9] * m_fData[9] + m_fData[10] * m_fData[10];
    return (sqrt(x) + sqrt(y) + sqrt(z)) * 0.33333334f;
}

#pragma endregion

void I3D_math::InitHooks()
{
    uint32_t engineHandle = (uint32_t)GetModuleHandle("LS3DF.dll");
    auto rebase = [engineHandle](uint32_t adr) -> uint32_t {
        return (adr - 0x10000000) + engineHandle;
    };

    //S_vector
    ReversibleHooks::Install("S_vector", "CosAngleTo", rebase(0x1002E710), &S_vector::CosAngleTo);
    ReversibleHooks::Install("S_vector", "AngleTo", rebase(0x1002E7A0), &S_vector::AngleTo);
    ReversibleHooks::Install("S_vector", "RotateByMatrix", rebase(0x1002CF40), &S_vector::RotateByMatrix);
    ReversibleHooks::Install("S_vector", "RotateByNormMatrix", rebase(0x1002EAB0), &S_vector::RotateByNormMatrix);

    //S_quat
    void(__stdcall S_quat:: * Make_m)(S_matrix const& mat) = &S_quat::Make;
    void(__stdcall S_quat:: * Make_v)(S_vector const& axis, float angle) = &S_quat::Make;

    ReversibleHooks::Install("S_quat", "RotationMatrix", rebase(0x1002EC00), &S_quat::RotationMatrix);
    ReversibleHooks::Install("S_quat", "GetDir", rebase(0x1002ED60), &S_quat::GetDir);
    ReversibleHooks::Install("S_quat", "Make(struct S_matrix const &)", rebase(0x1002EEF0), Make_m);
    ReversibleHooks::Install("S_quat", "Make(struct S_vector const &, float)", rebase(0x1002EE00), Make_v);
    ReversibleHooks::Install("S_quat", "operator*", rebase(0x1002FA70), &S_quat::operator*);
    ReversibleHooks::Install("S_quat", "Inverse", rebase(0x1002FD70), &S_quat::Inverse);
    ReversibleHooks::Install("S_quat", "Slerp", rebase(0x1002FEE0), &S_quat::Slerp);
    ReversibleHooks::Install("S_quat", "Normalize*", rebase(0x10030140), &S_quat::Normalize);


    //S_matrix
    void(__stdcall S_matrix:: * SetDir_v1)(S_vector const& dir) = &S_matrix::SetDir;
    void(__stdcall S_matrix:: * SetDir_v2)(S_vector const& v1, S_vector const& v2) = &S_matrix::SetDir;

    void(__stdcall S_matrix:: * SetRot_q)(S_quat const& rot) = &S_matrix::SetRot;
    void(__stdcall S_matrix:: * SetRot_m)(S_matrix const& mat) = &S_matrix::SetRot;

    ReversibleHooks::Install("S_matrix", "Inverse", rebase(0x1002D0B0), &S_matrix::Inverse);
    ReversibleHooks::Install("S_matrix", "RotateByMatrix*", rebase(0x1002CF40), &S_matrix::RotateByMatrix);
    ReversibleHooks::Install("S_matrix", "operator*", rebase(0x1002D3A0), &S_matrix::operator*);
    ReversibleHooks::Install("S_matrix", "operator*=", rebase(0x1002D220), &S_matrix::operator*=);
    ReversibleHooks::Install("S_matrix", "Mul4X4", rebase(0x1002D520), &S_matrix::Mul4X4);
    ReversibleHooks::Install("S_matrix", "Make4X4", rebase(0x1002D780), &S_matrix::Make4X4);  
    ReversibleHooks::Install("S_matrix", "SetDir(struct S_vector const &)", rebase(0x1002D9D0), SetDir_v1);
    ReversibleHooks::Install("S_matrix", "SetDir3(struct S_vector const&, struct S_vector const&)", rebase(0x1002DCF0), &S_matrix::SetDir3);
    ReversibleHooks::Install("S_matrix", "SetDir(struct S_vector const &, struct S_vector const &)", rebase(0x1002E030), SetDir_v2);
    ReversibleHooks::Install("S_matrix", "SetRot3", rebase(0x1002E070), &S_matrix::SetRot3);
    ReversibleHooks::Install("S_matrix", "SetRot_q", rebase(0x1002E1C0), SetRot_q);
    ReversibleHooks::Install("S_matrix", "SetRot_m", rebase(0x1002E1F0), SetRot_m);
    ReversibleHooks::Install("S_matrix", "GetScale", rebase(0x1002E3A0), &S_matrix::GetScale);
    ReversibleHooks::Install("S_matrix", "GetScale2", rebase(0x1002E510), &S_matrix::GetScale2);
    ReversibleHooks::Install("S_matrix", "GetUScale", rebase(0x1002E580), &S_matrix::GetUScale);
}
