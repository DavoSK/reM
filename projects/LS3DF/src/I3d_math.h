#pragma once
struct S_matrix;
struct S_quat;

struct S_vector 
{
    S_vector(float x, float y, float z);
    S_vector();

    S_vector operator*(float const& scalar);
    void operator*=(float const& scalar);
    void operator/=(float const& scalar);

    S_vector operator*(S_vector const& vec);
    void operator*=(S_vector const& vec);

    S_vector operator*(S_matrix const& mat);
    void operator*=(S_matrix const& mat);

    S_vector operator+(S_vector const& vec);
    void operator+=(S_vector const& vec);

    double Dot(S_vector const& vec);
    S_vector operator-(S_vector const& vec);

    void GetNormal(S_vector const& vec1, S_vector const& vec2, S_vector const& vec3);
    double Magnitude2() const;
    double Magniture() const;
    void SetNormalized(S_vector const& vec);
    S_vector    __stdcall RotateByMatrix(S_matrix const& mat);
    S_vector    __stdcall RotateByNormMatrix(S_matrix const& mat);
    S_vector    __stdcall Cross(S_vector const& vec);
    double      __stdcall CosAngleTo(S_vector const& vec);
    double      __stdcall AngleTo(S_vector const& vec);

    float x; 
    float y;
    float z;
};

static_assert(sizeof(S_vector) == 0xC);

struct S_quat
{
    S_matrix    __stdcall RotationMatrix();
    S_vector    __stdcall GetDir();
    S_quat      __stdcall Slerp(S_quat const& quat, float t, bool unk);

    void        __stdcall Make(S_vector const& axis, float angle);
    void        __stdcall Make(S_matrix const& mat);
    S_quat operator*(S_quat const& quat);
    void        __stdcall Normalize();

    float w;
    float x;
    float y;
    float z;
};

static_assert(sizeof(S_quat) == 0x10);

struct S_matrix 
{
    void        __stdcall Identity();
    bool        __stdcall Inverse(S_matrix const& mat);
    S_matrix    __stdcall RotateByMatrix(S_matrix const& mat);
    S_matrix    __stdcall operator*(S_matrix const& mat);
    void        __stdcall operator*=(S_matrix const& mat);
    S_matrix    __stdcall Mul4X4(S_matrix const& mat);
    void        __stdcall SetDir(S_vector const& dir);
    void        __stdcall SetDir3(S_vector const& v1, S_vector const& v2);
    void        __stdcall SetDir(S_vector const& v1, S_vector const& v2);
    void        __stdcall SetRot3(S_quat const& rot);
    void        __stdcall SetRot(S_quat const& rot);
    void        __stdcall SetRot(S_matrix const& mat);
    S_vector    __stdcall GetScale();
    S_vector    __stdcall GetScale2();
    double      __stdcall GetUScale();

    union  
    {
        float   m_01, m_02, m_03, m_04,
                m_11, m_12, m_13, m_14,
                m_21, m_22, m_23, m_24;
        float m_fData[16];
    };
};

static_assert(sizeof(S_matrix) == 0x40);

namespace I3D_math
{
    void InitHooks();
};