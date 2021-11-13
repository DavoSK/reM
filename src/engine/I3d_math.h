#pragma once
struct S_matrix;
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

    void __stdcall GetNormal(S_vector const& vec1, S_vector const& vec2, S_vector const& vec3);
    double __stdcall Magnitude2() const;
    double __stdcall Magniture() const;
    void __stdcall  SetNormalized(S_vector const& vec);
    S_vector __stdcall RotateByMatrix(S_matrix const& mat);
    S_vector __stdcall RotateByNormMatrix(S_matrix const& mat);
    S_vector __stdcall Cross(S_vector const& vec);
    double __stdcall CosAngleTo(S_vector const& vec);
    double __stdcall AngleTo(S_vector const& vec);

    static void InitHooks();
    float x; 
    float y;
    float z;
};

struct S_quat
{
    float w;
    float x;
    float y;
    float z;
};

struct S_matrix 
{
    union  
    {
        float m_01, m_02, m_03, m_04,
          m_11, m_12, m_13, m_14,
          m_21, m_22, m_23, m_24;
        float m_fData[16];
    };
};