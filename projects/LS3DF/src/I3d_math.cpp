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

void S_vector::GetNormal(const S_vector& u1, const S_vector& v1, const S_vector& w1)
{
	float ux = w1.x - v1.x, uy = w1.y - v1.y, uz = w1.z - v1.z;
	float vx = u1.x - v1.x, vy = u1.y - v1.y, vz = u1.z - v1.z;
	x = uy * vz - uz * vy;
	y = uz * vx - ux * vz;
	z = ux * vy - uy * vx;
}

double S_vector::Magnitude2() const
{
	return (x * x) + (y * y) + (z * z);
}

double S_vector::Magnitude() const
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

S_vector S_vector::Cross(S_vector const& v) const
{
	return S_vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

double S_vector::CosAngleTo(S_vector const& vec)
{
	float val = (vec.z * z + vec.y * y + vec.x * x
		/ sqrt(
			(x * x
				+ y * y
				+ z * z)
			* (vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)));
	return sqrt((val + 1.0f) * 0.5f);
}

double S_vector::AngleTo(S_vector const& v)
{
	float f = Magnitude() * v.Magnitude();
	if (f < MRG_ZERO) return 0.0f;
	float acos_val = Dot(v) / f;
	return (float)acos(::Min(::Max(acos_val, -1.0f), 1.0f));
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

S_quat S_quat::Slerp(const S_quat& q, float t, bool shorten) const
{
	float cos_a = Dot(q);

	cos_a = Max(-1.0f, Min(1.0f, cos_a));
	float angle = (float)acos(cos_a);
	float sin_a = (float)sin(angle);
	if (I3DFabs(sin_a) < 1e-6f)
		return (*this);

	float flipped = false;
	//select shorter path
	if (/*shorten &&*/ cos_a < 0.0f) {
		angle -= PI;
		flipped = true;
	}

	float inv_sin_a = 1.0f / sin_a;
	float c0 = (float)sin((1.0f - t) * angle) * inv_sin_a;
	float c1 = (float)sin(t * angle) * (flipped ? -inv_sin_a : inv_sin_a);

	S_quat ret;
	ret.x = x * c0 + q.x * c1;
	ret.y = y * c0 + q.y * c1;
	ret.z = z * c0 + q.z * c1;
	ret.w = w * c0 + q.w * c1;
	return ret;
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
	float v3; // st7
	float v4; // st7
	float v5; // st7
	float v6; // st7
	float v7; // st7
	float v8; // st7
	float v9; // st7
	float v10; // st7
	float v13; // st5
	float v14; // st6
	float v15; // st6
	float v16; // st7
	float v17; // st7
	float v18; // st7
	float v19; // st7
	float v20; // st7
	float v21; // st6
	float v22; // st5
	float v23; // st4
	float v24; // st3
	float v25; // st2
	float v26; // st7
	float v27; // st6
	float v28; // st5
	float v29; // st4
	bool v32; // c0
	bool v33; // c3
	float v34; // st7
	float v35; // st7
	float v36; // st7
	float v39; // [esp+0h] [ebp-CCh]
	float v41; // [esp+4h] [ebp-C8h]
	float v43; // [esp+8h] [ebp-C4h]
	float v44; // [esp+8h] [ebp-C4h]
	float v45; // [esp+8h] [ebp-C4h]
	S_vector v46; // [esp+14h] [ebp-B8h] BYREF
	float v49; // [esp+20h] [ebp-ACh]
	float v50; // [esp+24h] [ebp-A8h]
	S_vector v51; // [esp+28h] [ebp-A4h] BYREF
	float v54; // [esp+34h] [ebp-98h]
	float v55; // [esp+38h] [ebp-94h]
	float v56; // [esp+3Ch] [ebp-90h]
	float v57; // [esp+40h] [ebp-8Ch]
	float v58; // [esp+44h] [ebp-88h]
	float v59; // [esp+48h] [ebp-84h]
	float v60; // [esp+4Ch] [ebp-80h]
	float v62; // [esp+54h] [ebp-78h]
	float v63; // [esp+58h] [ebp-74h]
	S_quat v64;
	S_vector v68; // [esp+6Ch] [ebp-60h] BYREF
	S_quat v72; // [esp+7Ch] [ebp-50h] BYREF
	S_matrix v73; // [esp+8Ch] [ebp-40h] BYREF

	S_vector* thirdRow = (S_vector*)((float*)&mat.m_fData[8]);
	S_vector* secRow = (S_vector*)((float*)&mat.m_fData[4]);
	S_vector* firstRow = (S_vector*)((float*)&mat.m_fData[0]);
	S_vector v61 = S_vector((float)firstRow->Magnitude(), (float)secRow->Magnitude(), (float)thirdRow->Magnitude());

	if (v61.z + v61.y + v61.x < 0.0000000099999999f)
	{
	LABEL_28:
		w = 1.0f;
		z = 0.0f;
		y = 0.0f;
		x = 0.0f;
		return;
	}
	if (v61.z * v61.y * v61.x >= 0.0000000099999999f)
	{
		float v11 = 1.0f / v61.x;
		float v12 = 1.0f / v61.y;
		v54 = v12 * mat.m_fData[4];
		v46.x = v54;
		v13 = v12 * mat.m_fData[5];
		v68.x = v54;
		v55 = v13;
		v14 = v12 * mat.m_fData[6];
		v46.y = v55;
		v68.y = v55;
		v56 = v14;
		v46.z = v56;
		v15 = v11 * mat.m_fData[0];
		v68.z = v56;
		v58 = v11 * mat.m_fData[1];
		v59 = v11 * mat.m_fData[2];
		v46.x = v15 + v54;
		v46.y = v55 + v58;
		v46.z = v56 + v59;
		if (fabs(v55) >= 0.9999999900000001f)
		{
			v59 = 0.0f;
			v58 = 0.0f;
			if (v55 <= 0.0f)
			{
				v57 = 0.0f;
				v60 = 1.0f;
				v64.w = 0.0f;
				v64.x = v58;
				v64.y = v59;
				v64.z = 1.0f;
			}
			else
			{
				v57 = 1.0f;
				v60 = 0.0f;
				v64.w = 1.0f;
				v64.x = v58;
				v64.y = v59;
				v64.z = 0.0f;
			}
			goto LABEL_47;
		}

		v51.y = 0.0f;
		v51.x = v56;
		v58 = 0.0f;
		v51.z = -v54;
		v57 = v56;
		v59 = v51.z;
		v16 = v56 * v56 + v51.z * v51.z;
		v49 = v16;
		if (fabs(v16 - 1.0f) >= 0.00000000999999993922529f)
		{
			if (v49 >= 0.0000000099999999f)
			{
				v19 = 1.0f / sqrt(v49);
				v49 = v19;
				v17 = v19 * v51.x;
				v58 = v58 * v49;
				v59 = v59 * v49;
				goto LABEL_46;
			}
			if (v51.x != 0.0f)
			{
				if (v51.x >= 0.0f)
					v17 = 1.0f;
				else
					v17 = -1.0f;
				goto LABEL_46;
			}
			if (v51.z == 0.0f)
			{
				v58 = 1.0f;
			}
			else
			{
				if (v51.z >= 0.0f)
					v18 = 1.0f;
				else
					v18 = -1.0f;
				v59 = v18;
			}
		}
		v17 = v57;
	LABEL_46:
		v51.x = -v17;
		v51.y = -v58;
		v51.z = -v59;
		v44 = acos(v55);
		v64.Make(v51, v44);
	LABEL_47:
		if (fabs(v64.w) < 1.0f)
		{
			v20 = v64.x + v64.x;
			v21 = v64.y + v64.y;
			v22 = v64.z + v64.z;
			v23 = v21 * v64.x;
			v24 = v64.z * v22;
			v25 = v22 * v64.w;
			v51.x = 1.0f - (v64.y * v21 + v24) + v25 + v23;
			v51.y = v23 - v25 + 1.0f - (v64.x * v20 + v24);
			v51.z = v64.y * v22 + v22 * v64.x + v21 * v64.w - v20 * v64.w;
		}
		else
		{
			v51.x = 1.0f;
			v51.y = 1.0f;
			v51.z = 0.0f;
		}
		v57 = v51.x;
		v58 = v51.y;
		v59 = v51.z;
		v51.x = v51.x - v54;
		v51.y = v51.y - v55;
		v51.z = v51.z - v56;
		v26 = v46.x - v54;
		v27 = v46.y - v55;
		v28 = v46.z - v56;
		v29 = v51.z * v28 + v27 * v51.y + v26 * v51.x;
		v50 = v29;
		if (fabs(v29) >= 0.9999999900000001f)
		{
			//v31 = v30;
			v32 = v50 < 0.0f;
			v33 = v50 == 0.0f;
			if (/*(v31 & 0x41) != 0*/ v50 >= 0.0f)
			{
				v50 = 0.0;
				if (v32 || v33)
					v50 = 3.1415927f;

				v72.Make(v68, v50);
			}
			else
			{
				v50 = 0.0f;
				if (v32 || v33)
					v50 = 3.1415927f;
				v46.x = -v54;
				v46.y = -v55;
				v46.z = -v56;

				v72.Make(v46, v50);
			}
			goto LABEL_73;
		}
		v54 = v51.y * v28 - v27 * v51.z;
		v46.x = v54;
		v55 = v51.z * v26 - v28 * v51.x;
		v46.y = v55;
		v56 = v27 * v51.x - v51.y * v26;
		v46.z = v56;
		v34 = v56 * v56 + v55 * v55 + v54 * v54;
		v49 = v34;
		if (fabs(v34 - 1.0f) >= 0.00000000999999993922529f)
		{
			if (v49 >= 0.0000000099999999f)
			{
				v49 = 1.0f / sqrt(v49);
				v35 = v54 * v49;
				v46.y = v46.y * v49;
				v46.z = v46.z * v49;
				goto LABEL_72;
			}
			if (v54 != 0.0f)
			{
				if (v54 >= 0.0f)
					v35 = 1.0f;
				else
					v35 = -1.0f;
				goto LABEL_72;
			}
			if (v56 == 0.0f)
			{
				if (v55 >= 0.0f)
					v46.y = 1.0f;
				else
					v46.y = -1.0f;
			}
			else
			{
				if (v56 >= 0.0f)
					v36 = 1.0f;
				else
					v36 = -1.0f;
				v46.z = v36;
			}
		}
		v35 = v46.z;
	LABEL_72:
		v68.x = -v35;
		v68.y = -v46.y;
		v68.z = -v46.z;
		v45 = acos(v50);
		v72.Make(v68, v45);
	LABEL_73:
		*this = v64 * v72;
		return;
	}
	memcpy(&v73, &mat, sizeof(S_matrix));
	if (v61.x < 0.0000000099999999f)
	{
		v43 = mat.m_fData[9] * mat.m_fData[4] - mat.m_fData[5] * mat.m_fData[8];
		v41 = mat.m_fData[6] * mat.m_fData[8] - mat.m_fData[10] * mat.m_fData[4];
		v39 = mat.m_fData[10] * mat.m_fData[5] - mat.m_fData[6] * mat.m_fData[9];
		v46 = S_vector(v39, v41, v43);

		v73.m_fData[0] = v46.x;
		v73.m_fData[1] = v46.y;
		v73.m_fData[2] = v46.z;

		v50 = v46.x * v46.x + v46.y * v46.y + v46.z * v46.z;
		v49 = v50;
		if (v50 >= 1e-08)
			v3 = fabs(v50 - 1.0f) >= 0.00000000999999993922529f ? sqrt(v50) : 1.0f;
		else
			v3 = 0.0f;
		v61.x = v3;
		if (v3 < 0.0000000099999999f)
		{
			v46.x = 1.0f;
			v46.y = 0.0f;
			v46.z = 0.0f;
			v73.m_fData[0] = 1.0f;
			v73.m_fData[1] = 0.0f;
			v73.m_fData[2] = 0.0f;
			v61.x = 1.0f;
		}
	}
	if (v61.y < 0.0000000099999999f)
	{
		v46.x = mat.m_fData[1] * mat.m_fData[10] - mat.m_fData[9] * mat.m_fData[2];
		v4 = mat.m_fData[2] * mat.m_fData[8];
		v73.m_fData[4] = v46.x;

		v46.y = v4 - mat.m_fData[0] * mat.m_fData[10];
		v5 = mat.m_fData[9] * mat.m_fData[0];

		v73.m_fData[5] = v46.y;

		v46.z = v5 - mat.m_fData[8] * mat.m_fData[1];
		v73.m_fData[6] = v46.z;
		v50 = v46.x * v46.x + v46.z * v46.z + v46.y * v46.y;
		v49 = v50;

		if (v50 >= 1e-08)
			v6 = fabs(v50 - 1.0f) >= 0.00000000999999993922529f ? sqrt(v50) : 1.0f;
		else
			v6 = 0.0;
		v62 = v6;
		if (v6 < 0.0000000099999999f)
		{
			v46.x = 0.0f;
			v46.y = 1.0f;
			v46.z = 0.0f;
			v73.m_fData[4] = 0.0f;
			v73.m_fData[5] = 1.0f;
			v73.m_fData[6] = 0.0f;
			v62 = 1.0f;
		}
	}
	if (v61.z < 0.0000000099999999f)
	{
		v46.x = mat.m_fData[6] * mat.m_fData[1] - mat.m_fData[2] * mat.m_fData[5];
		v7 = secRow->x * mat.m_fData[2];
		v73.m_fData[8] = v46.x;

		v46.y = v7 - mat.m_fData[6] * mat.m_fData[0];
		v8 = mat.m_fData[5] * mat.m_fData[0];
		v73.m_fData[9] = v46.y;

		v46.z = v8 - secRow->x * mat.m_fData[1];
		v73.m_fData[10] = v46.z;

		v50 = v46.x * v46.x + v46.z * v46.z + v46.y * v46.y;
		v49 = v50;
		if (v50 >= 1e-08)
		{
			if (fabs(v50 - 1.0f) >= 0.00000000999999993922529f)
				v9 = sqrt(v50);
			else
				v9 = 1.0f;
		}
		else
		{
			v9 = 0.0f;
		}
		v63 = v9;
		if (v9 < 0.0000000099999999f)
		{
			v46.x = 0.0f;
			v46.y = 0.0f;
			v46.z = 1.0f;
			v73.m_fData[8] = 0.0f;
			v73.m_fData[9] = 0.0f;
			v73.m_fData[10] = 1.0f;
			v63 = 1.0f;
		}
	}

	v49 = (float)((S_vector*)&v73.m_fData[8])->Magnitude();
	v50 = (float)((S_vector*)&v73.m_fData[4])->Magnitude();
	v10 = (float)((S_vector*)&v73.m_fData[0])->Magnitude();
	if (v50 * v49 * v10 < 0.0000000099999999f)
		goto LABEL_28;

	Make(v73);
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

S_quat __stdcall S_quat::RotateByMatrix(S_matrix const& mat)
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
	m_fData[5] = -(m_fData[2] * dir.x - m_fData[0] * dir.z);
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
	float v6 = m_fData[10] * v2.z + m_fData[9] * v2.y + m_fData[8] * v2.x;
	S_vector v30 = S_vector(v6 * m_fData[8], v6 * m_fData[9], v6 * m_fData[10]);
	S_vector v27 = v2 - v30;

	if (v27.z * v27.z + v27.y * v27.y + v27.x * v27.x < 0.0000000099999999f)
	{
		S_vector v33 = S_vector(m_fData[8] * m_fData[10],
			m_fData[9] * m_fData[10],
			m_fData[10] * m_fData[10]);

		v30 = S_vector(-v33.x, -v33.y, 1.0f - v33.z);
		v27 = v30;

		if (v30.x * v30.x + v30.y * v30.y + v30.z * v30.z < 0.0000000099999999f)
		{
			v33 = S_vector(m_fData[8] * m_fData[9],
				m_fData[9] * m_fData[9],
				m_fData[10] * m_fData[9]);

			S_vector v30 = S_vector(-v33.x, 1.0f - v33.y, -v33.z);
			v27 = v30;

			if (v33.Dot(v33) < 0.0000000099999999f)
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