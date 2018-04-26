#pragma once

#include "Matrix048.h"

class CVector048;
class CMatrix048;
class CQuaternion;

class CEuler
{
public:
	CEuler(void);
	~CEuler(void);
	float h,p,b;
	void Set(float h,float p,float b);
	operator float*(){return &h;}

	CVector048 ToVector048(CVector048 *updir);
	CMatrix048 ToMatrix();
	CQuaternion ToQuaternion();
	void eulerNormal();
};

