#pragma once

class CEuler;

class CVector048
{
public:
	CVector048(void);
	~CVector048(void);
	float x,y,z;
	operator float*(){return &x;}
	void Set(float fx,float fy,float fz);
	CVector048& operator=(CVector048& p);//重载运算符
	CVector048 operator+(CVector048& p);
	CVector048 operator-(CVector048& p);
	bool operator==(CVector048& p);
	bool operator!=(CVector048& p);
	CVector048 operator*(float data);
	float dotMul(CVector048 &n);//点乘
	CVector048 crossMul(CVector048 &n);//叉乘
	void Normalize();//正规化
	float len();//模
	CVector048 project(CVector048 &n);//！投影
	CEuler ToEuler();
};