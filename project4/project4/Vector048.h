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
	CVector048& operator=(CVector048& p);//���������
	CVector048 operator+(CVector048& p);
	CVector048 operator-(CVector048& p);
	bool operator==(CVector048& p);
	bool operator!=(CVector048& p);
	CVector048 operator*(float data);
	float dotMul(CVector048 &n);//���
	CVector048 crossMul(CVector048 &n);//���
	void Normalize();//���滯
	float len();//ģ
	CVector048 project(CVector048 &n);//��ͶӰ
	CEuler ToEuler();
};