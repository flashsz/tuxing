#pragma once
#include "Vector048.h"

class CQuaternion;

class CMatrix048
{
public:
	CMatrix048(void);
	~CMatrix048(void);
	float m00,m10,m20,m30,m01,m11,m21,m31,m02,m12,m22,m32,m03,m13,m23,m33;
	operator float*(){return &m00;}
	void Set(float *p);
	CMatrix048& operator=(const CMatrix048& p);//矩阵赋值
	CMatrix048 operator*(float d);//矩阵数乘
	CMatrix048 operator*(CMatrix048& p);//矩阵相乘
	void SetRotate(float seta,CVector048 axis);	//设置为旋转矩阵
	void SetTrans(CVector048 trans);		//设置为平移矩阵
	void SetScale(CVector048 p);		//设置为缩放矩阵
	float Inverse();//矩阵求逆,成功返回行列式的值，否则返回0
	CMatrix048 GetInverse();//返回逆矩阵


	CVector048 MulPosition(CVector048& p);	//矩阵乘以空间位置得到一个位置，w默认为1，结果不除w，与*不同。
	CVector048 MulVector(CVector048& p);	//矩阵乘以空间一个向量，w为0	
	CMatrix048& SetRotate(float seta,int axis);//0表示x轴旋转，1表示y轴,2表示z轴

	CEuler ToEuler();
	CQuaternion ToQuaternion();


};

