#include "stdafx.h"
#include "Matrix048.h"
#include "Quaternion.h"
#include "math.h"
float pi=3.1416;

CMatrix048::CMatrix048(void)
{
}


CMatrix048::~CMatrix048(void)
{
}

CMatrix048& CMatrix048::operator=(const CMatrix048& p)
{
	m00=p.m00;m01=p.m01;m02=p.m02;m03=p.m03;
	m10=p.m10;m11=p.m11;m12=p.m12;m13=p.m13;
	m20=p.m20;m21=p.m21;m22=p.m22;m23=p.m23;
	m30=p.m30;m31=p.m31;m32=p.m32;m33=p.m33;
	return *this;
}

CMatrix048 CMatrix048::operator*(float d)
{
	CMatrix048 mat;
	mat.m00=d*m00;mat.m01=d*m01;mat.m02=d*m02;mat.m03=d*m03;
	mat.m10=d*m10;mat.m11=d*m11;mat.m12=d*m12;mat.m13=d*m13;
	mat.m20=d*m20;mat.m21=d*m21;mat.m22=d*m22;mat.m23=d*m23;
	mat.m30=d*m30;mat.m31=d*m31;mat.m32=d*m32;mat.m33=d*m33;
	return mat;
}

CMatrix048 CMatrix048::operator*(CMatrix048& p)
{
	CMatrix048 mat;
	mat.m00 = m00*p.m00 + m01*p.m10 + m02*p.m20 + m03*p.m30;
	mat.m01 = m00*p.m01 + m01*p.m11 + m02*p.m21 + m03*p.m31;
	mat.m02 = m00*p.m02 + m01*p.m12 + m02*p.m22 + m03*p.m32;
	mat.m03 = m00*p.m03 + m01*p.m13 + m02*p.m23 + m03*p.m33;
	mat.m10 = m10*p.m00 + m11*p.m10 + m12*p.m20 + m13*p.m30;
	mat.m11 = m10*p.m01 + m11*p.m11 + m12*p.m21 + m13*p.m31;
	mat.m12 = m10*p.m02 + m11*p.m12 + m12*p.m22 + m13*p.m32;
	mat.m13 = m10*p.m03 + m11*p.m13 + m12*p.m23 + m13*p.m33;
	mat.m20 = m20*p.m00 + m21*p.m10 + m22*p.m20 + m23*p.m30;
	mat.m21 = m20*p.m01 + m21*p.m11 + m22*p.m21 + m23*p.m31;
	mat.m22 = m20*p.m02 + m21*p.m12 + m22*p.m22 + m23*p.m32;
	mat.m23 = m20*p.m03 + m21*p.m13 + m22*p.m23 + m23*p.m33;
	mat.m30 = m30*p.m00 + m31*p.m10 + m32*p.m20 + m33*p.m30;
	mat.m31 = m30*p.m01 + m31*p.m11 + m32*p.m21 + m33*p.m31;
	mat.m32 = m30*p.m02 + m31*p.m12 + m32*p.m22 + m33*p.m32;
	mat.m33 = m30*p.m03 + m31*p.m13 + m32*p.m23 + m33*p.m33;
	return mat;
}



CVector048 CMatrix048::MulPosition(CVector048& p)	//矩阵乘以空间位置得到一个位置，w默认为1，结果不除w，与*不同。
{
	int i;
	CVector048 vec;
	float *mat = (float*)&m00;
	for( i=0;i<3;i++)
	{
		vec[i] = mat[i]*p.x +mat[4+i]*p.y +mat[8+i]*p.z+mat[12+i];
	}
	return vec;
}

CVector048 CMatrix048::MulVector(CVector048& p)
{
	int i;
	CVector048 vec;
	float *mat = (float*)&m00;
	for( i=0;i<3;i++)
	{
		vec[i] = float(mat[i]*p.x +mat[4+i]*p.y +mat[8+i]*p.z);
	}
	return vec;
}

void CMatrix048::SetRotate(float seta,CVector048 axis)
{
	axis.Normalize();
	seta=3.14159265758/180*seta;
	float c=cos(seta);
	float s=sin(seta);
	m00=axis.x*axis.x*(1-c)+c;              m01=axis.x*axis.y*(1-c)-axis.z*s;          m02=axis.x*axis.z*(1-c)+axis.y*s;  m03=0;
	m10=axis.x*axis.y*(1-c)+axis.z*s;       m11=axis.y*axis.y*(1-c)+c ;                m12=axis.y*axis.z*(1-c)-axis.x*s;  m13=0;
	m20=axis.x*axis.z*(1-c)-axis.y*s;       m21=axis.y*axis.z*(1-c)+axis.x*s;          m22=axis.z*axis.z*(1-c)+c;         m23=0;
	m30=0;                                  m31=0;                                     m32=0;                             m33=1;
}

void CMatrix048::SetTrans(CVector048 trans)
{
	m00=1;  m01=0;  m02=0;  m03=trans.x;
	m10=0;  m11=1;  m12=0;  m13=trans.y;
	m20=0;  m21=0;  m22=1;  m23=trans.z;
	m30=0;  m31=0;  m32=0;  m33=1;
}

void CMatrix048::SetScale(CVector048 p)
{
	m00=p.x;  m01=0;    m02=0;    m03=0;
	m10=0;    m11=p.y;  m12=0;    m13=0;
	m20=0;    m21=0;    m22=p.z;  m23=0;
	m30=0;    m31=0;    m32=0;    m33=1;
}

float CMatrix048::Inverse()
{
	float c00,c01,c02,c03,det;
	c00=m11*m22*m33+m21*m32*m13+m31*m12*m23-m13*m22*m31-m12*m21*m33-m11*m23*m32;
	//c0=m*m*m+m*m*m+m*m*m-m*m*m-m*m*m-m*m*m;
	c01=m10*m22*m33+m20*m32*m13+m30*m12*m23-m13*m22*m30-m12*m20*m33-m10*m23*m32;
	c02=m10*m21*m33+m20*m31*m13+m30*m11*m23-m13*m21*m30-m11*m20*m33-m10*m23*m31;
	c03=m10*m21*m32+m20*m31*m12+m30*m11*m22-m12*m21*m30-m11*m20*m32-m10*m22*m31;
	det=m00*c00+m01*c01+m02*c02+m03*c03;
	return det;
}

CMatrix048 CMatrix048:: GetInverse()
{
	CMatrix048 mat;
	float c00,c01,c02,c03,c10,c11,c12,c13,c20,c21,c22,c23,c30,c31,c32,c33,det;
	c00=m11*m22*m33+m21*m32*m13+m31*m12*m23-m13*m22*m31-m12*m21*m33-m11*m23*m32;
	//c0=m*m*m+m*m*m+m*m*m-m*m*m-m*m*m-m*m*m;
	c01=-(m10*m22*m33+m20*m32*m13+m30*m12*m23-m13*m22*m30-m12*m20*m33-m10*m23*m32);
	c02=m10*m21*m33+m20*m31*m13+m30*m11*m23-m13*m21*m30-m11*m20*m33-m10*m23*m31;
	c03=-(m10*m21*m32+m20*m31*m12+m30*m11*m22-m12*m21*m30-m11*m20*m32-m10*m22*m31);
	c10=-(m01*m22*m33+m21*m32*m03+m31*m02*m23-m03*m22*m31-m02*m21*m33-m01*m23*m32);
	c11=m00*m22*m33+m20*m32*m03+m30*m02*m23-m03*m22*m30-m02*m20*m33-m00*m23*m32;
	c12=-(m00*m21*m33+m20*m31*m03+m30*m01*m23-m03*m21*m30-m01*m20*m33-m00*m23*m31);
	c13=m00*m21*m32+m20*m31*m02+m30*m01*m22-m02*m21*m30-m01*m20*m32-m00*m22*m31;
	c20=m01*m12*m33+m11*m32*m03+m31*m02*m13-m03*m12*m31-m02*m11*m33-m01*m13*m32;
	c21=-(m00*m12*m33+m10*m32*m03+m30*m02*m13-m03*m12*m30-m02*m10*m33-m00*m13*m32);
	c22=m00*m11*m33+m10*m31*m03+m30*m01*m13-m03*m11*m30-m01*m10*m33-m00*m13*m31;
	c23=-(m00*m11*m32+m10*m31*m02+m30*m01*m12-m02*m11*m30-m01*m10*m32-m00*m12*m31);
	c30=-(m01*m12*m23+m11*m22*m03+m21*m02*m13-m03*m12*m21-m02*m11*m23-m01*m13*m22);
	c31=m00*m12*m23+m10*m22*m03+m20*m02*m13-m03*m12*m20-m02*m10*m23-m00*m13*m22;
	c32=-(m00*m11*m23+m10*m21*m03+m20*m01*m13-m03*m11*m20-m01*m10*m23-m00*m13*m21);
	c33=m00*m11*m22+m10*m21*m02+m20*m01*m12-m02*m11*m20-m01*m10*m22-m00*m12*m21;
	det=m00*c00+m01*c01+m02*c02+m03*c03;

	mat.m00=c00/det; mat.m01=c10/det; mat.m02=c20/det; mat.m03=c30/det;
	mat.m10=c01/det; mat.m11=c11/det; mat.m12=c21/det; mat.m13=c31/det;
	mat.m20=c02/det; mat.m21=c12/det; mat.m22=c22/det; mat.m23=c32/det;
	mat.m30=c03/det; mat.m31=c13/det; mat.m32=c23/det; mat.m33=c33/det;

	return mat;
}

CMatrix048& CMatrix048::SetRotate(float seta,int axis)//0表示x轴旋转，1表示y轴,2表示z轴
{
	
	float c = cos(seta);
	float s = sin(seta);
	if(axis==0)
	{
		m00=1;m01=0;m02=0;m03=0;
		m10=0;m11=c;m12=-s;m13=0;
		m20=0;m21=s;m22=c;m23=0;
		m30=0;m31=0;m32=0;m33=1;
	}
	else if(axis==1)
	{
		m00=c;m01=0;m02=s;m03=0;
		m10=0;m11=1;m12=0;m13=0;
		m20=-s;m21=0;m22=c;m23=0;
		m30=0;m31=0;m32=0;m33=1;
	}
	else
	{
		m00=c;m01=-s;m02=0;m03=0;
		m10=s;m11=c;m12=0;m13=0;
		m20=0;m21=0;m22=1;m23=0;
		m30=0;m31=0;m32=0;m33=1;
	}
	return *this;
}

CEuler CMatrix048::ToEuler()
{
	CEuler eul;
	float pa,ha,ba;//弧度制的phb
	pa=asin(-m12);
	ha=atan2(m02,m22);
	ba=atan2(m10,m11);
	eul.b=ba*180/pi;
	eul.p=pa*180/pi;
	eul.h=ha*180/pi;
	return eul;
}

CQuaternion CMatrix048::ToQuaternion()
{
	CQuaternion qua;
	qua.w=0.5*sqrt(m00+m11+m22+1);
	qua.x=(m21-m12)/(4*qua.w);
	qua.y=(m02-m20)/(4*qua.w);
	qua.z=(m10-m01)/(4*qua.w);
	return qua;
}