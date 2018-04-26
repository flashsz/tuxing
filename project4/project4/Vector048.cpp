#include "stdafx.h"
#include "Vector048.h"
#include "Euler.h"
#include "math.h"

#define pi 3.1416

CVector048::CVector048(void)
{
	x=0;y=0;z=0;
}


CVector048::~CVector048(void)
{
}
void CVector048::Set(float fx,float fy,float fz)
{
	x=fx;y=fy;z=fz;
}
CVector048& CVector048::operator=(CVector048& p)
{
	x=p.x;y=p.y;z=p.z;
	return *this;
}
CVector048 CVector048::operator+(CVector048& p)//������ʾ�ұ��������
{
	CVector048 vec;
	vec.x = x+p.x;
	vec.y = y+p.y;
	vec.z = z+p.z;
	return vec;
}
CVector048 CVector048::operator-(CVector048& p)
{
	CVector048 vec;
	vec.x = x-p.x;
	vec.y = y-p.y;
	vec.z = z-p.z;
	return vec;
}
bool CVector048::operator==(CVector048& p)
{
	if(x == p.x && y == p.y && z == p.z)
		return 1;
	else
		return 0;
}
//bool Vector3::operator==(const Vector3 &a) const{
  //  return x==a.x && y==a.y && z==a.z;
//}
bool CVector048::operator!=(CVector048& p)
{
	if(x == p.x && y == p.y && z == p.z)
		return 0;
	else
		return 1;
}
float CVector048::dotMul(CVector048 &n)//���
{
	return x*n.x+y*n.y+z*n.z;
}
CVector048 CVector048::crossMul(CVector048 &n)//���
{
	CVector048 vec;
	vec.x=y*n.z-z*n.y;
	vec.y=z*n.x-x*n.z;
	vec.z=x*n.y-y*n.x;
	return vec;
}
float CVector048::len()//��������ģ
{	
	return float(sqrt(x*x+y*y+z*z));
}
void CVector048::Normalize()
{	
	float l=len();	
	x /= l;
	y /= l;
	z /= l;
}
CVector048 CVector048::operator*(float data)
{
	CVector048 vec;
	vec.x = x*data;
	vec.y = y*data;
	vec.z = z*data;
	return vec;
}
CVector048 CVector048::project(CVector048 &n)//ͶӰ����
{
	CVector048 vec;
	float sqrtl=n.x*n.x+n.y*n.y+n.z*n.z;
	float abdot=x*n.x+y*n.y+z*n.z;
	float k=abdot/sqrtl;
	vec.x=k*n.x;
	vec.y=k*n.y;
	vec.z=k*n.z;
	return vec;
}

CEuler CVector048::ToEuler()
{
	CEuler eul;
	CVector048 n,a;//a��ԭ������xozƽ���ͶӰ������n=��0��0��-1������ʼ��λ����
	n.x=0; n.y=0; n.z=-1;
	a.x=x; a.y=0; a.z=z;
	if(x<0)
		eul.h=180*acos(n.dotMul(a)/a.len())/pi;
	else
		eul.h=-180*acos(n.dotMul(a)/a.len())/pi;
	if(y>0)
		eul.p=180*acos(dotMul(a)/(a.len()*len()))/pi;
	else
		eul.p=-180*acos(dotMul(a)/(a.len()*len()))/pi;
	eul.b=0;
	return eul;
}