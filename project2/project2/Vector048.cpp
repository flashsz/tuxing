#include "stdafx.h"
#include "Vector048.h"
#include "math.h"

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
CVector048 CVector048::operator+(CVector048& p)//：：表示右边属于左边
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
float CVector048::dotMul(CVector048 &n)//点乘
{
	return x*n.x+y*n.y+z*n.z;
}
CVector048 CVector048::crossMul(CVector048 &n)//叉乘
{
	CVector048 vec;
	vec.x=y*n.z-z*n.y;
	vec.y=z*n.x-x*n.z;
	vec.z=x*n.y-y*n.x;
	return vec;
}
float CVector048::len()//求向量的模
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
CVector048 CVector048::project(CVector048 &n)//投影向量
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