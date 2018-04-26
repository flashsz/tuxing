#include "StdAfx.h"
#include "Quaternion.h"
#include "math.h"

float pi=3.1416;


CQuaternion::CQuaternion(void)
{
}


CQuaternion::~CQuaternion(void)
{
}

CMatrix048 CQuaternion::ToMatrix048()
{
	CMatrix048 mat;
	mat.m00=1-2*y*y-2*z*z;    mat.m01=2*x*y-2*w*z;    mat.m02=2*x*z+2*w*y;      mat.m03=0;
	mat.m10=2*x*y+2*w*z;      mat.m11=1-2*x*x-2*z*z;  mat.m12=2*y*z-2*w*x;      mat.m13=0;
	mat.m20=2*x*z-2*w*y;      mat.m21=2*y*z+2*w*x;    mat.m22=1-2*x*x-2*y*y;    mat.m23=0;
	mat.m30=0;                mat.m31=0;              mat.m32=0;                mat.m33=1;
	return mat;
}

CEuler CQuaternion::ToEuler()
{
	CEuler eul;
	eul.p=180*asin(2*w*x-2*y*z)/pi;
	if(cos(eul.p)!=0)
	{
		eul.h=180*atan2(2*z*x+2*w*y,1-2*x*x-2*y*y)/pi;
		eul.b=180*atan2(2*x*y+2*w*z,1-2*z*z-2*x*x)/pi;
	}
	else
	{
		eul.h=180*atan2(2*w*y-2*z*x,1-2*y*y-2*z*z)/pi;
		eul.b=0;
	}
	return eul;

}


void CQuaternion::SetAngle(float angle,CVector048 axis) //四元数设置
{
	
	CVector048 nx,ny,nz;
	float axisx,axisy,axisz;
	nx.x=1;nx.y=0;nx.z=0;
	ny.x=0;ny.y=1;ny.z=0;
	nz.x=0;nz.y=0;nz.z=1;
	axisx=axis.dotMul(nx);
	axisy=axis.dotMul(ny);
	axisz=axis.dotMul(nz);

	w=cos(angle/2);
	x=sin(angle/2)*axisx;
	y=sin(angle/2)*axisy;
	z=sin(angle/2)*axisz;
}

CQuaternion& CQuaternion::operator=(const CQuaternion& p) //重载赋值
{
	w=p.w;
	x=p.x;
	y=p.y;
	z=p.z;
	return *this;
}
	
CQuaternion CQuaternion::operator+(const CQuaternion& p) //重载‘+’
{
	w=w+p.w;
	x+=p.x;
	y+=p.y;
	z+=p.z;
	return *this;
}

CQuaternion CQuaternion::operator*(float data) //重载数乘
{
	w*=data;
	x*=data;
	y*=data;
	z*=data;
	return *this;
}
	
CQuaternion CQuaternion::operator*(const CQuaternion&p)//四元数乘法
{
	CQuaternion qua;
	qua.w=w*p.w-x*p.x-y*p.y-z*p.z;
	qua.x=w*p.x+x*p.w+z*p.y-y*p.z;
	qua.y=w*p.y+y*p.w+x*p.z-z*p.x;
	qua.z=w*p.z+z*p.w+y*p.x-x*p.y;
	return qua;
}
	
float CQuaternion::dotMul(const CQuaternion&p) //点乘
{
	float l;
	l=w*p.w+x*p.x+y*p.y+z*p.z;
	return l;
}
	
float CQuaternion::len() //求模
{
	float l;
	l=sqrt(w*w+x*x+y*y+z*z);
	return l;
}
	
CQuaternion CQuaternion::Normalize()	//求标准化
{
	float l;
	l=len();
	w/=l;
	x/=l;
	y/=l;
	z/=l;
	return *this;
}
	
CQuaternion& CQuaternion::Inverse()//求逆四元数,会改变自身。
{
	float l2;//l2表示模的平方
	l2=len()*len();
	w=w/l2;
	x=-x/l2;
	y=-y/l2;
	z=-z/l2;
	return *this;
}
	
CQuaternion CQuaternion::GetInverse()//求逆四元数,不改变自身，生成新的四元数
{
	CQuaternion qua;
	float l2;//l2表示模的平方
	l2=len()*len();
	qua.w=w/l2;
	qua.x=-x/l2;
	qua.y=-y/l2;
	qua.z=-z/l2;
	return qua;
}
	
CQuaternion CQuaternion::Div(const CQuaternion&b) //求差 当前为a,求c=a-b
{
	CQuaternion a,ainv,c;
	a.x=x; a.y=y; a.z=z; a.w=w;
	ainv=a;
	ainv.Inverse();
	c=ainv*b;
	return c;
}
	
void CQuaternion::GetAngle(float& angle,CVector048& axis) //求旋转轴和角度
{
	//?唯一性问题？

	Normalize();
	float anglea,xa,ya,za;
	anglea=2*acos(w);
	angle=180*anglea/pi;
	axis.x=x/sin(anglea/2);
	axis.y=y/sin(anglea/2);
	axis.z=z/sin(anglea/2);
}

CQuaternion &CQuaternion::Power(float t)
{
	float angle;

	angle=acos(w);
	w=cos(2*angle);
	x=x*sin(2*angle)/sin(angle);
	y=y*sin(2*angle)/sin(angle);
	z=z*sin(2*angle)/sin(angle);
	return *this;
}
	
//注意单位化
CQuaternion CQuaternion::Slerp(const CQuaternion& Vend,float t) //插值。从当前四元数插值到Vend四元数,t是参数[0,1]
{
	float angle,k0,k1;
	CQuaternion qua,q,p;
	q.x=x; q.y=y; q.z=z;q.w=w;
	p.x=Vend.x; p.y=Vend.y; p.z=Vend.z; p.w=Vend.w;
	p.Normalize();
	q.Normalize();
	angle=acos(q.w*p.w+q.x*p.x+q.y*p.y+q.z*p.z);
	k0=sin((1-t)*angle)/sin(angle);
	k1=sin(t*angle)/sin(angle);
	qua=q*k0+p*k1;
	return qua;
}

//void CQuaternion::Slerp(const CQuaternion& Vend,int n,float *t,CQuaternion *Result)//插值。一次插值出n个数据。插值参数保存在数组t中，结果返回到数组Result中。
//{
//	Normalize();
//}