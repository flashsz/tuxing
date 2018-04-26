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
	mat.m00=1-2*y*y-2*z*z;    mat.m01=2*x*y-2*w*z;    mat.m02=2*x*z+2*w*y;
	mat.m10=2*x*y+2*w*z;      mat.m11=1-2*x*x-2*z*z;  mat.m12=2*y*z-2*w*x;
	mat.m20=2*x*z-2*w*y;      mat.m21=2*y*z+2*w*x;    mat.m22=1-2*x*x-2*y*y;
	return mat;
}

CEuler CQuaternion::ToEuler()
{
	CEuler eul;
	eul.p=asin(2*w*x-2*y*z);
	if(cos(eul.p)!=0)
	{
		eul.h=atan2(2*z*x+2*w*y,1-2*x*x-2*y*y);
		eul.b=atan2(2*x*y+2*w*z,1-2*z*z-2*x*x);
	}
	else
	{
		eul.h=atan2(2*w*y-2*z*x,1-2*y*y-2*z*z);
		eul.b=0;
	}
	return eul;

}


void CQuaternion::SetAngle(float angle,CVector048 axis) //��Ԫ������
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

CQuaternion& CQuaternion::operator=(const CQuaternion& p) //���ظ�ֵ
{
	w=p.w;
	x=p.x;
	y=p.y;
	z=p.z;
	return *this;
}
	
CQuaternion CQuaternion::operator+(const CQuaternion& p) //���ء�+��
{
	w=w+p.w;
	x+=p.x;
	y+=p.y;
	z+=p.z;
	return *this;
}

CQuaternion CQuaternion::operator*(float data) //��������
{
	w*=data;
	x*=data;
	y*=data;
	z*=data;
	return *this;
}
	
CQuaternion CQuaternion::operator*(const CQuaternion&p)//��Ԫ���˷�
{
	w=w*p.w-x*p.x-y*p.y-z*p.z;
	x=w*p.x+x*p.w+y*p.z-z*p.y;
	y=w*p.y+y*p.w+z*p.x-x*p.z;
	z=w*p.z+z*p.w+x*p.y-y*p.x;
}
	
float CQuaternion::dotMul(const CQuaternion&p) //���
{
	float l;
	l=w*p.w+x*p.x+y*p.y+z*p.z;
	return l;
}
	
float CQuaternion::len() //��ģ
{
	float l;
	l=sqrt(w*w+x*x+y*y+z*z);
	return l;
}
	
CQuaternion CQuaternion::Normalize()	//���׼��
{
	float l;
	l=len();
	w/=l;
	x/=l;
	y/=l;
	z/=l;
	return *this;
}
	
CQuaternion& CQuaternion::Inverse()//������Ԫ��,��ı�����
{
	float l2;//l2��ʾģ��ƽ��
	l2=len()*len();
	w=w/l2;
	x=-x/l2;
	y=-y/l2;
	z=-z/l2;
	return *this;
}
	
CQuaternion CQuaternion::GetInverse()//������Ԫ��,���ı����������µ���Ԫ��
{
	CQuaternion qua;
	float l2;//l2��ʾģ��ƽ��
	l2=len()*len();
	qua.w=w/l2;
	qua.x=-x/l2;
	qua.y=-y/l2;
	qua.z=-z/l2;
	return qua;
}
	
CQuaternion CQuaternion::Div(const CQuaternion&b) //��� ��ǰΪa,��c=a-b
{
	CQuaternion binv,c;
	binv=b;
	binv.Inverse();
	c=binv*(*this);
	return c;
}
	
void CQuaternion::GetAngle(float& angle,CVector048& axis) //����ת��ͽǶ�
{
	//?Ψһ�����⣿
	Normalize();
	float anglea,xa,ya,za;
	anglea=2*acos(w);
	angle=180*anglea/pi;
	axis.x=x*sin(anglea/2);
	axis.y=y*sin(anglea/2);
	axis.z=z*sin(anglea/2);
}
	
//ע�ⵥλ��
CQuaternion CQuaternion::Slerp(const CQuaternion& Vend,float t) //��ֵ���ӵ�ǰ��Ԫ����ֵ��Vend��Ԫ��,t�ǲ���[0,1]
{
	Normalize();
}

void CQuaternion::Slerp(const CQuaternion& Vend,int n,float *t,CQuaternion *Result)//��ֵ��һ�β�ֵ��n�����ݡ���ֵ��������������t�У�������ص�����Result�С�
{
	Normalize();
}