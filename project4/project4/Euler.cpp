#include "StdAfx.h"
#include "Euler.h"
#include "Quaternion.h"
#include "math.h"

//float pi;
#define pi 3.1416

CEuler::CEuler(void)
{
	h=0;p=0;b=0;
}


CEuler::~CEuler(void)
{
}

CVector048 CEuler::ToVector048(CVector048 *updir=0)
{
	CVector048 n,vec1,vec2;
	CMatrix048 mat; 

	n.x=0; n.y=0; n.z=-1;
	vec2.x=0; vec2.y=1; vec2.z=0;
	mat=ToMatrix();
	vec1=mat.MulVector(n);
	vec2=mat.MulVector(vec2);
	updir->x=vec2.x; updir->y=vec2.y; updir->z=vec2.z;
	return vec1;
}


CMatrix048 CEuler::ToMatrix()
{
	CMatrix048 mat;
	float ha,pa,ba;
	float ch,sh,cp,sp,cb,sb;
	ha=h*pi/180;
	pa=p*pi/180;
	ba=b*pi/180;
	ch=cos(ha);  sh=sin(ha);
	cp=cos(pa);  sp=sin(pa);
	cb=cos(ba);  sb=sin(ba);
	mat.m00=ch*cb+sh*sp*sb;  mat.m01=-ch*sb+sh*sp*cb;  mat.m02=sh*cp;  mat.m03=0;
	mat.m10=sb*cp;           mat.m11=cb*cp;            mat.m12=-sp;    mat.m13=0;
	mat.m20=-sh*cb+ch*sp*sb; mat.m21=sb*sh+ch*sp*cb;   mat.m22=ch*cp;  mat.m23=0;
	mat.m30=0;               mat.m31=0;                mat.m32=0;      mat.m33=1;
	return mat;
}

CQuaternion CEuler::ToQuaternion()
{
	CQuaternion qua;
	float ha,pa,ba;
	ha=h*pi/180;
	pa=p*pi/180;
	ba=b*pi/180;
	qua.w=cos(ha/2)*cos(pa/2)*cos(ba/2)+sin(ha/2)*sin(pa/2)*sin(ba/2);
	qua.x=cos(ha/2)*sin(pa/2)*cos(ba/2)+sin(ha/2)*cos(pa/2)*sin(ba/2);
	qua.y=sin(ha/2)*cos(pa/2)*cos(ba/2)-cos(ha/2)*sin(pa/2)*sin(ba/2);
	qua.z=cos(ha/2)*cos(pa/2)*sin(ba/2)-sin(ha/2)*sin(pa/2)*cos(ba/2);
	return qua;
}

void CEuler::eulerNormal()
{
	if(p==90||p==-90)
	{
		h=h-b;
		b=0;
	}
	while(h>180)
	{
		h-=360;
	}
	while(h<-180)
		b+=360;
	while(b>180)
	{
		b-=360;
	}
	while(b<-180)
		h+=360;
	while(p>180)
	{
		p-=360;
	}
	while(p<-180)
		p+=360;
	if(p>90)
	{
		h=180-h;
		p=p-90;
		b=180-b;
	}
	if(p<-90)
	{
		h=180-h;
		p=p+90;
		b=180-b;
	}
}

