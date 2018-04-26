#pragma once
#include "Euler.h"

class CEuler;
class CMatrix048;

class CQuaternion
{
public:
	CQuaternion(void);
	~CQuaternion(void);
	float x,y,z,w;
	void Set(float x,float y,float z,float w);
	operator float*(){return &x;}

	CEuler ToEuler();
	CMatrix048 ToMatrix048();
	void SetAngle(float angle,CVector048 axis); //��Ԫ������
	CQuaternion& operator=(const CQuaternion& p); //���ظ�ֵ
	CQuaternion operator+(const CQuaternion& p); //���ء�+��
	CQuaternion operator*(float data); //��������
	CQuaternion operator*(const CQuaternion&p); //��Ԫ���˷�
	float dotMul(const CQuaternion&p); //���
	float len(); //��ģ
	CQuaternion Normalize();	//���׼��
	CQuaternion& Inverse();//������Ԫ��,��ı�����
	CQuaternion GetInverse();//������Ԫ��,���ı����������µ���Ԫ��
	CQuaternion Div(const CQuaternion&b); //��� ��ǰΪa,��c=a-b
	void GetAngle(float& angle,CVector048& axis); //����ת��ͽǶ�
	CQuaternion Slerp(const CQuaternion& Vend,float t); //��ֵ���ӵ�ǰ��Ԫ����ֵ��Vend��Ԫ��,t�ǲ���[0,1]
	void Slerp(const CQuaternion& Vend,int n,float *t,CQuaternion *Result);//��ֵ��һ�β�ֵ��n�����ݡ���ֵ��������������t�У�������ص�����Result�С�


};

