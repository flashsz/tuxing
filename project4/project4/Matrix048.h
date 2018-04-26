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
	CMatrix048& operator=(const CMatrix048& p);//����ֵ
	CMatrix048 operator*(float d);//��������
	CMatrix048 operator*(CMatrix048& p);//�������
	void SetRotate(float seta,CVector048 axis);	//����Ϊ��ת����
	void SetTrans(CVector048 trans);		//����Ϊƽ�ƾ���
	void SetScale(CVector048 p);		//����Ϊ���ž���
	float Inverse();//��������,�ɹ���������ʽ��ֵ�����򷵻�0
	CMatrix048 GetInverse();//���������


	CVector048 MulPosition(CVector048& p);	//������Կռ�λ�õõ�һ��λ�ã�wĬ��Ϊ1���������w����*��ͬ��
	CVector048 MulVector(CVector048& p);	//������Կռ�һ��������wΪ0	
	CMatrix048& SetRotate(float seta,int axis);//0��ʾx����ת��1��ʾy��,2��ʾz��

	CEuler ToEuler();
	CQuaternion ToQuaternion();


};

