#include "stdafx.h"

#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include <GL/glut.h>
#include "Vector048.h"
#include "Matrix048.h"
#include "Euler.h"
#include "Quaternion.h"
#include <vector> 
#include <string> 
#include <fstream> 
#include <iostream> 

void myDisplay(void);
#define POINTNUM 151//һ��152���㣬�����ֻ��151
#define CIRCLENUM 20
#define pi 3.1416
CVector048 allpos[POINTNUM*CIRCLENUM];//ÿ���ؼ��㼰����Χ���Ƶ�СԲ����λ������
CVector048 circlepos[CIRCLENUM];//ÿ��СԲ����λ������
CVector048 pointpos[POINTNUM];//ÿ���ؼ����λ������
CVector048 robotPos,robotDir;//������λ�á���������
CVector048 eyePos;
CMatrix048 rotmat,tranmat;
int lockMode=0,travelMode=1;//lockmode0���ɣ�lockmode1������lockmode2���ӣ�travelmode0ŷ����travelmode1������ϵ
int run=0,type=0;//ÿ��displayʱrun++��ÿ+20�θı�һ�»�������̬
int drawmode;//ѡ��Բ����ܵ����Ʒ�ʽ
int robotIndex;//�������������ĸ����
float modelangle;//ģ��������ת�ĽǶ�
float robotspeed=0.01;
float distance=5;//�ӵ㵽�����˵ľ��룬��������任����˷����������ϵ�»�����λ�����꣬���㳤�ȼ��ɵ�
float rx=0,ry=0,rz=0;
float mspeed=0.1,rspeed=0.1;//�ӵ���ƶ��ٶȺ���ת�ٶ�
//float g_IEyeMat[16]={1,0,0,0,
//					 0,1,0,0,
//					 0,0,1,0,
//					 0,0,0,1},
//	  g_EyeMat[16]={1,0,0,0,
//					0,1,0,0,
//					0,0,1,0,
//					0,0,0,1};
CMatrix048 g_EyeMat,g_IEyeMat;
FILE *fq;
FILE *fout;
bool a=0,s=0,d=0,q=0,w=0,e=0,j=0,k=0,l=0,u=0,i=0,o=0;

void SetRC()
{
	//������һ��Բ��·����
	for(int i=0; i<CIRCLENUM; i++)
	{
		float angle = i*2*3.14/(CIRCLENUM-1);
		circlepos[i].x = 0;
		circlepos[i].y = 1*cos(angle);
		circlepos[i].z = 1*sin(angle);
	}
	//��ʼ��λ������
	float R=2,seta=0;
	int midlast = 57;//��һ�ε����һ����
	for(int i=0; i<POINTNUM; i++)
	{
		if(i<9){
			pointpos[i][0]=-12+i*0.5;
			pointpos[i][1]=2;
			pointpos[i][2]=0;
		}
		else if(i<26){
			pointpos[i][0]=-10;
			pointpos[i][1]=4-(i-9)*0.5;
			pointpos[i][2]=0;
		}
		else if(i<31){
			pointpos[i][0]=-12;
			pointpos[i][1]=-(i-26)*0.5;
			pointpos[i][2]=0;
		}
		else if(i<36){
			pointpos[i][0]=-8;
			pointpos[i][1]=-0.5*(i-31);
			pointpos[i][2]=0;
		}
		else if(i<45){
			pointpos[i][0]=-6+(i-36)*0.5;
			pointpos[i][1]=2;
			pointpos[i][2]=0;
		}
		else if(i<58){
			pointpos[i][0]=-7+(i-45)*0.5;
			pointpos[i][1]=-4;
			pointpos[i][2]=0;
		}
		else if(i<75){
			pointpos[i][0]=-4;
			pointpos[i][1]=4-(i-58)*0.5;
			pointpos[i][2]=0;
		}
		else if(i<78){
			pointpos[i][0]=1.5+(i-75)*0.5;
			pointpos[i][1]=4;
			pointpos[i][2]=0;
		}
		else if(i<91){
			pointpos[i][0]=2;
			pointpos[i][1]=2.5-(i-78)*0.5;
			pointpos[i][2]=0;
		}
		else if(i<109){
			pointpos[i][0]=2.5+(i-91)*0.5;
			pointpos[i][1]=-4;
			pointpos[i][2]=0;
		}
		else if(i<118){
			pointpos[i][0]=6+(i-109)*0.5;
			pointpos[i][1]=4;
			pointpos[i][2]=0;
		}
		else if(i<133){
			pointpos[i][0]=4.5+(i-118)*0.5;
			pointpos[i][1]=2;
			pointpos[i][2]=0;
		}
		else if(i<141){
			pointpos[i][0]=6;
			pointpos[i][1]=1.5-(i-133)*0.5;
			pointpos[i][2]=0;
		}
		else if(i<149){
			pointpos[i][0]=10;
			pointpos[i][1]=1.5-(i-141)*0.5;
			pointpos[i][2]=0;
		}
		else if(i<151){
			pointpos[i][0]=10.5+(i-149)*0.5;
			pointpos[i][1]=-2.5;
			pointpos[i][2]=0;
		}
	}		
	CMatrix048 mat;
	for(int i=0; i<POINTNUM; i++)
	{
		CVector048 dir;
		int midlast=57;
		float rotang = 0;
		if(i!=POINTNUM-1&&i!=midlast&&i!=8&&i!=25&&i!=30&&i!=35&&i!=44&&i!=74&&i!=77&&i!=108&&i!=117&&i!=132&&i!=140)
		{
			dir = pointpos[(i+1)%POINTNUM]-pointpos[i];				
		}
		else
		{
			dir = pointpos[i] - pointpos[(i+POINTNUM-1)%POINTNUM];				
		}
		dir.Normalize();//����
		rotang = acos(dir.x);
		if(dir.y<0) rotang = -rotang;
		mat.SetRotate(rotang,2);//����Ϊ��ת����
		mat[12] = pointpos[i].x;	//����ƽ�Ʋ��֡�
		mat[13] = pointpos[i].y;
		mat[14] = pointpos[i].z;
		for(int j=0;j<CIRCLENUM;j++)
		{
			int index = i*CIRCLENUM+j;
			allpos[index] = mat.MulPosition(circlepos[j]);
		}
	}
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
	glGetFloatv(GL_MODELVIEW_MATRIX,g_IEyeMat);//��eyemat��ieyemat����ʼ��Ϊ��λ����
	robotPos = pointpos[0];
	robotDir = pointpos[1]-pointpos[0];
	robotDir.Normalize();
	eyePos.x=0; eyePos.y=0; eyePos.z=25;
	glEnable(GL_DEPTH_TEST);
}

void myReshape(int w,int h)
{	
	GLfloat nRange = 100.0f;
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60,GLfloat(w)/h,1,1000);
	//glOrtho(-20,20,-20,20,-1000,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void update()//�������¡�
{
	if(robotspeed>0)
	{
		if(robotIndex<=POINTNUM-1)
		{
			float leftlen = (pointpos[(robotIndex+1)%POINTNUM] - robotPos).len();//����������λ������һ���ؼ���֮������ľ���
			if(leftlen>robotspeed)//����Ǿ�������ٶ��ƶ�
			{
				robotPos = robotPos + robotDir * robotspeed;
				if(lockMode==2)
				{
					eyePos = eyePos + robotDir * robotspeed;
				}
			}
			else
			{
				robotIndex++;
				robotIndex=robotIndex%POINTNUM;
				float temp=robotspeed;
				while(temp>leftlen+(pointpos[(robotIndex+1)%POINTNUM]-pointpos[robotIndex]).len())
				{
					temp=temp-(pointpos[(robotIndex+1)%POINTNUM]-pointpos[robotIndex]).len();
					robotIndex++;
					robotIndex=robotIndex%POINTNUM;
					//leftlen = (pointpos[robotIndex] - robotPos).len();
				}
				robotDir = pointpos[(robotIndex+1)%POINTNUM]-pointpos[robotIndex];
				robotDir.Normalize();
				if(lockMode==2)
				{
					eyePos = eyePos + (pointpos[robotIndex] + robotDir * (temp-leftlen) - robotPos);
				}
				robotPos = pointpos[robotIndex] + robotDir * (temp-leftlen);	
			}
		}
	}
	else if(robotspeed<0)
	{
		if(robotIndex>=0)
		{
			float leftlen = (robotPos - pointpos[robotIndex]).len();//С������λ������һ���ؼ���֮������ľ���
			if(leftlen>(-robotspeed))//����Ǿ�������ٶ��ƶ�
			{
				robotPos = robotPos + robotDir * robotspeed;
				if(lockMode==2)
				{
					eyePos = eyePos + robotDir * robotspeed;
				}
			}
			else
			{		
				//robotIndex--;
				float temp=robotspeed;
				int tempindex=0;
				if(robotIndex<=0)
					tempindex=POINTNUM-1;
				else
					tempindex=robotIndex-1;
				while((-temp)>leftlen+(pointpos[robotIndex]-pointpos[tempindex]).len())
				{
					temp=temp+(pointpos[robotIndex]-pointpos[tempindex]).len();
					if(robotIndex<=0)
						robotIndex=POINTNUM-1;
					else
						robotIndex--;
					if(robotIndex<=0)
						tempindex=POINTNUM-1;
					else
						tempindex=robotIndex-1;
				}
				robotDir = pointpos[robotIndex]-pointpos[tempindex];
				robotDir.Normalize();
				if(lockMode==2)
				{
					eyePos = eyePos + (pointpos[robotIndex] + robotDir * (temp+leftlen) - robotPos);
				}
				robotPos = pointpos[robotIndex] + robotDir * (temp+leftlen);
				robotIndex=tempindex;
			}
		}
	}
}

void SetView()
{
	/*if(travelMode==0)
	{
		glPushMatrix();
		glLoadIdentity();
		rx = -asin(g_EyeMat.m21)/pi*180;
		ry = atan2(g_EyeMat.m20, g_EyeMat.m22) / pi * 180;
		rz = atan2(g_EyeMat.m01, g_EyeMat.m11) / pi * 180;
		glRotatef(-rz,0,0,1);
		glRotatef(-rx,1,0,0);
		glRotatef(-ry,0,1,0);
		glGetFloatv(GL_MODELVIEW_MATRIX,rotmat);
		tranmat=rotmat.GetInverse()*g_EyeMat;
		eyePos.x = -tranmat.m03;
		eyePos.y = -tranmat.m13;
		eyePos.z = -tranmat.m23;
		glPopMatrix();
	}*/

	if(lockMode==1)
	{
		CMatrix048 mat;
		mat.SetRotate(pi/2,2);
		CVector048 pos;
		CVector048 temp;
		//distance=(eyePos-robotPos).len();
		temp=robotDir*distance;
		eyePos = robotPos - temp;
		CVector048 updir = mat.MulVector(robotDir);
		gluLookAt(eyePos.x,eyePos.y,eyePos.z,robotPos.x,robotPos.y,robotPos.z,updir.x,updir.y,updir.z);
		glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
		rx = -asin(g_EyeMat.m21)/pi*180;
		ry = atan2(g_EyeMat.m20, g_EyeMat.m22) / pi * 180;
		rz = atan2(g_EyeMat.m01, g_EyeMat.m11) / pi * 180;//��rx/y/z/eyePos��������ȷ
		glPushMatrix();
		glLoadIdentity();		
		glRotatef(ry,0,1,0);
		glRotatef(rx,1,0,0);
		glRotatef(rz,0,0,1);
		glGetFloatv(GL_MODELVIEW_MATRIX,g_IEyeMat);//g_IEyeMatֻ�����﷢���˸ı�
		glPopMatrix();
	}
	else
	{
			glRotatef(-rz,0,0,1);	//��������ϵ�����������ϵ
			glRotatef(-rx,1,0,0);
			glRotatef(-ry,0,1,0);
			glTranslatef(-eyePos.x,-eyePos.y,-eyePos.z);
	}
}

void DrawRobot(int type)
{
	float size=0.5;
	glTranslatef(0,0.5*size,0);
	glRotatef(90,0,1,0);
	//ͷ
	glPushMatrix();
	//glColor3f(0.9882,0.6157,0.6039);
	glColor3f(0.9961,0.26274,0.3961);
	glScalef(1,1,0.1);
	glutSolidSphere(size*0.5,36,2);
	glPopMatrix();
	
	//����
	//glColor3f(0.9765,0.8039,00.6784);
	glColor3f(0.9882,0.6157,0.6039);
	glPushMatrix();
	glTranslatef(0,-size,0);
	glScalef(0.3,1,0.2);
	glutSolidCube(size);
	glPopMatrix();
	//�첲
	//glColor3f(0.7843,0.7843,0.6627);
	glColor3f(0.9765,0.8039,00.6784);
	glPushMatrix();
	glTranslatef(-size*0.7,-size*0.3,0);
	/*if(type==0)
		glRotatef(-45,1,0,0);
	else
		glRotatef(45,1,0,0);*/
	glTranslatef(0,-size*0.5,0);
	glScalef(1,0.2,0.2);
	glutSolidCube(size);
	glPopMatrix();
	
	//�첲
	//glColor3f(0.7843,0.7843,0.6627);
	glColor3f(0.9765,0.8039,00.6784);
	glPushMatrix();
	glTranslatef(size*0.7,-size*0.3,0);
	/*if(type==0)
		glRotatef(45,1,0,0);
	else
		glRotatef(-45,1,0,0);*/
	glTranslatef(0,-size*0.5,0);
	glScalef(1,0.2,0.2);
	glutSolidCube(size);
	glPopMatrix();

	//��
	//glColor3f(0.5137,0.6863,0.6078);
	glColor3f(0.7843,0.7843,0.6627);
	glPushMatrix();
	glTranslatef(-size*0.2,-size*1.5,0);
	if(type==0)
		glRotatef(10,1,0,0);
	else
		glRotatef(-10,1,0,0);
	glTranslatef(0,-size*0.5,0);
	glScalef(0.2,1,0.2);
	glutSolidCube(size);
	glPopMatrix();
	//��
	//glColor3f(0.5137,0.6863,0.6078);
	glColor3f(0.7843,0.7843,0.6627);
	glPushMatrix();
	glTranslatef(size*0.2,-size*1.5,0);
	if(type==0)
		glRotatef(-10,1,0,0);
	else
		glRotatef(10,1,0,0);
	glTranslatef(0,-size*0.5,0);
	glScalef(0.2,1,0.2);
	glutSolidCube(size);
	glPopMatrix(); 
}

void renderWorld()
{
	glColor3f(0.8,0,0);	
	glPushMatrix();
	glTranslatef(robotPos.x,robotPos.y,robotPos.z);//�����ƶ���С������
	int tangle;
	tangle = acos(robotDir.x)*180/3.14;
	//if tangle>10 ��ֵ
	if(robotDir.y<0) 
		tangle = -tangle;
	glRotatef(tangle,0,0,1);
	//glutSolidSphere(0.5,36,36);
	if(run%20==0) type = 1-type;
	//printf("run=%d\n",run);
	DrawRobot(type);
	glPopMatrix();

	if(drawmode==0)//����ģʽ
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		//glColor3f(0.5,0.4,0.1);	
		//glColor3f(0.9961,0.26274,0.3961);
		glColor3f(0.5137,0.6863,0.6078);
		for(int i=0;i<POINTNUM;i++)
		{
			CVector048 dir;
			int midlast=57;
			float rotang = 0;


			if(i!=POINTNUM-1&&i!=midlast&&i!=8&&i!=25&&i!=30&&i!=35&&i!=44&&i!=74&&i!=77&&i!=108&&i!=117&&i!=132&&i!=140)
			{
				dir = pointpos[(i+1)%POINTNUM]-pointpos[i];				
			}
			else
			{
				dir = pointpos[i] - pointpos[(i+POINTNUM-1)%POINTNUM];				
			}
			dir.Normalize();//����
			rotang = acos(dir.x)*180/3.14;
			if(dir.y<0) rotang = -rotang;

			glPushMatrix();
			glTranslatef(pointpos[i].x,pointpos[i].y,pointpos[i].z);
			glRotatef(rotang,0,0,1);
			glBegin(GL_LINE_STRIP);		//��СԲȦ
			for(int j=0;j<CIRCLENUM;j++)
				glVertex3fv(circlepos[j]);
			glEnd();
			glPopMatrix();
			/*glBegin(GL_LINE_STRIP);		
			for(int j=0;j<CIRCLENUM;j++)
				glVertex3fv(g_allpos[i*CIRCLENUM+j]);*/
			glEnd();
		}
	}
	else if(drawmode==1)
	{		
		int midlast = 57;//��һ�ʻ����һ�㡣
		CMatrix048 mat;
		float lastrotang = 0;
		
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glBegin(GL_TRIANGLE_STRIP);
	
		for(int i=0;i<POINTNUM-1;i++)
		{
			if(i==midlast||i==8||i==25||i==30||i==35||i==44||i==74||i==77||i==108||i==117||i==132||i==140||i==151)//��һ�ʻ��������ڶ��ʻ���ʼ
			{
				glEnd();
				glColor4f(0.1,0.1,0.1,0.5);
				//glColor4f(0.5137,0.6863,0.6078,0.5);
				glBegin(GL_TRIANGLE_STRIP);				
			}
			else
			{
				/*glColor3f(0.5,0.4,0.1);*/
				glColor3f(0.5137,0.6863,0.6078);
			}
			for(int j=0;j<CIRCLENUM;j++)
			{
				int index1 = i*CIRCLENUM+j;
				int index2 = index1+CIRCLENUM;
				glVertex3fv(allpos[index1]);
				glVertex3fv(allpos[index2]);
			}			
		}
		glEnd();
	}
}

void Keyboard()
{
	bool bChange=false;
	if(w)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,-mspeed,0);//�Ȱ�ģ������
			//eyePos.y+=mspeed;
			glMultMatrixf(g_EyeMat);//�л���������ϵ���൱��������
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);//ģ�;���ŵ�g_EyeMat��
			glPopMatrix();
		}
		else
		{
			eyePos.x+=g_IEyeMat[4]*mspeed;
			eyePos.y+=g_IEyeMat[5]*mspeed;
			eyePos.z+=g_IEyeMat[6]*mspeed;
		}
	}
	if(s)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,mspeed,0);
			//eyePos.y-=mspeed;	
			glMultMatrixf(g_EyeMat);//�л���������ϵ���൱��������
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);//ģ�;���ŵ�g_EyeMat��
			
			glPopMatrix();
		}
		else
		{
			eyePos.x-=g_IEyeMat[4]*mspeed;
			eyePos.y-=g_IEyeMat[5]*mspeed;
			eyePos.z-=g_IEyeMat[6]*mspeed;
		}
	}
	if(a)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(mspeed,0,0);
			//eyePos.x-=mspeed;
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			eyePos.x-=g_IEyeMat[0]*mspeed;
			eyePos.y-=g_IEyeMat[1]*mspeed;
			eyePos.z-=g_IEyeMat[2]*mspeed;
		}
	}
	if(d)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(-mspeed,0,0);
			//eyePos.x+=mspeed;
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			eyePos.x+=g_IEyeMat[0]*mspeed;
			eyePos.y+=g_IEyeMat[1]*mspeed;
			eyePos.z+=g_IEyeMat[2]*mspeed;
		}
	}
	if(q)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,0,mspeed);
			//eyePos.z-=mspeed;
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
		
			glPopMatrix();
		}
		else
		{
			eyePos.x-=g_IEyeMat[8]*mspeed;
			eyePos.y-=g_IEyeMat[9]*mspeed;
			eyePos.z-=g_IEyeMat[10]*mspeed;
		}
	}
	if(e)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,0,-mspeed);
			//eyePos.z+=mspeed;
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			
			glPopMatrix();
		}
		else
		{
			eyePos.x+=g_IEyeMat[8]*mspeed;
			eyePos.y+=g_IEyeMat[9]*mspeed;
			eyePos.z+=g_IEyeMat[10]*mspeed;
		}
	}
	if(i)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed,1,0,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rx+=rspeed;
			bChange = true;
		}
	}
	if(k)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed,1,0,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rx-=rspeed;
			bChange = true;
		}
	}
	if(j)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed,0,1,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			ry+=rspeed;
			bChange = true;
		}
	}
	if(l)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed,0,1,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			ry-=rspeed;
			bChange = true;
		}
	}
	if(u)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed,0,0,1);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rz+=rspeed;
			bChange = true;
		}
	}
	if(o)
	{
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed,0,0,1);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rz-=rspeed;
			bChange = true;
		}
	}

	if(bChange)//�����ӵ����������
	{
		glPushMatrix();
		glLoadIdentity();		
		glRotatef(ry,0,1,0);
		glRotatef(rx,1,0,0);
		glRotatef(rz,0,0,1);
		glGetFloatv(GL_MODELVIEW_MATRIX,g_IEyeMat);//g_IEyeMatֻ�����﷢���˸ı�
		glPopMatrix();
	}
}

void myDisplay(void)
{
	//static int run=0;//��������̬
	run++;
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	
	SetView(); Keyboard();
	renderWorld();

	glPopMatrix();
	glutSwapBuffers();
} 

void myTimerFunc(int val)
{
	//modelangle+=0.1;
	modelangle=0;
	update();
	myDisplay();
	glutTimerFunc(1,myTimerFunc,1);
}

void myKeyboardFunc(unsigned char key,int x, int y)
{
	switch(key)
	{
	case 'w':
		if(lockMode==1)
			break;
		w=1;
		break;
	case 's':
		if(lockMode==1)
			break;
		s=1;
		break;
	case 'a':
		if(lockMode==1)
			break;
		a=1;
		break;
	case 'd':
		if(lockMode==1)
			break;
		d=1;
		break;
	case 'q':
		if(lockMode==1)
		{
			distance-=mspeed;
			break;
		}
		q=1;
		break;
	case 'e':
		if(lockMode==1)
		{
			distance+=mspeed;
			break;
		}
		e=1;
		break;
	case 'i':
		if(lockMode==1)
			break;
		i=1;
		break;
	case 'k':
		if(lockMode==1)
			break;
		k=1;
		break;
	case 'j':
		if(lockMode==1)
			break;
		j=1;
		break;
	case 'l':
		if(lockMode==1)
			break;
		l=1;
		break;
	case 'u':
		if(lockMode==1)
			break;
		u=1;
		break;
	case 'o':
		if(lockMode==1)
			break;
		o=1;
		break;
	case '1':
		if(lockMode==0||lockMode==2)
		{
			lockMode=1;
			printf("lockMode=%d\n",lockMode);
			CVector048 temp;
			distance=(eyePos-robotPos).len();
			temp=robotDir*distance;
			eyePos = robotPos - temp;
		}
		else if(lockMode==1)
		{
			lockMode=0;
			printf("lockMode=%d\n",lockMode);
		}
		break;
	case '2':
		if(lockMode==0||lockMode==1)
			lockMode=2;
		else if(lockMode==2)
			lockMode=0;
		printf("lockMode=%d\n",lockMode);
		break;
	case '3':
		if(travelMode==1)
			break;
		travelMode = 1;//0->1
		
		/*glPushMatrix();
		glLoadIdentity();
		rx = -asin(g_EyeMat.m21)/pi*180;
		ry = atan2(g_EyeMat.m20, g_EyeMat.m22) / pi * 180;
		rz = atan2(g_EyeMat.m01, g_EyeMat.m11) / pi * 180;
		glPushMatrix();
		glRotatef(-rz,0,0,1);
		glRotatef(-rx,1,0,0);
		glRotatef(-ry,0,1,0);
		
		glGetFloatv(GL_MODELVIEW_MATRIX,rotmat);
		glPopMatrix();
		tranmat=rotmat.GetInverse()*g_EyeMat;
		eyePos.x = -tranmat.m03;
		eyePos.y = -tranmat.m13;
		eyePos.z = -tranmat.m23;
		glPopMatrix();*/

		printf("travelMode:%d\n",travelMode);
		break;
	case '4':
		if(travelMode==1)
			break;
		travelMode=1;

		/*glPushMatrix();
		glLoadIdentity();		
		glRotatef(-rz,0,0,1);
		glRotatef(-rx,1,0,0);
		glRotatef(-ry,0,1,0);
		glTranslatef(-eyePos.x,-eyePos.y,-eyePos.z);
		glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
		glPopMatrix();*/
		printf("travelMode:0\n");
		break;
	case ' ':	
		drawmode=1-drawmode;		
		break;
	case ']':
		mspeed*=1.1;
		printf("mspeed:%.1f\n",mspeed);
		break;
	case '[':
		mspeed*=0.9;
		printf("mspeed:%.1f\n",mspeed);
		break;
	case '}':
		rspeed*=1.1;
		printf("rspeed:%.1f\n",mspeed);
		break;
	case '{':
		rspeed*=0.9;
		printf("rspeed:%.1f\n",mspeed);
		break;
	}
}

void glutKeyboardUpFunc(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'w':
		w = 0;
		break;
	case 'a':
		a = 0;
		break;
	case 's':
		s = 0;
		break;
	case 'd':
		d = 0;
		break;
	case 'q':
		q = 0;
		break;
	case 'e':
		e = 0;
		break;
	case 'i':
		i = 0;
		break;
	case 'j':
		j = 0;
		break;
	case 'k':
		k = 0;
		break;
	case 'l':
		l = 0;
		break;
	case 'u':
		u = 0;
		break;
	case 'o':
		o = 0;
		break;
	default:
		break;
	}
}



void readEul(CEuler &eul)
{
	fscanf(fq,"%f,%f,%f",&eul.h,&eul.p,&eul.b);
}

void readQua(CQuaternion &qua)
{
	fscanf(fq,"%f,%f,%f,%f",&qua.x,&qua.y,&qua.z,&qua.w);
}

void readMat(CMatrix048 &mat)
{
	for(int i=0;i<15;i++)
	{
		fscanf(fq,"%f,",&mat[i]);
	}
	fscanf(fq,"%f",&mat[15]);
}

void readVec(CVector048 &vec)
{
	fscanf(fq,"%f,%f,%f",&vec.x,&vec.y,&vec.z);
}

void writeEul(CEuler &eul,int t)//t=0�����Чλ����t=1�����λС��
{
	if(t==0)
		fprintf(fout,"%g,%g,%g",eul.h,eul.p,eul.b);
	else
		fprintf(fout,"%.2f,%.2f,%.2f",eul.h,eul.p,eul.b);
}


void writeQua(CQuaternion &qua,int t)
{
	if(t==0)
		fprintf(fout,"%g,%g,%g,%g",qua.x,qua.y,qua.z,qua.w);
	else
		fprintf(fout,"%.2f,%.2f,%.2f,%.2f",qua.x,qua.y,qua.z,qua.w);
}

void writeMat(CMatrix048 &mat,int t)
{
	if(t==0)
	{
		int i;
		for(i=0;i<15;i++)
		{
			fprintf(fout,"%g,",mat[i]);
		}
		fprintf(fout,"%g",mat[i]);
	}
	else
	{
		int i;
		for(i=0;i<15;i++)
		{
			fprintf(fout,"%.2f,",mat[i]);
		}
		fprintf(fout,"%.2f",mat[i]);
	}
}

void writeVec(CVector048 &vec,int t)
{
	if(t==0)
		fprintf(fout,"%g,%g,%g",vec.x,vec.y,vec.z);
	else
		fprintf(fout,"%.2f,%.2f,%.2f",vec.x,vec.y,vec.z);
}

void readtest()
{
	fq=fopen("../test.txt","rt+");
	if(fq==NULL)
	{
		printf("error");
		fclose(fq);
	}
	else
	{
		char s[100];
		char s1[23]="ŷ����ת������";
		char s2[23]="����ת��ŷ����";
		char s3[26]="ŷ����ת����Ԫ��";
		char s4[26]="��Ԫ��ת��ŷ����";
		char s5[23]="ŷ����ת������";
		char s6[23]="����ת��ŷ����";
		char s7[23]="����ת����Ԫ��";
		char s8[23]="��Ԫ��ת������";
		char s9[20]="ŷ���Ǳ�׼��";
		char s10[20]="��Ԫ����λ��";
		char s11[16]="��Ԫ�����";
		char s12[16]="��Ԫ�����";
		char s13[16]="��Ԫ�����";
		char s14[16]="��Ԫ������";
		char s15[32]="��Ԫ����ǶȺ���ת��";
		char s16[16]="��Ԫ����ֵ";

		while(fscanf(fq,"%[^\n]",s)!=EOF)
		{ 
		        CVector048 vec1;
				CVector048 vec2;
				CVector048 vec3;	
	 			CMatrix048 mat1;
				CMatrix048 mat2;
				CMatrix048 mat3;
				CEuler eul1;
				CEuler eul2;
				CEuler eul3;
				CQuaternion qua1;
				CQuaternion qua2;
				CQuaternion qua3;
				float result;
				int i;
				
				if(strcmp(s,s1)==0)//ŷ����ת������
				{
					printf("1");
					fgetc(fq);//�Ե�\n
					readEul(eul1);
					fscanf(fq,"\n");
					CVector048 *updir;
					updir=&vec2;
					vec1=eul1.ToVector048(updir);
					vec2.x=updir->x;
					vec2.y=updir->y;
					vec2.z=updir->z;
					//FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeEul(eul1,0);
					fprintf(fout,"\t");
					writeVec(vec1,1);
					fprintf(fout,"\t");
					writeVec(vec2,1);
					fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s2)==0)//����תŷ����
				{
					printf("2");
			    	fgetc(fq);
					readVec(vec1);
					fscanf(fq,"\n");

					eul1=vec1.ToEuler();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeVec(vec1,0);
					fprintf(fout,"\t");
					writeEul(eul1,1);
					fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s3)==0)//ŷ����ת��Ԫ��
				{
					printf("3");
					fgetc(fq);
					readEul(eul1);
					fscanf(fq,"\n");

					qua1=eul1.ToQuaternion();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeEul(eul1,0);
					fprintf(fout,"\t");
					writeQua(qua1,1);
					fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s4)==0)//��Ԫ��תŷ����
				{
					printf("4");
					float a;
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\n");

					eul1=qua1.ToEuler();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0);
					fprintf(fout,"\t");
					writeEul(eul1,1);
					fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s5)==0)//ŷ����ת����
				{
					printf("5");
					fgetc(fq);
					readEul(eul1);
					fscanf(fq,"\n");

					mat1=eul1.ToMatrix();
					
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeEul(eul1,0); fprintf(fout,"\t");
					writeMat(mat1,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s6)==0)//����ת��ŷ����
				{
					printf("6");
					fgetc(fq);
					readMat(mat1);
					fscanf(fq,"\n");

					eul1=mat1.ToEuler();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeMat(mat1,0); fprintf(fout,"\t");
					writeEul(eul1,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s7)==0)//����ת����Ԫ��
				{
					printf("7");
					fgetc(fq);
					readMat(mat1);
					fscanf(fq,"\n");

					qua1=mat1.ToQuaternion();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeMat(mat1,0); fprintf(fout,"\t");
					writeQua(qua1,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s8)==0)//��Ԫ��ת������
				{
					printf("8");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\n");

					mat1=qua1.ToMatrix048();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeMat(mat1,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s9)==0)//ŷ���Ǳ�׼��
				{
					printf("9");
					fgetc(fq);
					readEul(eul1);
					fscanf(fq,"\n");

					eul2=eul1;
					eul2.eulerNormal();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeEul(eul1,0); fprintf(fout,"\t");
					writeEul(eul2,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s10)==0)//��Ԫ����λ��
				{
					printf("10");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\n");

					qua2=qua1;
					qua2.Normalize();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeQua(qua2,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s11)==0)//��Ԫ�����
				{
					printf("11");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\t");
					readQua(qua2);
					fscanf(fq,"\n");

					qua3=qua1*qua2;

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeQua(qua2,0); fprintf(fout,"\t");
					writeQua(qua3,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s12)==0)//��Ԫ�����
				{
					printf("12");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\t");
					readQua(qua2);
					fscanf(fq,"\n");

					qua3=qua1.Div(qua2);

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeQua(qua2,0); fprintf(fout,"\t");
					writeQua(qua3,1); fprintf(fout,"\n");
					fclose(fout);
				}
				
				
				else if(strcmp(s,s13)==0)//��Ԫ�����
				{
					printf("13");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\t");
					readQua(qua2);
					fscanf(fq,"\n");
					float t;

					t=qua1.dotMul(qua2);

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeQua(qua2,0); fprintf(fout,"\t");
					fprintf(fout,"%.2f\n",t);
					fclose(fout);
				}
				else if(strcmp(s,s14)==0)//��Ԫ������
				{
					printf("14");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\n");

					qua2=qua1;
					qua2.Inverse();

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeQua(qua2,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s15)==0)//��Ԫ����ǶȺ���ת��
				{
					float ang;
					printf("15");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\n");

					qua2=qua1;
					qua2.GetAngle(ang,vec1);

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					fprintf(fout,"%.2f\t",ang);
					writeVec(vec1,1); fprintf(fout,"\n");
					fclose(fout);
				}
				else if(strcmp(s,s16)==0)//��Ԫ����ֵ
				{
					float dert;
					printf("16");
					fgetc(fq);
					readQua(qua1);
					fscanf(fq,"\t");
					readQua(qua2);
					fscanf(fq,"\t");
					fscanf(fq,"%f\n",&dert);

					qua3=qua1;
					qua3=qua3.Slerp(qua2,dert);

					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					writeQua(qua1,0); fprintf(fout,"\t");
					writeQua(qua2,0); fprintf(fout,"\t");
					fprintf(fout,"%g\t",dert);
					writeQua(qua3,1); fprintf(fout,"\n");
					fclose(fout);
				}
		}
		fclose(fq);
		
	}
}

int main(int argc, char *argv[])
{
	readtest();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(100, 100);//��Ļ���Ͻ�Ϊ��0��0�� ��λ������
    glutInitWindowSize(400, 400);//���ô��ڴ�С ����Ϊ��λ
    glutCreateWindow("��һ��OpenGL����");
    glutDisplayFunc(&myDisplay);//����������Ͳ��
	glutTimerFunc(1,myTimerFunc,0);//��ͣ�ı�����ת�ĽǶȣ�Ȼ���ظ�����myDisplay�ػ����
	glutReshapeFunc(&myReshape);
	glutKeyboardFunc(&myKeyboardFunc);
	glutKeyboardUpFunc(&glutKeyboardUpFunc);
	SetRC();
    glutMainLoop();
    
	
	

	return 0;
}
