#include "stdafx.h"

#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include <GL/glut.h>
#include "Vector048.h"
#include "Matrix048.h"
#include <vector> 
#include <string> 
#include <fstream> 
#include <iostream> 

void myDisplay(void);
#define POINTNUM 152//一共152个点，但编号只到151
#define CIRCLENUM 20
#define pi 3.1416
CVector048 allpos[POINTNUM*CIRCLENUM];//每个关键点及其周围环绕的小圆环的位置向量
CVector048 circlepos[CIRCLENUM];//每个小圆环的位置向量
CVector048 pointpos[POINTNUM];//每个关键点的位置向量
CVector048 robotPos,robotDir;//机器人位置、方向向量
CVector048 eyePos;
int lockMode=0,travelMode=0;//lockmode0自由，lockmode1锁定，lockmode2叠加；travelmode0欧拉，travelmode1子坐标系
int run;//每次display时run++；每+20次改变一下机器人形态
int drawmode;//选择圆环或管道绘制方式
int robotIndex;//机器人现在在哪个点后
float modelangle;//模型整体旋转的角度
float robotspeed=0.1;
float distance=5;//视点到机器人的距离，可由坐标变换矩阵乘法获得眼坐标系下机器人位置坐标，计算长度即可得
float rx=-25,ry=0,rz=0;
float mspeed=5,rspeed=1;//视点的移动速度和旋转速度
float g_IEyeMat[16]={1,0,0,0,
					 0,1,0,0,
					 0,0,1,0,
					 0,0,0,1},
	  g_EyeMat[16]={1,0,0,0,
					0,1,0,0,
					0,0,1,0,
					0,0,0,1};
FILE *fq;

void SetRC()
{
	//定义了一个圆的路径。
	for(int i=0; i<CIRCLENUM; i++)
	{
		float angle = i*2*3.14/(CIRCLENUM-1);
		circlepos[i].x = 0;
		circlepos[i].y = 1*cos(angle);
		circlepos[i].z = 1*sin(angle);
	}
	//初始化位置向量
	float R=2,seta=0;
	int midlast = 57;//第一段的最后一个点
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
		float rotang = 0;
		if(i!=POINTNUM-1&&i!=midlast&&i!=8&&i!=25&&i!=30&&i!=35&&i!=44&&i!=74&&i!=77&&i!=108&&i!=117&&i!=132&&i!=140)
		{
			dir = pointpos[(i+1)%POINTNUM]-pointpos[i];				
		}
		else
		{
			dir = pointpos[i] - pointpos[(i+POINTNUM-1)%POINTNUM];				
		}
		dir.Normalize();//方向
		rotang = acos(dir.x);
		if(dir.y<0) rotang = -rotang;
		mat.SetRotate(rotang,2);//设置为旋转矩阵。
		mat[12] = pointpos[i].x;	//设置平移部分。
		mat[13] = pointpos[i].y;
		mat[14] = pointpos[i].z;
		for(int j=0;j<CIRCLENUM;j++)
		{
			int index = i*CIRCLENUM+j;
			allpos[index] = mat.MulPosition(circlepos[j]);
		}
	}
	robotPos = pointpos[0];
	robotDir = pointpos[1]-pointpos[0];
	robotDir.Normalize();
	eyePos.x=0; eyePos.y=500; eyePos.z=1000;
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

void update()//场景更新。
{
	if(robotspeed>0)
	{
		if(robotIndex<=POINTNUM-1)
		{
			float leftlen = (pointpos[(robotIndex+1)%POINTNUM] - robotPos).len();//机器人现在位置与下一个关键点之间相隔的距离
			if(leftlen>robotspeed)//如果是距离大，则按速度移动
			{
				robotPos = robotPos + robotDir * robotspeed;
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
				robotPos = pointpos[robotIndex] + robotDir * (temp-leftlen);
			}
		}
	}
	else if(robotspeed<0)
	{
		if(robotIndex>=0)
		{
			float leftlen = (robotPos - pointpos[robotIndex]).len();//小球现在位置与上一个关键点之间相隔的距离
			if(leftlen>(-robotspeed))//如果是距离大，则按速度移动
				robotPos = robotPos + robotDir * robotspeed;
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
				robotPos = pointpos[robotIndex] + robotDir * (temp+leftlen);
				robotIndex=tempindex;
			}
		}
	}
}

void SetView()
{
	if(travelmode==0)//子坐标方式
	{
		glLoadMatrixf(g_EyeMat);//travelMode=0，把当前矩阵设置为g_EyeMat
	}
	else if(travelmode==1) //欧拉角方式
	{
		glRotatef(-rz,0,0,1);	//世界坐标系到摄像机坐标系
		glRotatef(-rx,1,0,0);
		glRotatef(-ry,0,1,0);
		glTranslatef(-eyePos.x,-eyePos.y,-eyePos.z);
	}
	if(lockMode==0)
	{
		glTranslatef(0,0,-25);//?后移干嘛 写完再看一下
		glRotatef(modelangle,0,1,0);
	}  
	else if(lockMode==1)
	{
		CMatrix048 mat;
		mat.SetRotate(pi/2);
		CVector048 pos = robotPos - robotDir*distance;
		CVector048 updir = mat.MulVector(robotDir);
		gluLookAt(pos.x,pos.y,pos.z,robotPos.x,robotPos.y,robotPos.z,updir.x,updir.y,updir.z);
	}
}

void DrawRobot(int run)
{

}

void renderWorld()
{
	glColor3f(0.8,0,0);	
	glPushMatrix();
	glTranslatef(robotPos.x,robotPos.y,robotPos.z);//绘制移动的小机器人
	int tangle;
	tangle = acos(robotDir.x)*180/3.14;
	//if tangle>10 插值
	if(robotDir.y<0) 
		tangle = -tangle;
	glRotatef(tangle,0,0,1);
	//glutSolidSphere(0.5,36,36);
	if(run%20==0) run = 1-run;
	DrawRobot(run);
	glPopMatrix();

	if(g_mode==0)//曲线模式
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		//glColor3f(0.5,0.4,0.1);	
		//glColor3f(0.9961,0.26274,0.3961);
		glColor3f(0.5137,0.6863,0.6078);
		for(int i=0;i<POINTNUM;i++)
		{
			CVector048 dir;
			float rotang = 0;


			if(i!=POINTNUM-1&&i!=midlast&&i!=8&&i!=25&&i!=30&&i!=35&&i!=44&&i!=74&&i!=77&&i!=108&&i!=117&&i!=132&&i!=140)
			{
				dir = pointpos[(i+1)%POINTNUM]-pointpos[i];				
			}
			else
			{
				dir = pointpos[i] - pointpos[(i+POINTNUM-1)%POINTNUM];				
			}
			dir.Normalize();//方向
			rotang = acos(dir.x)*180/3.14;
			if(dir.y<0) rotang = -rotang;

			glPushMatrix();
			glTranslatef(pointpos[i].x,pointpos[i].y,pointpos[i].z);
			glRotatef(rotang,0,0,1);
			glBegin(GL_LINE_STRIP);		//画小圆圈
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
	else if(g_mode==1)
	{		
		int midlast = 57;//第一笔画最后一点。
		CMatrix048 mat;
		float lastrotang = 0;
		
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glBegin(GL_TRIANGLE_STRIP);
	
		for(int i=0;i<POINTNUM-1;i++)
		{
			if(i==midlast||i==8||i==25||i==30||i==35||i==44||i==74||i==77||i==108||i==117||i==132||i==140||i==151)//第一笔画结束，第二笔画开始
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

void myDisplay(void)
{
	static int run=0;//机器人形态
	run++;
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	SetView(); 
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
	bool bChange=false;
	switch(key)
	{
	case 'w':
		if(lockMode==1)
			break;
		//eyePos.y+=mspeed;
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,-mspeed,0);//先把模型下移
			//eyePos.y+=mspeed;
			glMultMatrixf(g_EyeMat);//切换到眼坐标系，相当于眼上移
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);//模型矩阵放到g_EyeMat中
			glPopMatrix();
		}
		else
		{
			eyePos.x+=g_IEyeMat[4]*mspeed;
			eyePos.y+=g_IEyeMat[5]*mspeed;
			eyePos.z+=g_IEyeMat[6]*mspeed;
		}
		break;
	case 's':
		if(lockMode==1)
			break;
		//eyePos.y-=mspeed;	
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,mspeed,0);
			//eyePos.y-=mspeed;	
			glMultMatrixf(g_EyeMat);//切换到眼坐标系，相当于眼上移
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);//模型矩阵放到g_EyeMat中
			
			glPopMatrix();
		}
		else
		{
			eyePos.x-=g_IEyeMat[4]*mspeed;
			eyePos.y-=g_IEyeMat[5]*mspeed;
			eyePos.z-=g_IEyeMat[6]*mspeed;
		}
		
		break;
	case 'a':
		if(lockMode==1)
			break;
		//eyePos.x-=mspeed;
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
		
		break;
	case 'd':
		if(lockMode==1)
			break;
		//eyePos.x+=mspeed;
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
		
		break;
	case 'q':
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
		//eyePos.z-=mspeed;
		
		break;
	case 'e':
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
		//eyePos.z+=mspeed;
		
		break;
	case 'i':
		if(lockMode==1)
			break;
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
		break;
	case 'k':
		if(lockMode==1)
			break;
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
		
		break;
	case 'j':
		if(lockMode==1)
			break;
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
		
		break;
	case 'l':
		if(lockMode==1)
			break;
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
		
		break;
	case 'u':
		if(lockMode==1)
			break;
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
		
		break;
	case 'o':
		if(lockMode==1)
			break;
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
		
		break;
	case '1':
		if(lockMode==0||lockMode==2)
			lockMode=1;
		if(lockMode==1)
			lockMode=0;
		break;
	case '2':
		if(lockMode==0||lockMode==1)
			lockMode=2;
		if(lockMode==2)
			lockMode=0;
		break;
	case '3':
		travelMode = 1;
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();		
			glRotatef(-rz,0,0,1);
			glRotatef(-rx,1,0,0);
			glRotatef(-ry,0,1,0);
			glTranslatef(-eyePos.x,-eyePos.y,-eyePos.z);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			//考虑这里怎么写？？
		}
		printf("travelMode:%d\n",travelMode);
		break;
	case '4':
		travelMode=0;
		if(travelMode==0)
		{
			glPushMatrix();
			glLoadIdentity();		
			glRotatef(-rz,0,0,1);
			glRotatef(-rx,1,0,0);
			glRotatef(-ry,0,1,0);
			glTranslatef(-eyePos.x,-eyePos.y,-eyePos.z);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
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
	if(bChange)//计算视点矩阵的逆矩阵
	{
		glPushMatrix();
		glLoadIdentity();		
		glRotatef(ry,0,1,0);
		glRotatef(rx,1,0,0);
		glRotatef(rz,0,0,1);
		glGetFloatv(GL_MODELVIEW_MATRIX,g_IEyeMat);//g_IEyeMat只在这里发生了改变
		glPopMatrix();
	}
}

void readEul(CEuler &eul)
{
	fscanf(fq,"%f,%f,%f",&eul.h,&eul.p,&eul.b);
}

void readQua(CQuaternion &qua)
{
	fscanf(fq,"%f,%f,%f,%f",&qua.);
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

void writeEul(CEuler &eul)
{
	
}

void writeQua()
{

}

void writeMat(CMatric048 &mat)
{
	for(int i=0;i<15;i++)
	{
		fprintf(fout,"%.2f,",mat[i]);
	}
	fprintf(fout,"%.2f",mat[i]);
}

void writeVec(CVector048 &vec)
{
	fprintf(fout,"%.2f,%.2f,%.2f",vec.x,vec.y,vec.z);
}

int main(int argc, char *argv[])
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
		char s1[16]="欧拉角转换向量";
		char s2[23]="向量转换欧拉角";
		char s3[23]="欧拉角转换四元数";
		char s4[23]="四元数转换欧拉角";
		char s5[23]="欧拉角转换矩阵";
		char s6[23]="矩阵转换欧拉角";
		char s7[16]="矩阵转换四元数";
		char s8[16]="四元数转换矩阵";
		char s9[16]="欧拉角标准化";
		char s10[16]="四元数单位化";
		char s11[16]="四元数相乘";
		char s12[16]="四元数求差";
		char s13[16]="四元数点乘";
		char s14[16]="四元数求逆";
		char s15[32]="四元数求角度和旋转轴";
		char s16[16]="四元数插值";

		while(fscanf(fq,"%[^\n]",s)!=EOF)
		{ 
		        CVector048 vec1;
				CVector048 vec2;
				CVector048 vec3;	
	 			CMatrix048 mat1;
				CMatrix048 mat2;
				CMatrix048 mat3;
				float result;
				int i;
				
				if(strcmp(s,s1)==0)//欧拉角转换向量
				{
					printf("1");
					fgetc(fq);//吃掉\n
					//fscanf(fq,"%f,%f,%f\t%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z,&vec2.x,&vec2.y,&vec2.z);
					for(i=0;i<15;i++)
					{
						fscanf(fq,"%f,",&mat1[i]);
					}
					fscanf(fq,"%f\t",&mat1[15]);
					for(i=0;i<15;i++)
					{
						fscanf(fq,"%f,",&mat2[i]);
					}
					fscanf(fq,"%f\n",&mat2[15]);
					mat3=mat1*mat2;
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					//fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z,vec3.x,vec3.y,vec3.z);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\t",mat1[i]);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat2[i]);
					}
					fprintf(fout,"%.2f\t",mat2[i]);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat3[i]);
					}
					fprintf(fout,"%.2f\n",mat3[i]);
					fclose(fout);
				}
				else if(strcmp(s,s2)==0)//矩阵乘向量
				{
					printf("2");
			    	fgetc(fq);
					//fscanf(fq,"%f,%f,%f\t%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z,&vec2.x,&vec2.y,&vec2.z);
					for(i=0;i<15;i++)
					{
						fscanf(fq,"%f,",&mat1[i]);
					}
					fscanf(fq,"%f\t",&mat1[15]);
					fscanf(fq,"%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z);
					vec2=mat1.MulVector(vec1);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					//fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\t%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z,result);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\t",mat1[i]);
					fprintf(fout,"%.2f,%.2f,%.2f\t",vec1.x,vec1.y,vec1.z);
					fprintf(fout,"%.2f,%.2f,%.2f\n",vec2.x,vec2.y,vec2.z);
					fclose(fout);
				}
				else if(strcmp(s,s3)==0)//矩阵乘位置
				{
					printf("3");
					fgetc(fq);
					for(i=0;i<15;i++)
					{
						fscanf(fq,"%f,",&mat1[i]);
					}
					fscanf(fq,"%f\t",&mat1[15]);
					fscanf(fq,"%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z);
					vec2=mat1.MulPosition(vec1);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\t",mat1[i]);
					fprintf(fout,"%.2f,%.2f,%.2f\t",vec1.x,vec1.y,vec1.z);
					fprintf(fout,"%.2f,%.2f,%.2f\n",vec2.x,vec2.y,vec2.z);
					fclose(fout);
				}
				else if(strcmp(s,s4)==0)//矩阵设置旋转
				{
					printf("4");
					float a;
					fgetc(fq);
					fscanf(fq,"%f\t%f,%f,%f\n",&a,&vec1.x,&vec1.y,&vec1.z);
					mat1.SetRotate(a,vec1);
					vec2.Normalize();
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f\t%.2f,%.2f,%.2f\t",a,vec1.x,vec1.y,vec1.z);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\n",mat1[i]);
					fclose(fout);
				}
				else if(strcmp(s,s5)==0)//矩阵设置平移
				{
					printf("5");
					fgetc(fq);
					fscanf(fq,"%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z);
					mat1.SetTrans(vec1);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t",vec1.x,vec1.y,vec1.z);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\n",mat1[i]);
					fclose(fout);
				}
				else if(strcmp(s,s6)==0)//矩阵设置缩放
				{
					printf("6");
					fgetc(fq);
					fscanf(fq,"%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z);
					mat1.SetScale(vec1);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t",vec1.x,vec1.y,vec1.z);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\n",mat1[i]);
					fclose(fout);
				}
				else if(strcmp(s,s7)==0)//矩阵求逆
				{
					printf("7");
					fgetc(fq);
					for(i=0;i<15;i++)
					{
						fscanf(fq,"%f,",&mat1[i]);
					}
					fscanf(fq,"%f",&mat1[15]);
					mat2=mat1.GetInverse();
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat1[i]);
					}
					fprintf(fout,"%.2f\t",mat1[i]);
					for(i=0;i<15;i++)
					{
						fprintf(fout,"%.2f,",mat2[i]);
					}
					fprintf(fout,"%.2f\n",mat2[i]);
					fclose(fout);
				}
		}
		fclose(fq);
		
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE| GLUT_DEPTH);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);
    glutCreateWindow("第一个OpenGL程序");
    glutDisplayFunc(&myDisplay);
	glutTimerFunc(1,myTimerFunc,1);
	glutReshapeFunc(&myReshape);
	glutKeyboardFunc(&myKeyboardFunc);
	SetRC();
    glutMainLoop();
	
    return 0;
}
