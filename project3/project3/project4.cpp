// 001.cpp : Defines the entry point for the console application.
//

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
int g_iCurFrame=0;
int g_mode = 0;
float g_angle = 0 ;
int look_mode=0;
#define POINTNUM 76
#define CIRCLENUM 20
#define pi 3.1416;
CVector048 g_pos[POINTNUM];
CVector048 g_circle[CIRCLENUM];
CVector048 g_allpos[POINTNUM*CIRCLENUM];
CVector048 g_ballpos,g_balldir;//球位置和方向。
float g_ballspeed=0.1;//球速度
int g_ballindex;//当前球所在的曲线节点位置。


void update()//场景更新。
{
	if(g_ballspeed>0)
	{
		if(g_ballindex<=POINTNUM-1)
		{
			float leftlen = (g_pos[(g_ballindex+1)%POINTNUM] - g_ballpos).len();//小球现在位置与下一个关键点之间相隔的距离
			if(leftlen>g_ballspeed)//如果是距离大，则按速度移动
				g_ballpos = g_ballpos + g_balldir * g_ballspeed;
			else
			{
				g_ballindex++;
				g_ballindex=g_ballindex%POINTNUM;
				float temp=g_ballspeed;
				while(temp>leftlen+(g_pos[(g_ballindex+1)%POINTNUM]-g_pos[g_ballindex]).len())
				{
					temp=temp-(g_pos[(g_ballindex+1)%POINTNUM]-g_pos[g_ballindex]).len();
					g_ballindex++;
					g_ballindex=g_ballindex%POINTNUM;
					//leftlen = (g_pos[g_ballindex] - g_ballpos).len();
				}
				g_balldir = g_pos[(g_ballindex+1)%POINTNUM]-g_pos[g_ballindex];
				g_balldir.Normalize();
				g_ballpos = g_pos[g_ballindex] + g_balldir * (temp-leftlen);				//如果一次移动距离过大，跨越了2个关键路径怎么办？同学自己用代码解决。
			}
		}
	}
	else if(g_ballspeed<0)
	{
		if(g_ballindex>=0)
		{
			float leftlen = (g_ballpos - g_pos[g_ballindex]).len();//小球现在位置与上一个关键点之间相隔的距离
			if(leftlen>(-g_ballspeed))//如果是距离大，则按速度移动
				g_ballpos = g_ballpos + g_balldir * g_ballspeed;
			else
			{		
				//g_ballindex--;
				float temp=g_ballspeed;
				int tempindex=0;
				if(g_ballindex<=0)
					tempindex=POINTNUM-1;
				else
					tempindex=g_ballindex-1;
				while((-temp)>leftlen+(g_pos[g_ballindex]-g_pos[tempindex]).len())
				{
					temp=temp+(g_pos[g_ballindex]-g_pos[tempindex]).len();
					if(g_ballindex<=0)
						g_ballindex=POINTNUM-1;
					else
						g_ballindex--;
					if(g_ballindex<=0)
						tempindex=POINTNUM-1;
					else
						tempindex=g_ballindex-1;
				}
				g_balldir = g_pos[g_ballindex]-g_pos[tempindex];
				g_balldir.Normalize();
				g_ballpos = g_pos[g_ballindex] + g_balldir * (temp+leftlen);				//如果一次移动距离过大，跨越了2个关键路径怎么办？同学自己用代码解决。
				g_ballindex=tempindex;
			}
		}
	}
}
void myTimerFunc(int val)
{
	g_angle+=0.1;
	//g_angle=0;
	update();
	myDisplay();
	glutTimerFunc(1,myTimerFunc,1);
}
void SetRC()
{
	//定义了一个圆的路径。
	for(int i=0; i<CIRCLENUM; i++)
	{
		float angle = i*2*3.14/(CIRCLENUM-1);
		g_circle[i].x = 0;
		g_circle[i].y = 1*cos(angle);
		g_circle[i].z = 1*sin(angle);
	}
	//初始化位置向量
	float R=2,seta=0;
	int midlast = POINTNUM/2-1;//第一段的最后一个点
	for(int i=0; i<POINTNUM; i++)
	{
		//g_pos[i].x = R*cos(seta);
		//g_pos[i].y = R*sin(seta);	
		//g_pos[i].z = 0;
		//if(i<midlast)
		//{
		//	g_pos[i].x-=15;
		//	R+=0.1;
		//	seta +=0.2;
		//}
		//else if(i==midlast)
		//{
		//	g_pos[i].x-=15;
		//	seta = 3.14 - seta;//角度对称，下一个段做准备。
		//}
		//else if(i>midlast)
		//{
		//	g_pos[i].x+=15;
		//	R-=0.1;
		//	seta +=0.2;
		//}			
		if(i<4){
			g_pos[i][0]=-11.5+i;
			g_pos[i][1]=2;
			g_pos[i][2]=0;
		}
		else if(i<12){
			g_pos[i][0]=-10;
			g_pos[i][1]=3.5-(i-4);
			g_pos[i][2]=0;
		}
		else if(i<14){
			g_pos[i][0]=-12;
			g_pos[i][1]=-0.5-(i-12);
			g_pos[i][2]=0;
		}
		else if(i<16){
			g_pos[i][0]=-8;
			g_pos[i][1]=-0.5-(i-14);
			g_pos[i][2]=0;
		}
		else if(i<20){
			g_pos[i][0]=-5.5+(i-16);
			g_pos[i][1]=2;
			g_pos[i][2]=0;
		}
		else if(i<28){
			g_pos[i][0]=-7.5+(i-20);
			g_pos[i][1]=-4;
			g_pos[i][2]=0;
		}
		else if(i<36){
			g_pos[i][0]=-4;
			g_pos[i][1]=3.5-(i-28);
			g_pos[i][2]=0;
		}
		else if(i<38){
			g_pos[i][0]=2.5+(i-36);
			g_pos[i][1]=4;
			g_pos[i][2]=0;
		}
		else if(i<44){
			g_pos[i][0]=2;
			g_pos[i][1]=1.5-(i-38);
			g_pos[i][2]=0;
		}
		else if(i<54){
			g_pos[i][0]=2.5+(i-44);
			g_pos[i][1]=-4;
			g_pos[i][2]=0;
		}
		else if(i<58){
			g_pos[i][0]=6.5+(i-54);
			g_pos[i][1]=4;
			g_pos[i][2]=0;
		}
		else if(i<66){
			g_pos[i][0]=4.5+(i-58);
			g_pos[i][1]=2;
			g_pos[i][2]=0;
		}
		else if(i<70){
			g_pos[i][0]=6;
			g_pos[i][1]=1.5-(i-66);
			g_pos[i][2]=0;
		}
		else if(i<74){
			g_pos[i][0]=10;
			g_pos[i][1]=1.5-(i-70);
			g_pos[i][2]=0;
		}
		else if(i<76){
			g_pos[i][0]=10.5+(i-74);
			g_pos[i][1]=-2;
			g_pos[i][2]=0;
		}
	}		
	CMatrix048 mat;
	for(int i=0; i<POINTNUM; i++)
	{
		CVector048 dir;
		float rotang = 0;
		if(i!=POINTNUM-1&&i!=midlast&&i!=3&&i!=11&&i!=13&&i!=15&&i!=19&&i!=27&&i!=35&&i!=53&&i!=57&&i!=65&&i!=69)
		{
			dir = g_pos[(i+1)%POINTNUM]-g_pos[i];				
		}
		else
		{
			dir = g_pos[i] - g_pos[(i+POINTNUM-1)%POINTNUM];				
		}
		dir.Normalize();//方向
		rotang = acos(dir.x);
		if(dir.y<0) rotang = -rotang;
		mat.SetRotate(rotang,2);//设置为旋转矩阵。
		mat[12] = g_pos[i].x;	//设置平移部分。
		mat[13] = g_pos[i].y;
		mat[14] = g_pos[i].z;
		for(int j=0;j<CIRCLENUM;j++)
		{
			int index = i*CIRCLENUM+j;
			g_allpos[index] = mat.MulPosition(g_circle[j]);
		}
	}
	g_ballpos = g_pos[0];
	g_balldir = g_pos[1]-g_pos[0];
	g_balldir.Normalize();
	glEnable(GL_DEPTH_TEST);
	
}
void myKeyboardFunc(unsigned char key,int x, int y)
{
	switch(key)
	{
	case ' ':	
		g_mode=1-g_mode;		
		break;
	case '1':
		/*g_ballpos = g_pos[0];
		g_balldir = g_pos[1]-g_pos[0];
		g_balldir.Normalize();
		g_ballindex=0;*/
		look_mode=1-look_mode;
		break;
	case '+':
		g_ballspeed+=0.05;
		break;
	case '-':
		g_ballspeed-=0.05;
		break;
	}
}

void DrawRobot(int type)
{
	float size=0.5;
	glTranslatef(0,0.5*size,0);
	glRotatef(90,0,1,0);
	//头
	glPushMatrix();
	//glColor3f(0.9882,0.6157,0.6039);
	glColor3f(0.9961,0.26274,0.3961);
	glScalef(1,1,0.1);
	glutSolidSphere(size*0.5,36,2);
	glPopMatrix();
	
	//身子
	//glColor3f(0.9765,0.8039,00.6784);
	glColor3f(0.9882,0.6157,0.6039);
	glPushMatrix();
	glTranslatef(0,-size,0);
	glScalef(0.3,1,0.2);
	glutSolidCube(size);
	glPopMatrix();
	//胳膊
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
	
	//胳膊
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

	//腿
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
	//腿
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
void myDisplay(void)
{
	static int type=0;
	g_iCurFrame++;
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	if(look_mode==0)
	{
		glTranslatef(0,0,-25);
		glRotatef(g_angle,0,1,0);
	}
	if(look_mode==1)
	{
		CVector048 pos = g_ballpos + g_balldir*(-10);
		gluLookAt(pos.x,pos.y,pos.z,g_ballpos.x,g_ballpos.y,g_ballpos.z,0,1,0);
	}

	/*//绘制小机器人在中心
	if(g_iCurFrame%20==0) type = 1-type;
	DrawRobot(type);*/

	glColor3f(0.8,0,0);	
	glPushMatrix();
	glTranslatef(g_ballpos.x,g_ballpos.y,g_ballpos.z);//绘制移动的小机器人
	int tangle;
	tangle = acos(g_balldir.x)*180/3.14;
	if(g_balldir.y<0) 
		tangle = -tangle;
	glRotatef(tangle,0,0,1);
	//glutSolidSphere(0.5,36,36);
	if(g_iCurFrame%20==0) type = 1-type;
	DrawRobot(type);
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
			if(i!=POINTNUM-1&&i!=37&&i!=3&&i!=11&&i!=13&&i!=15&&i!=19&&i!=27&&i!=35&&i!=53&&i!=57&&i!=65&&i!=69)
			{
				dir = g_pos[(i+1)%POINTNUM]-g_pos[i];				
			}
			else
			{
				dir = g_pos[i] - g_pos[(i+POINTNUM-1)%POINTNUM];				
			}		
			dir.Normalize();//方向
			rotang = acos(dir.x)*180/3.14;
			if(dir.y<0) rotang = -rotang;

			glPushMatrix();
			glTranslatef(g_pos[i].x,g_pos[i].y,g_pos[i].z);
			glRotatef(rotang,0,0,1);
			glBegin(GL_LINE_STRIP);		//画小圆圈
			for(int j=0;j<CIRCLENUM;j++)
				glVertex3fv(g_circle[j]);
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
		int midlast = POINTNUM/2-1;//第一笔画最后一点。
		CMatrix048 mat;
		float lastrotang = 0;
		
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glBegin(GL_TRIANGLE_STRIP);
	
		for(int i=0;i<POINTNUM-1;i++)
		{
			if(i==midlast||i==3||i==11||i==13||i==15||i==19||i==27||i==35||i==53||i==57||i==65||i==69)//第一笔画结束，第二笔画开始
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
				glVertex3fv(g_allpos[index1]);
				glVertex3fv(g_allpos[index2]);
			}			
		}
		glEnd();
	}

	glPopMatrix();
	glutSwapBuffers();
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

int main(int argc, char *argv[])
{
    
	
	FILE *fq;
	
	fq=fopen("../test.txt","rt+");
	if(fq==NULL)
	{
		printf("error");
		fclose(fq);
	}
	else
	{
		char s[100];
		char s1[16]="矩阵相乘";
		char s2[23]="矩阵乘向量";
		char s3[23]="矩阵乘位置";
		//char s4[16]="向量相减";
		//char s5[16]="向量赋值";
		//char s6[16]="向量比较";
		char s4[23]="矩阵设置旋转";
		char s5[23]="矩阵设置平移";
		char s6[23]="矩阵设置缩放";
		char s7[16]="矩阵求逆";
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
				
				if(strcmp(s,s1)==0)//矩阵相乘
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

	/*glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE| GLUT_DEPTH);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);
    glutCreateWindow("第一个OpenGL程序");
    glutDisplayFunc(&myDisplay);
	glutTimerFunc(1,myTimerFunc,1);
	glutReshapeFunc(&myReshape);
	glutKeyboardFunc(&myKeyboardFunc);
	SetRC();
    glutMainLoop();*/
	
    return 0;
}