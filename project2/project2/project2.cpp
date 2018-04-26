// project2.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include <GL/glut.h>
#include "Vector048.h"
#include <vector> 
#include <string> 
#include <fstream> 
#include <iostream> 

void myDisplay(void);
int g_iCurFrame=0;
int g_mode = 0;
float g_angle = 0 ;
#define POINTNUM 76

CVector048 g_pos[POINTNUM];
CVector048 g_ballpos,g_balldir;//球位置和方向。
float g_ballspeed=0.3;//球速度
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
	update();
	myDisplay();
	glutTimerFunc(1,myTimerFunc,1);
}
void SetRC()
{
	//初始化位置向量
	
	for(int i=0; i<POINTNUM; i++)
	{
		if(i<4){
			g_pos[i][0]=-11.5+i;
			g_pos[i][1]=2;
			g_pos[i][2]=10;
		}
		else if(i<12){
			g_pos[i][0]=-10;
			g_pos[i][1]=3.5-(i-4);
			g_pos[i][2]=10;
		}
		else if(i<14){
			g_pos[i][0]=-12;
			g_pos[i][1]=-0.5-(i-12);
			g_pos[i][2]=10;
		}
		else if(i<16){
			g_pos[i][0]=-8;
			g_pos[i][1]=-0.5-(i-14);
			g_pos[i][2]=10;
		}
		else if(i<20){
			g_pos[i][0]=-5.5+(i-16);
			g_pos[i][1]=2;
			g_pos[i][2]=10;
		}
		else if(i<28){
			g_pos[i][0]=-7.5+(i-20);
			g_pos[i][1]=-4;
			g_pos[i][2]=10;
		}
		else if(i<36){
			g_pos[i][0]=-4;
			g_pos[i][1]=3.5-(i-28);
			g_pos[i][2]=10;
		}
		else if(i<38){
			g_pos[i][0]=2.5+(i-36);
			g_pos[i][1]=4;
			g_pos[i][2]=10;
		}
		else if(i<44){
			g_pos[i][0]=2;
			g_pos[i][1]=1.5-(i-38);
			g_pos[i][2]=10;
		}
		else if(i<54){
			g_pos[i][0]=2.5+(i-44);
			g_pos[i][1]=-4;
			g_pos[i][2]=10;
		}
		else if(i<58){
			g_pos[i][0]=6.5+(i-54);
			g_pos[i][1]=4;
			g_pos[i][2]=10;
		}
		else if(i<66){
			g_pos[i][0]=4.5+(i-58);
			g_pos[i][1]=2;
			g_pos[i][2]=10;
		}
		else if(i<70){
			g_pos[i][0]=6;
			g_pos[i][1]=1.5-(i-66);
			g_pos[i][2]=10;
		}
		else if(i<74){
			g_pos[i][0]=10;
			g_pos[i][1]=1.5-(i-70);
			g_pos[i][2]=10;
		}
		else if(i<76){
			g_pos[i][0]=10.5+(i-74);
			g_pos[i][1]=-2;
			g_pos[i][2]=10;
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
		g_ballpos = g_pos[0];
		g_balldir = g_pos[1]-g_pos[0];
		g_balldir.Normalize();
		g_ballindex=0;
		break;
	case '+':
		g_ballspeed+=0.1;
		break;
	case '-':
		g_ballspeed-=0.1;
		break;
	}
}


void myDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glTranslatef(0,0,-25);
	glRotatef(g_angle,0,1,0);

	glColor3f(0.8,0,0);	
	glPushMatrix();
	glTranslatef(g_ballpos.x,g_ballpos.y,g_ballpos.z);
	glutSolidSphere(0.5,36,36);
	glPopMatrix();
	glColor3f(0.5,0.4,0.1);	
	for(int i=0;i<POINTNUM;i++)
	{
		glPushMatrix();
		//glTranslatef(g_pos[i].x,g_pos[i].y,g_pos[i].z);
		//glutWireSphere(1,4,2);
		glTranslatef(g_pos[i][0],g_pos[i][1],g_pos[i][2]);
		glutWireCube(1);
		glPopMatrix();
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
		char s1[16]="向量相加";
		char s2[16]="向量点乘";
		char s3[16]="向量叉乘";
		//char s4[16]="向量相减";
		//char s5[16]="向量赋值";
		//char s6[16]="向量比较";
		char s4[23]="向量标准化";
		char s5[16]="向量求模";
		char s6[16]="向量投影";
		while(fscanf(fq,"%[^\n]",s)!=EOF)
		{ 
		        CVector048 vec1;
				CVector048 vec2;
				CVector048 vec3;	
				float result;
				
				if(strcmp(s,s1)==0)//向量相加
				{
					fgetc(fq);//吃掉\n
					fscanf(fq,"%f,%f,%f\t%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z,&vec2.x,&vec2.y,&vec2.z);
					vec3=vec1+vec2;
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z,vec3.x,vec3.y,vec3.z);
				
				}
				else if(strcmp(s,s2)==0)//向量点乘
				{
			    	fgetc(fq);
					fscanf(fq,"%f,%f,%f\t%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z,&vec2.x,&vec2.y,&vec2.z);
					result=vec1.dotMul(vec2);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\t%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z,result);
				}
				else if(strcmp(s,s3)==0)//向量叉乘
				{
					fgetc(fq);
					fscanf(fq,"%f,%f,%f\t%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z,&vec2.x,&vec2.y,&vec2.z);
					vec3=vec1.crossMul(vec2);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z,vec3.x,vec3.y,vec3.z);
				}
				else if(strcmp(s,s4)==0)//向量标准化
				{
					fgetc(fq);
					fscanf(fq,"%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z);
					vec2=vec1;
					vec2.Normalize();
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z);
				}
				else if(strcmp(s,s5)==0)//向量求模
				{
					fgetc(fq);
					fscanf(fq,"%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z);
					result=vec1.len();
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t%.2f\n",vec1.x,vec1.y,vec1.z,result);
				}
				else if(strcmp(s,s6)==0)//向量投影
				{
					fgetc(fq);
					fscanf(fq,"%f,%f,%f\t%f,%f,%f\n",&vec1.x,&vec1.y,&vec1.z,&vec2.x,&vec2.y,&vec2.z);
					vec3=vec1.project(vec2);
					FILE *fout;
					fout=fopen("../out.txt","a");
					fprintf(fout,"%s\n",s);
					fprintf(fout,"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f\n",vec1.x,vec1.y,vec1.z,vec2.x,vec2.y,vec2.z,vec3.x,vec3.y,vec3.z);
				}
		}
		fclose(fq);
		fclose(fout);
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
