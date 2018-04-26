// 001.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include <GL/glut.h>

void myDisplay(void);
int g_iCurFrame=0;
int g_mode = 0;
float g_angle = 0 ;
#define POINTNUM 76
float pos[POINTNUM][3];
void myTimerFunc(int val)
{
	g_angle++;
	myDisplay();
	glutTimerFunc(1,myTimerFunc,1);//每隔1毫秒再次调用myTimerFunc函数
}
void SetRC()
{
	//初始化位置向量
	
	for(int i=0; i<POINTNUM; i++)//i<76
	{
		//pos[i][0] = 10*cos(5*3.14/POINTNUM*i);
		//pos[i][1] = 10*sin(5*3.14/POINTNUM*i);	
		//pos[i][2] = float(i)*10/POINTNUM;
		if(i<4){
			pos[i][0]=-11.5+i;
			pos[i][1]=2;
			pos[i][2]=10;
		}
		else if(i<12){
			pos[i][0]=-10;
			pos[i][1]=3.5-(i-4);
			pos[i][2]=10;
		}
		else if(i<14){
			pos[i][0]=-12;
			pos[i][1]=-0.5-(i-12);
			pos[i][2]=10;
		}
		else if(i<16){
			pos[i][0]=-8;
			pos[i][1]=-0.5-(i-14);
			pos[i][2]=10;
		}
		else if(i<20){
			pos[i][0]=-5.5+(i-16);
			pos[i][1]=2;
			pos[i][2]=10;
		}
		else if(i<28){
			pos[i][0]=-7.5+(i-20);
			pos[i][1]=-4;
			pos[i][2]=10;
		}
		else if(i<36){
			pos[i][0]=-4;
			pos[i][1]=3.5-(i-28);
			pos[i][2]=10;
		}
		else if(i<38){
			pos[i][0]=2.5+(i-36);
			pos[i][1]=4;
			pos[i][2]=10;
		}
		else if(i<44){
			pos[i][0]=2;
			pos[i][1]=1.5-(i-38);
			pos[i][2]=10;
		}
		else if(i<54){
			pos[i][0]=2.5+(i-44);
			pos[i][1]=-4;
			pos[i][2]=10;
		}
		else if(i<58){
			pos[i][0]=6.5+(i-54);
			pos[i][1]=4;
			pos[i][2]=10;
		}
		else if(i<66){
			pos[i][0]=4.5+(i-58);
			pos[i][1]=2;
			pos[i][2]=10;
		}
		else if(i<70){
			pos[i][0]=6;
			pos[i][1]=1.5-(i-66);
			pos[i][2]=10;
		}
		else if(i<74){
			pos[i][0]=10;
			pos[i][1]=1.5-(i-70);
			pos[i][2]=10;
		}
		else if(i<76){
			pos[i][0]=10.5+(i-74);
			pos[i][1]=-2;
			pos[i][2]=10;
		}
	}
	
	glEnable(GL_DEPTH_TEST);//决定图形的透明层度是否显示
}
void myKeyboardFunc(unsigned char key,int x, int y)//键盘控制，g_mode等于0或1，空格键时改变
{
	switch(key)
	{
	case ' ':	
		if(g_mode<4) g_mode=g_mode++;
		else g_mode=0;
		break;
	}
}
void myDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glTranslatef(0,0,-25);//平移
	glRotatef(g_angle,0,1,0);//旋转
	glColor3f(0.8,0.7,0.2);
	
	if(g_mode==0)//绘制小点
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		for(int i=0;i<POINTNUM;i++)
		{
			float r = 1;
			for(int j=0;j<20;j++)//随机生成小点的位置
			{
				float x = float(rand())/RAND_MAX-0.5;
				float y = float(rand())/RAND_MAX-0.5;
				float z = float(rand())/RAND_MAX-0.5;
				float px = pos[i][0]+x*r;
				float py = pos[i][1]+y*r;
				float pz = pos[i][2]+z*r;
				glVertex3f(px,py,pz);
			}			
		}
		glEnd();
	}
	else if(g_mode==1)//绘制线框圆模型
	{
		for(int i=0;i<POINTNUM;i++)
		{
			glPushMatrix();
			glTranslatef(pos[i][0],pos[i][1],pos[i][2]);
			glutWireSphere(0.75,10,10);//渲染球体，半径，纬线条数，经线条数
			glPopMatrix();
		}
	}
	else if(g_mode==2)//绘制线框立方体模型
	{
		for(int i=0;i<POINTNUM;i++)
		{
			glPushMatrix();
			glTranslatef(pos[i][0],pos[i][1],pos[i][2]);
			glutWireCube(1);
			glPopMatrix();
		}
	}
	else if(g_mode==3)//绘制线框八面体模型
	{
		for(int i=0;i<POINTNUM;i++)
		{
			glPushMatrix();
			glTranslatef(pos[i][0],pos[i][1],pos[i][2]);
			glutWireOctahedron();
			glPopMatrix();
		}
	}
	else if(g_mode==4)//绘制实心立方体模型
	{
		for(int i=0;i<POINTNUM;i++)
		{
			glPushMatrix();
			glTranslatef(pos[i][0],pos[i][1],pos[i][2]);
			glutSolidCube(1);
			glPopMatrix();
		}
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
	gluPerspective(60,GLfloat(w)/h,1,1000);//视角设置，角度，窗口纵横比，最近距离处，最远距离处
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE| GLUT_DEPTH);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(800, 400);
    glutCreateWindow("第一个OpenGL程序");
    glutDisplayFunc(&myDisplay);
	glutTimerFunc(1,myTimerFunc,1);
	glutReshapeFunc(&myReshape);
	glutKeyboardFunc(&myKeyboardFunc);
	SetRC();
    glutMainLoop();
    return 0;
}