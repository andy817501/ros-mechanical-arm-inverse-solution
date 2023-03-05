#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

#define RAD2ANG (3.1415926535898/180.0)
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )
double v1[4]={0,0,0,0};
double v2[4]={0,0,0,0};
decltype(v1) &inverseKinematics(double x, double y, double z);
float help_rad(float a,float b ,float c);
int main()
{
    double (&v)[4]=inverseKinematics(40, 0, 0);       //逆解目标为（0，30，0）这个坐标
    printf("j0:%f,j1:%f,j2:%f,j3:%f\r\n", v[0],v[1],v[2],v[3]);
}

decltype(v1)&inverseKinematics(double x, double y, double z)
{
	double a, b, c;
	double L1 = 12.98, L2 = 14.688, L3 = 18.031;//3节手臂的长度
	double m, n, t, q, p;
	double j1, j2, j3, j0;//4个舵机的旋转角度
	double x1, y1, z1;        //逆解后正解算出来的值，看是否与逆解值相等
	char i = 0;
	j0 = atan2(y, x);
	a = x / cos(j0);
	if (x == 0) a = y; //如果x为0，需要交换x，y
	b = z;

	for (j3 = 90; j3 >= -90; j3--)
	{
		j3 *= RAD2ANG;
        // 2 * a*L1*sin(j1) + 2 * b*L1*cos(j1)=;
        // float rx=-cos(j3)*(2*L3*L2)+pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2);
         float rx=-cos(j3)*(2*L3*L2)+pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2);
        j1=help_rad(2*a*L1,2*b*L1,rx);
		// j3 = acos((pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2) - 2 * a*L1*sin(j1) - 2 * b*L1*cos(j1)) / (2 * L2*L3));
		//if (abs(ANG2RAD(j3)) >= 135) { j1 = ANG2RAD(j1); continue; }
		m = L2 * sin(j1) + L3 * sin(j1)*cos(j3) + L3 * cos(j1)*sin(j3);
		n = L2 * cos(j1) + L3 * cos(j1)*cos(j3) - L3 * sin(j1)*sin(j3);
		t = a - L1 * sin(j1);
		p = pow(pow(n, 2) + pow(m, 2), 0.5);
		q = asin(m / p);
		j2 = asin(t / p) - q;
		//if (abs(ANG2RAD(j2)) >= 135) { j1 = ANG2RAD(j1); continue; }

		x1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*cos(j0);
		y1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*sin(j0);
		z1 = L1 * cos(j1) + L2 * cos(j1 + j2) + L3 * cos(j1 + j2 + j3);
		j1 = ANG2RAD(j1);
		j2 = ANG2RAD(j2);
		j3 = ANG2RAD(j3);
		if (x1<(x + 1) && x1 >(x - 1) && y1<(y + 1) && y1 >(y - 1) && z1<(z + 1) && z1 >(z - 1))
		{
			 printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
                v1[0]=ANG2RAD(j0);
                v1[1]=j1;
                v1[2]=j2;
                v1[3]=j3;

            // andy->push_back(ANG2RAD(j0));
            // andy->push_back(j1);
            // andy->push_back(j2);
            // andy->push_back(j3);
            // float z=j3;
            // j3=v1.at(3);
            // printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
            break;
			i = 1;
		}
	}
    // andy->resize(4,0);
	for (j3 = 90; j3 >= -90; j3--)
	{
		j3 *= RAD2ANG;
        // 2 * a*L1*sin(j1) + 2 * b*L1*cos(j1)=;
        // float rx=-cos(j3)*(2*L3*L2)+pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2);
         float rx=-cos(j3)*(2*L3*L2)+pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2);
        j1=help_rad(2*a*L1,2*b*L1,rx);
		m = L2 * sin(j1) + L3 * sin(j1)*cos(j3) + L3 * cos(j1)*sin(j3);
		n = L2 * cos(j1) + L3 * cos(j1)*cos(j3) - L3 * sin(j1)*sin(j3);
		t = a - L1 * sin(j1);
		p = pow(pow(n, 2) + pow(m, 2), 0.5);
		q = asin(m / p);
		j2 = -(asin(t / p) - q);
		//if (abs(ANG2RAD(j2)) >= 135) { j1 = ANG2RAD(j1); continue; }
		x1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*cos(j0);
		y1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*sin(j0);
		z1 = L1 * cos(j1) + L2 * cos(j1 + j2) + L3 * cos(j1 + j2 + j3);
		j1 = ANG2RAD(j1);
		j2 = ANG2RAD(j2);
		j3 = ANG2RAD(j3);
		if (x1<(x + 1) && x1 >(x - 1) && y1<(y + 1) && y1 >(y - 1) && z1<(z + 1) && z1 >(z - 1))
		{
			 printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
			i = 1;
                v2[0]=ANG2RAD(j0);
                v2[1]=j1;
                v2[2]=j2;
                v2[3]=j3;
            break;
		}
	}

	if (i == 0)
    {printf("无解");
    }
    double dleat1=90-(v1[3]);
    double dleat2=90-(v2[3]);
    if(dleat1<=dleat2)
    {
        return v1;
    }
    else{
        return v2;
    }
    // return andy;
}



float help_rad(float a,float b ,float c)
{

	float mom=sqrt((pow(a,2)+pow(b,2)));
	float res=asin(c/mom);
	float delt=atan2(b,a);
	float rad=res-delt;
	return rad;
}