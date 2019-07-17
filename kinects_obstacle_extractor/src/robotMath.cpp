


#include "../include/robotMath.h"

/***************************************
 * nearest integer function
 **************************************/
//int round(double x)
//{
//	double temp = x - floor(x);
//
//	if (temp > 0.5)
//	{
//		return floor(x) + 1;
//	}
//	else
//	{
//		return floor(x);
//	}
//}

/***************************************
 * factorial function
 **************************************/
int factorial(int x)
{
	int i;
	int y=1;
	for(i=1;i<=x;i++){
		y = y*i;
	}
	return y;
}

/***************************************
 * function for norm distance in R3
 **************************************/
double norm3(double* a)
{
	double c;
	c = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);

	return c;
}

/***************************************
 * function for distance from a[3] to b[3]
 **************************************/
double dis3(double* a, double* b)
{
	double c;

	c = sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));

	return c;
}

/*********************************************
 * function for dot multiplication y=x1.x2
 ********************************************/
double dot3(double* x1, double* x2)
{
	double y = x1[0]*x2[0] + x1[1]*x2[1] + x1[2]*x2[2];
	return y;
}

/*********************************************
 * function for cross multiplication y=x1��x2
 ********************************************/
void cross3(double* y, double* x1, double* x2)
{
    y[0] = x1[1]*x2[2] - x1[2]*x2[1];
    y[1] = x1[2]*x2[0] - x1[0]*x2[2];
    y[2] = x1[0]*x2[1] - x1[1]*x2[0];

}



/******************************************************************************************
 * ����һԪ���η�������ʵ������		a*x^2 + b*x + c = 0
 * Input:
 * a,b,c:				����ϵ��
 *
 * Output:
 * x:					���̸�
 *
 * return:				��0��������ȷ����-1����������
 * ***************************************************************************************/
int root2(double a, double b, double c, double* x1, double* x2)
{
	if (a != 0)
	{
		double A = b*b - 4*a*c;
		double B =-b/(2*a);
		double C;
		//cout << "A = " << A << endl;
		if (A < 0)
		{
			//DEBUG("error: no real root in <root2>\n");
			return -1;
		}
		else
		{
			C = sqrt(A)/(2*a);
			*x1 = B + C;
			*x2 = B - C;
			return 0;
		}
	}
	else
	{
		// DEBUG("error: not a quadratic equation in <root2>\n");
		return -1;
	}
}


/******************************************************************************************
 * ����һԪ���η�������ʵ������		a*x^3 + b*x^2 + c*x + d = 0
 * Input:
 * a,b,c,d:				����ϵ��
 *
 * Output:
 * x:					���̸�
 *
 * return:				��0��������ȷ����-1����������
 * ***************************************************************************************/
int root3(double a, double b, double c, double d, double* z)
{
	if (a != 0)
	{
		double A = b*b - 3*a*c;
		double B = b*c - 9*a*d;
		double C = c*c - 3*b*d;
		double D = B*B - 4*A*C;

		if ((0 == A)&&(0 == B))
		{
			*z = -b/(3*a);
			//cout << "1 1" << endl;
			return 0;
		}
		if(D > 0)
		{
			double y1 = A*b + 1.5*a*(-B + sqrt(B*B - 4*A*C));
			double y2 = A*b + 1.5*a*(-B - sqrt(B*B - 4*A*C));
			//cout << "y1 = " << y1 << endl;
			//cout << "y2 = " << y2 << endl;
			if(y1 > 0 && y2 > 0)
			{
				*z = (-b - (pow(y1,1/3.0) + pow(y2,1/3.0)))/(3*a);
			}
			if(y1 <= 0 && y2 <= 0)
			{
				*z = (-b - (-pow(-y1,1/3.0) +(-pow(-y2,1/3.0))))/(3*a);
			}
			if(y1 > 0 && y2 <= 0)
			{
				*z = (-b - (pow(y1,1/3.0) + (-pow(-y2,1/3.0))))/(3*a);
			}
			if(y1 <= 0 && y2 > 0)
			{
				*z = (-b - (-pow(-y1,1/3.0) + pow(y2,1/3.0)))/(3*a);
			}
			//cout << "1 2" << endl;
			return 0;
		}
		if ((0 == D)&&(A != 0))
		{
			double k = B/A;
			//z = -k/2;
			*z = -b/a + k;
			//cout << "1 3" << endl;
			return 0;
		}
		if ((D < 0)&&(A > 0))
		{
			double T = (2*A*b - 3*a*B)/(2*sqrt(A)*sqrt(A)*sqrt(A));
			double num = 0.0;

			if(T<=-1.0000000000000){
				num=PI;
			}else if(T>=1.0000000000000){
				num=0;
			}else{
				num=acos(T);
			}

			//z = (-b - 2*sqrt(A)*cos(acos(T)/3))/(3*a);
			//z = (-b + sqrt(A)*(cos(acos(T)/3) - sqrt(3)*sin(acos(T)/3)))/(3*a);
			//z = (-b + sqrt(A)*(cos(acos(T)/3) + sqrt(3)*sin(acos(T)/3)))/(3*a);
			*z = (-b + sqrt(A)*(cos(num/3.0) + sqrt(3.0)*sin(num/3.0)))/(3.0*a);

			return 0;
		}
		return 0;
	}
	else
	{
		//cout << "The equation is not a threeEquation !!" << endl;
		// DEBUG("error: not a cubic equation in <root3>\n");
		return -1;
	}
}


/******************************************************************************************
 * ����һԪ�Ĵη���ʵ������		a0*x^4 + b0*x^3 + c0*x^2 + d0*x + e0 = 0
 * Input:
 * a0,b0,c0,d0,e0:		����ϵ��
 *
 * Output:
 * x:					���̸�
 *
 * return:				���̸�����	"-1"����������"0"����û��ʵ��
 * ***************************************************************************************/
int root4(double a0,double b0,double c0,double d0,double e0,double* x)
{
	int size = 0;
	double x1,x2,x3,x4;

	if (a0 != 0)
	{
		// �Ĵη���ϵ����Ϊһ
		// double a = 1;
		double b = b0/a0;
		double c = c0/a0;
		double d = d0/a0;
		double e = e0/a0;

		double a3 = 1;
		double b3 = -c;
		double c3 = b*d - 4*e;
		double d3 = 4*e*c - d*d - e*b*b;
		// �����η��̵���һʵ��z
		double z;
		if (root3(a3,b3,c3,d3,&z)==0)
		{
			//cout << "z = " << z << endl;
		}
		//����һ�����η��̵�ʵ��x1,x2
		double y = z;

		if((0.25*b*b + y - c)==0)
		{
			//cout <<"0.5by-d = " << 0.5*b*y - d << endl;
			//cout <<"0.25y*y - e = " << 0.25*y*y - e << endl;
			double a2 = 1;
			double b2 = 0.5*b;
			double c2 = 0.5*y + sqrt(0.25*y*y - e);
			if (root2(a2,b2,c2,&x1,&x2)==0)
			{
				//cout << " OK 1 1" << endl;
				size = size + 2;
				x[0] = x1;
				x[1] = x2;
			}
			b2 = 0.5*b;
			c2 = 0.5*y - sqrt(0.25*y*y - e);
			if (root2(a2,b2,c2,&x3,&x4)==0)
			{
				//	cout << " OK 1 2" << endl;
				size = size + 2;

				if(size>=4){
					x[2] = x3;
					x[3] = x4;
				}
				else{
					x[0] = x3;
					x[1] = x4;
				}
			}
		}
		else
		{

			double a2 = 1;
			double b2 = 0.5*b - sqrt(0.25*b*b + y - c);
			double c2 = 0.5*y - (0.25*b*y - 0.5*d)/sqrt(0.25*b*b + y - c);
			if (root2(a2,b2,c2,&x1,&x2)==0)
			{
				//cout << " OK 2 1" << endl;
				size = size + 2;
				x[0] = x1;
				x[1] = x2;
			}
			else{
				;
			}
			//���ڶ������η��̵�ʵ��x3,x4
			b2 = 0.5*b + sqrt(0.25*b*b + y - c);
			c2 = 0.5*y + (0.25*b*y - 0.5*d)/sqrt(0.25*b*b + y - c);
			if (root2(a2,b2,c2,&x3,&x4)==0)
			{
				size = size + 2;

				if(size>=4){
					x[2] = x3;
					x[3] = x4;
				}
				else{
					x[0] = x3;
					x[1] = x4;
				}
			}
			else{
				;
			}
		}

		if (size==0){
			// DEBUG("error: no real root in <root4>\n");
			return 0;
		}
		else{
			return size;
		}

	}
	else{
		// DEBUG("error: not a quartic equation in <root4>\n");
		return -1;
	}
}


/******************************************************************************************
 * ׷�Ϸ��������Խ��������η�����
 * Input:
 * n:				���̸���
 * mid:				ϵ�������м��Խ�Ԫ����������
 * up:				ϵ�������м����϶Խ�Ԫ����������
 * down:			ϵ�������м����¶Խ�Ԫ����������
 * b:				��ϵ������
 *
 * Output:
 * x:				���̸�
 *
 * return:
 * ***************************************************************************************/
int crout(double* mid, double* up, double* down, double* b, double* x, int n)
{
	int i;

	//double p[n],q[n-1],y[n];
	double* p = new double[n];
	double* q = new double[n-1];
	double* y = new double[n];

	p[0] = mid[0];

	for(i=0;i<n-1;i++){
		q[i] = up[i]/p[i];
		p[i+1] = mid[i+1]-down[i]*q[i];
	}

	y[0] = b[0]/p[0];

	for(i=1;i<n;i++){
		y[i] = (b[i]-down[i-1]*y[i-1])/p[i];
	}

	x[n-1] = y[n-1];

	for(i=n-2;i>=0;i--){
		x[i] = y[i] - q[i]*x[i+1];
	}

	return 0;
}


int homogeneous2rot(double T[4][4],double R[3][3])
{

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			R[i][j] = T[i][j];
		}
	}
	return 0;
}

int homogeneous2trans(double T[4][4],double X[3])
{
	for (int i=0; i<3; i++)
	{
		X[i] = T[i][3];
	}

	return 0;
}

int homogeneous2pose(double T[4][4], double X[6])
{
	double R[3][3] = {{0.0}};
	homogeneous2rot(T,R);
	matrix2zyz(R,X+3);
	homogeneous2trans(T,X);
	return 0;
}

int rot2homogeneous(double R[3][3],double T[4][4])
{
	
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			T[i][j] = R[i][j];
		}
		T[i][3] = 0;
	}

	T[3][3] = 1.0;

	 return 0;
}

int trans2homogeneous(double X[3],double T[4][4])
{

	for (int i=0; i<3; i++)
	{
		T[i][0] = 0;
		T[i][1] = 0;
		T[i][2] = 0;
		T[i][3] = X[i];
		T[i][i] = 1;
	}
	T[3][3] = 1;
	 
	return 0;
}

int pose2homogeneous(double X[6],double T[4][4])
{
	double R[3][3] = {{0.0}};
	zyz2matrix(X+3,R);
	rot2homogeneous(R,T);
	
	for (int i=0; i<3; i++)
	{
		T[i][3] = X[i];
	}

	return 0;
}

/* *************************************************************
* ��ת���� to zyz ŷ����
* R[3][3]			��̬��ת����
* zyz[3]			zyz ŷ����
***************************************************************/
int matrix2zyz(double R[3][3], double* zyz)
{
	double sin_beta = sqrt(R[2][0] * R[2][0] + R[2][1] * R[2][1]);

	if (ABS(sin_beta)>1e-3) {
		zyz[0] = atan2(R[1][2], R[0][2]);
		zyz[1] = atan2(sin_beta, R[2][2]);
		zyz[2] = atan2(R[2][1], -R[2][0]);
	}
	else {
		zyz[0] = 0;
		zyz[1] = 0;
		zyz[2] = atan2(-R[0][1], R[0][0]);
	}
	return 0;
}

/* *************************************************************
* zyz ŷ���� to ��ת���� 
* zyz[3]			zyz ŷ����
* R[3][3]			��̬��ת����
***************************************************************/
int zyz2matrix(double* zyz, double R[3][3])
{
	double a = zyz[0], b = zyz[1], g = zyz[2];
	R[0][0] = cos(a)*cos(b)*cos(g)-sin(a)*sin(g);
	R[0][1] = -cos(a)*cos(b)*sin(g)-sin(a)*cos(g);
	R[0][2] = cos(a)*sin(b);
	R[1][0] = sin(a)*cos(b)*cos(g)+cos(a)*sin(g);
	R[1][1] = -sin(a)*cos(b)*sin(g)+cos(a)*cos(g);
	R[1][2] = sin(a)*sin(b);
	R[2][0] = -sin(b)*cos(g);
	R[2][1] = sin(b)*sin(g);
	R[2][2] = cos(b);

	return 0 ;
}


/***************************************
 * matrix(3��3) transpose function
 **************************************/
void Transpose3(double R[3][3], double R1[3][3])
{
	int i,j;
	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++){
			R[j][i] = R1[i][j];
		}
	}
}

/***************************************
 * matrix(3��3) multiplication function
 * R = R1*R2
 **************************************/
void M3p3(double R[3][3], double R1[3][3], double R2[3][3])
{
    int i, j, k;
    double temp[3] = {0.0};
    for(i = 0; i < 3; i++){
        for(k = 0; k < 3; k ++)
            temp[k] = 0;
        for(j = 0; j < 3; j++){
            for(k = 0; k < 3; k++){
                temp[k] += R1[i][j] * R2[j][k];
            }
        }
        for(k = 0; k < 3; k++){
            R[i][k] = temp[k];
        }
    }
}

/***************************************
 * matrix(3��3) and vector(3x1) multiplication function
 * R(3x1) = R1(3x3)*R2(3x1)
 **************************************/
void M3p3(double R[3], double R1[3][3], double R2[3])
{
    for (int i=0; i<3; i++)
    {
		double temp = 0.0;
		for (int j=0; j<3; j++)
		{
			temp = temp + R1[i][j] * R2[j];
		}
		R[i] = temp;
    }
}

/***************************************
 * matrix(4��4) multiplication function
 * R = R1*R2
 **************************************/
void M4p4(double R[4][4], double R1[4][4], double R2[4][4])
{
    int i, j, k;
    double temp[4] = {0.0};
    for(i = 0; i < 4; i++){
        for(k = 0; k < 4; k ++)
            temp[k] = 0;
        for(j = 0; j < 4; j++){
            for(k = 0; k < 4; k++){
                temp[k] += R1[i][j] * R2[j][k];
            }
        }
        for(k = 0; k < 4; k++){
            R[i][k] = temp[k];
        }
    }
}
/***************************************
 * matrix(4��4) multiplication function
 * R = R1*R2*R3
 **************************************/
void M4p4p4(double R[4][4], double R1[4][4], double R2[4][4], double R3[4][4])
{
	double R_tmp[4][4];

	M4p4(R_tmp,R1,R2);
	M4p4(R,R_tmp,R3);

}
/***************************************
 * rotation around x-axis
 *************************************/
void rot_x(double R[4][4], double t)
{
	//R = [1 0 0 0;0 cos(theta) -sin(theta) 0;0 sin(theta) cos(theta) 0;0 0 0 1];
	R[0][0] = 1.0;	R[0][1] = 0.0;		R[0][2] = 0.0;		R[0][3] = 0.0;
	R[1][0] = 0.0;	R[1][1] = cos(t);	R[1][2] = -sin(t);	R[1][3] = 0.0;
	R[2][0] = 0.0;	R[2][1] = sin(t);	R[2][2] = cos(t);	R[2][3] = 0.0;
	R[3][0] = 0.0;	R[3][1] = 0.0;		R[3][2] = 0.0;		R[3][3] = 1.0;
}

/***************************************
 * rotation around y-axis
 *************************************/
void rot_y(double R[4][4], double t)
{
	//R = [cos(theta) 0 sin(theta) 0;0 1 0 0;-sin(theta) 0 cos(theta) 0;0 0 0 1];
	R[0][0] = cos(t);	R[0][1] = 0.0;	R[0][2] = sin(t);	R[0][3] = 0.0;
	R[1][0] = 0.0;		R[1][1] = 1.0;	R[1][2] = 0.0;		R[1][3] = 0.0;
	R[2][0] = -sin(t);	R[2][1] = 0.0;	R[2][2] = cos(t);	R[2][3] = 0.0;
	R[3][0] = 0.0;		R[3][1] = 0.0;	R[3][2] = 0.0;		R[3][3] = 1.0;
}
/***************************************
 * rotation around z-axis
 *************************************/
void rot_z(double R[4][4], double t)
{
	//R = [cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
	R[0][0] = cos(t);	R[0][1] = -sin(t);	R[0][2] = 0.0;	R[0][3] = 0.0;
	R[1][0] = sin(t);	R[1][1] = cos(t);	R[1][2] = 0.0;	R[1][3] = 0.0;
	R[2][0] = 0.0;		R[2][1] = 0.0;		R[2][2] = 1.0;	R[2][3] = 0.0;
	R[3][0] = 0.0;		R[3][1] = 0.0;		R[3][2] = 0.0;	R[3][3] = 1.0;
}
/***************************************
 * move along l[3]
 *************************************/
void trans(double R[4][4], double l[3])
{
	R[0][0] = 1.0;	R[0][1] = 0.0;	R[0][2] = 0.0;	R[0][3] = l[0];
	R[1][0] = 0.0;	R[1][1] = 1.0;	R[1][2] = 0.0;	R[1][3] = l[1];
	R[2][0] = 0.0;	R[2][1] = 0.0;	R[2][2] = 1.0;	R[2][3] = l[2];
	R[3][0] = 0.0;	R[3][1] = 0.0;	R[3][2] = 0.0;	R[3][3] = 1.0;
}
/* *************************************************************
 * axis to rotation matrix
 * R[3][3]			rotation matrix
 * k[3]				axis unit vector
 * t				rotation angle
 ***************************************************************/
int rot2matrix(double R[3][3], double* k, double* t){

	if (t==NULL){
		*t = sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);
		if(*t==0){
			//DEBUG("Error in <rot2matrix> for invalid rotation axis 'k'(zero axis vector).\n");
			return -1;
		}
		k[0] = k[0]/(*t);
		k[1] = k[1]/(*t);
		k[2] = k[2]/(*t);
	}
	else if(  ABS(sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]) - 1) > MIN_VALUE_EQ_ZERO){
		//DEBUG("Error in <rot2matrix> for invalid rotation axis 'k'.\n");
		return -1;
	}

	double v = 1 - cos(*t);

	R[0][0] = k[0]*k[0]*v + cos(*t);		R[0][1] = k[0]*k[1]*v - k[2]*sin(*t);	R[0][2] = k[0]*k[2]*v + k[1]*sin(*t);
	R[1][0] = k[0]*k[1]*v + k[2]*sin(*t);	R[1][1] = k[1]*k[1]*v + cos(*t);		R[1][2] = k[1]*k[2]*v - k[0]*sin(*t);
	R[2][0] = k[0]*k[2]*v - k[1]*sin(*t);	R[2][1] = k[1]*k[2]*v + k[0]*sin(*t);	R[2][2] = k[2]*k[2]*v + cos(*t);

	return 0;
}

/* *************************************************************
 * rotation matrix to axis
 * R[3][3]			rotation matrix
 * k[3]				axis unit vector
 * t				rotation angle
 ***************************************************************/
int matrix2rot(double R[3][3], double* k, double* t){

	double temp = (R[0][0]+R[1][1]+R[2][2]-1)/2.0;

	if (ABS(temp) >1)
		temp = sign(temp)*1.0;

	*t = acos(temp);

	if(*t==0 || *t==PI){
		if(k==NULL){
			//DEBUG("Error in <matrix2rot> for invalid rotation axis 'k'.\n");
			return -1;
		}
		else{
			;
		}
	}
	else{
		double tmp[3];

		tmp[0] = 0.5*(R[2][1]-R[1][2])/sin(*t);
		tmp[1] = 0.5*(R[0][2]-R[2][0])/sin(*t);
		tmp[2] = 0.5*(R[1][0]-R[0][1])/sin(*t);

		if(k!=NULL){
			double d1 = (tmp[0]-k[0])*(tmp[0]-k[0]) + (tmp[1]-k[1])*(tmp[1]-k[1]) + (tmp[2]-k[2])*(tmp[2]-k[2]);
			double d2 = (tmp[0]+k[0])*(tmp[0]+k[0]) + (tmp[1]+k[1])*(tmp[1]+k[1]) + (tmp[2]+k[2])*(tmp[2]+k[2]);
			if(d1>d2){
				k[0] = -tmp[0];
				k[1] = -tmp[1];
				k[2] = -tmp[2];

				*t = -(*t);
			}
			else{
				k[0] = tmp[0];
				k[1] = tmp[1];
				k[2] = tmp[2];
			}
		}
		else{
			k[0] = tmp[0];
			k[1] = tmp[1];
			k[2] = tmp[2];
		}
	}

	return 0;
}
