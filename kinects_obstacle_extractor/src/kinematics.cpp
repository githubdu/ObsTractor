/*
 * kinematics.cpp
 */

#include "../include/kinematics.h"

/* *************************************************************************************
 * SRS����7���ɶȻ�е�۲ο�ƽ�����ⷽ��
 *
 * Input:
 * 		v:         	��-������������3��1��
 * 		r��        		�ؽڽǣ���������������Ϊ��ǰ�ؽڽǶȣ�
						��������������Ϊǰһ���ڹؽڽǶȻ�'NULL',
						NULL��ʾ�ǹ켣�滮��ʹ�ã����ܿռ�����ʵ�ʶ��⣩
 * 		l[]:       	���˳���l(1)-l(4)�ֱ���ʾ��
 * 		L01:		������1�ؽ�(2,3�ؽ�)ԭ�㣻
 * 		L34��		3�ؽ�(1,2�ؽ�)��4�ؽ�ԭ�㣻
 * 		L45��		4�ؽڵ�5�ؽ�(6,7�ؽ�)ԭ��L45��
 * 		L7t��		7�ؽ�(5,6�ؽ�)������ĩ��L7t��
 *
 * Output:
 * 		R03_0��    	�������Ĳο�ƽ����ת����
 * *************************************************************************************/
int kinematics_SRS_reference(double R03_0[3][3], double v[3], double* r, double* l)
{
    double l_34 = l[1];
    double l_45 = l[2];

    double r1,r2,r3,r4,r5,r6,r7;
    if(r!=NULL){
    	r1 = r[0];
    	r2 = r[1];
    	r3 = r[2];
    	r4 = -r[3];// iiwa�ĵ�4�᷽���Ƿ���
    	r5 = r[4];
    	r6 = r[5];
    	r7 = r[6];
    }
    else{
    	r1 = 0.0;
    	r2 = 0.0;
    	r3 = 0.0;
    	r4 = 0.0;
    	r5 = 0.0;
    	r6 = 0.0;
    	r7 = 0.0;
    }

    double a[3],b[3];
    a[0] = sin(r4)*l_45;
    a[1] = 0;
    a[2] = cos(r4)*l_45 + l_34;

    b[0] = v[0];
    b[1] = v[1];
    b[2] = v[2];

    /* a = [s4*l45; 0; c4*l45+l34];
     * R03{while r3=0} = [c1c2 -s1 c1s2;c2s1 c1 s1s2;-s2 0 c2];
     * ���� R03*a = b �����ο�ƽ����joint1��joint2�Ĺؽڽ�
     * c1*c2*a(1) + c1*s2*a(3) = b(1)
     * s1*c2*a(1) + s1*s2*a(3) = b(2)
     * -s2*a(1) +    c2*a(3) = b(3)*/
    double c1=0,c2=0,s1=0,s2=0;

	// 2017-06-16
	// a*sin(x) + b*cos(x) = c,
	//	�� x = atan2(c,+/-sqrt(a*a+b*b-c*c))-atan2(b,a)
	double A = -a[0], B = a[2], C = b[2],D=A*A+B*B - C*C;
	if (D>-(ACCURACY_FACTOR))
	{
		double temp1 = cos(atan2(C,sqrt(D))-atan2(B,A));
		double temp2 = cos(atan2(C,-sqrt(D))-atan2(B,A));
		if(ABS(temp1-cos(r2)) < ABS(temp2-cos(r2)) )
			c2 = temp1;
		else
			c2 = temp2;
		
		if (r2>=0)
			s2 = sqrt(1-c2*c2);
		else
			s2 = -sqrt(1-c2*c2);

		if ( s2==0 || (a[0]*c2+a[2]*s2) == 0){
			// joint1 ����ѡȡ�����˴�����ֱ����ֱ����״̬
			c1 = cos(r1);
			s1 = sin(r1);
		}
		else{
			c1 = b[0]/(a[0]*c2+a[2]*s2);
			s1 = b[1]/(a[0]*c2+a[2]*s2);
			if (ABS(c1) >1)
				c1 = sign(c1)*sqrt(1-s1*s1);
			if (ABS(s1)>1)
				s1 = sign(s1)*sqrt(1-c1*c1);
		}
	}
	// 2017-06-16
    if ( ABS(s2)>1+ACCURACY_FACTOR || ABS(c2)>1+ACCURACY_FACTOR ||
		ABS(s1)>1+ACCURACY_FACTOR || ABS(c1)>1+ACCURACY_FACTOR ){
        //DEBUG("error: invalid reference plan for error r1_0/r2_0 in <kinematics_SRS_reference>\n");
        return -1;
    }

    // R03_0 = [c1*c2 -s1 c1*s2;c2*s1 c1 s1*s2;-s2 0 c2];
    R03_0[0][0] = c1*c2;	R03_0[0][1] = -s1;	R03_0[0][2] = c1*s2;
    R03_0[1][0] = c2*s1;	R03_0[1][1] = c1;	R03_0[1][2] = s1*s2;
    R03_0[2][0] = -s2;		R03_0[2][1] = 0;	R03_0[2][2] = c2;


	return 0;
}

/* *************************************************************************************
 * ��������ֵ�����ؽڽǶ�
 * Input:
 * 		c:          ����ֵ
 * 		r_min��        	�ؽڽ�����
 * 		r_max:      �ؽڽ�����
 * 		r_ref:      �ο��ؽڽǶ�
 *
 * Output:
 * 		r��            	�������Ĺؽڽ�
 * *************************************************************************************/
int kinematics_SRS_check_c(double* r, double c, double r_max, double r_min, double r_ref)
{
    // �ж�joint�Ƿ������Ʒ�Χ���н�
    if ( r_max <= r_min ){
        //DEBUG("error: invalid joint limit r_max/r_min in <kinematics_SRS_check_c>\n");
        return -1;
    }

    if ( c > 1 ){
    	//DEBUG("error: invalid joint input parameter cos>1 in <kinematics_SRS_check_c>\n");
        c = 1;
    }

    if ( sign(r_max)!=sign(r_min) ){
        if ( c < MIN( cos(r_max), cos(r_min) ) ){
            // �����Ĺؽڽ� r
        	//DEBUG("error: invalid joint for exceed joint limit in <kinematics_SRS_check_c>\n");
            return -1;
        }
        else{
            if ( c < cos(r_max) || c < cos(r_min) ){
                if (c < cos(r_max))
                    *r = -acos(c);
                else
                    *r = acos(c);
            }
            else{
                // ���ݹؽڽǶ����ƣ����ܴ���2������(����ɢ��������,�����ڹ켣�滮�ɸ���ǰһʱ��r����ȷ��)
                // ����joint�켣������������,joint=0
                if (r_ref >= 0)
                    *r = acos(c);
                else
                    *r = -acos(c);
            }
        }
    }
    else{
        if ( c > MAX( cos(r_max), cos(r_min) ) || c < MIN( cos(r_max), cos(r_min) ) ){
            // �����Ĺؽڽ� r
        	//DEBUG("error: invalid joint for exceed joint limit in <kinematics_SRS_check_c>\n");
            return -1;
        }
        else{
            if ( sign(r_min) >= 0)
                *r = acos(c);
            else
                *r = -acos(c);
        }
    }

	return 0;
}

/******************************************************************************
 *  SRS����7���ɶȻ�е���˶�ѧ����
 *  �˶�ѧDH��������Ϊ4�����˳��ȱ�ʾ
 *  Input:
 *      r[]:      	�ؽڽ�r(1)-r(7)�ֱ���ʾjoint1-joint7��ת�Ƕȣ���λ�����ȣ�
 *      l[]:       	���˳���l(1)-l(4)�ֱ���ʾ��
 *      L01:		������1�ؽ�(2,3�ؽ�)ԭ�㣻
 *      L34��		3�ؽ�(1,2�ؽ�)��4�ؽ�ԭ�㣻
 *      L45��		4�ؽڵ�5�ؽ�(6,7�ؽ�)ԭ��L45��
 *      L7t:		7�ؽ�(5,6�ؽ�)������ĩ��L7t��
 *  Output:
 *      P:         	ĩ�˹�������ϵ{t}�����ڻ�����λ��,x,y,z,�Լ�zyzŷ���Ǻ�������
 *      R:        	ĩ�˹�������ϵ{t}������ԭ����̬����
 *      phai:     	�����ǣ������󹹳�ƽ�����ο�ƽ���нǣ��ο�ƽ��Ϊjoint3=0ʱ�ļ�����ƽ�棩
 *      pp:        	{1,2,3}����ԭ�㣬{4}����ԭ�㣬{5,6,7}����ԭ�㣬�ο�ƽ����{4}����ԭ��
 ***********************************************************************************/
int kinematics_SRS_forward(double P[7], double R[3][3], double* phai, double pp[][3], double* r, double* l)
{
    double L01 = l[0];
    double L34 = l[1];
    double L45 = l[2];
    double L7t = l[3];

	double T01[4][4],T12[4][4],T23[4][4],T34[4][4],T45[4][4],T56[4][4],T67[4][4],T7t[4][4];
	double T03[4][4],T04[4][4],T05[4][4],T0t[4][4];

	double l_tmp[3];
	double R1_tmp[4][4],R2_tmp[4][4],R3_tmp[4][4],R4_tmp[4][4];

	// step 1: �����������α任����&ĩ��λ��
	// ����T01:	T01 = trans(0,0,l_01)*rot_z(r(1));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L01;
	trans(R1_tmp,l_tmp);
	rot_z(R2_tmp,r[0]);
	M4p4(T01,R1_tmp,R2_tmp);
	// ����T12: T12 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(2));
	rot_x(R1_tmp,-PI/2);
	rot_z(R2_tmp,-PI/2);
	rot_z(R3_tmp,r[1]);
	M4p4p4(T12,R1_tmp,R2_tmp,R3_tmp);
	// ����T23: T23 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(3));
	rot_y(R1_tmp,PI/2);
	rot_z(R2_tmp,PI/2);
	rot_z(R3_tmp,r[2]);
	M4p4p4(T23,R1_tmp,R2_tmp,R3_tmp);
	// ����T34:	T34 = trans(0,0,l_34)*rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(4));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L34;
	trans(R1_tmp,l_tmp);
	rot_x(R2_tmp,-PI/2);
	rot_z(R3_tmp,-PI/2);
	M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
	rot_z(R1_tmp,-r[3]);// iiwa�ĵ������������Ƿ���
	M4p4(T34,R4_tmp,R1_tmp);
	// ����T45:	T45 = trans(l_45,0,0)*rot_y(pi/2)*rot_z(pi/2)*rot_z(r(5));
	l_tmp[0]=L45, l_tmp[1]=0, l_tmp[2]=0;
	trans(R1_tmp,l_tmp);
	rot_y(R2_tmp,PI/2);
	rot_z(R3_tmp,PI/2);
	M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
	rot_z(R1_tmp,r[4]);
	M4p4(T45,R4_tmp,R1_tmp);
	// ����T56:	T56 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(6));
	rot_x(R1_tmp,-PI/2);
	rot_z(R2_tmp,-PI/2);
	rot_z(R3_tmp,r[5]);
	M4p4p4(T56,R1_tmp,R2_tmp,R3_tmp);
	// ����T67:	T67 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(7));
	rot_y(R1_tmp,PI/2);
	rot_z(R2_tmp,PI/2);
	rot_z(R3_tmp,r[6]);
	M4p4p4(T67,R1_tmp,R2_tmp,R3_tmp);
	// ����T7t:	T7t = trans(0,0,l_7t);
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L7t;
	trans(T7t,l_tmp);

	// ����T03��T04��T05��T0t
	M4p4p4(T03,T01,T12,T23);
	M4p4(T04,T03,T34);
	M4p4(T05,T04,T45);
	M4p4(R1_tmp,T05,T56);
	M4p4p4(T0t,R1_tmp,T67,T7t);
	// ����ĩ��λ��
	P[0] = T0t[0][3];
	P[1] = T0t[1][3];
	P[2] = T0t[2][3];
	// ����ĩ����̬
	R[0][0] = T0t[0][0],	R[0][1] = T0t[0][1],	R[0][2] = T0t[0][2];
	R[1][0] = T0t[1][0],	R[1][1] = T0t[1][1],	R[1][2] = T0t[1][2];
	R[2][0] = T0t[2][0],	R[2][1] = T0t[2][1],	R[2][2] = T0t[2][2];

	// ����ŷ����
	matrix2zyz(R,&P[3]);

	// step 2������������
	double Xsw[3],vXsw[3];
	Xsw[0] = P[0] - R[0][2]*L7t;
	Xsw[1] = P[1] - R[1][2]*L7t;
	Xsw[2] = P[2] - R[2][2]*L7t - L01;

	double dd = sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]+Xsw[2]*Xsw[2]);

	vXsw[0] = Xsw[0]/dd;
	vXsw[1] = Xsw[1]/dd;
	vXsw[2] = Xsw[2]/dd;

	double R03_0[3][3],R03_0_T[3][3],Rphai0[3][3],R03[3][3];

	kinematics_SRS_reference(R03_0, Xsw, r, l);

	Transpose3(R03_0_T,R03_0);

	R03[0][0] = T03[0][0],	R03[0][1] = T03[0][1],	R03[0][2] = T03[0][2];
	R03[1][0] = T03[1][0],	R03[1][1] = T03[1][1],	R03[1][2] = T03[1][2];
	R03[2][0] = T03[2][0],	R03[2][1] = T03[2][1],	R03[2][2] = T03[2][2];

	M3p3(Rphai0,R03,R03_0_T);

	if (phai!=NULL)
	{
		matrix2rot(Rphai0,vXsw,phai);
		P[6] = *phai;
	}
	

	// step3�����������м���
	double T03_0[4][4],T04_0[4][4];
	T03_0[0][0]=R03_0[0][0], T03_0[0][1]=R03_0[0][1], T03_0[0][2]=R03_0[0][2], T03_0[0][3]=0;
	T03_0[1][0]=R03_0[1][0], T03_0[1][1]=R03_0[1][1], T03_0[1][2]=R03_0[1][2], T03_0[1][3]=0;
	T03_0[2][0]=R03_0[2][0], T03_0[2][1]=R03_0[2][1], T03_0[2][2]=R03_0[2][2], T03_0[2][3]=L01;
	T03_0[3][0]=0, 			 T03_0[3][1]=0, 		  T03_0[3][2]=0, 		   T03_0[3][3]=1;

	M4p4(T04_0,T03_0,T34);

	if (pp == NULL)
	{
		return 0;
	}

	pp[SHOULDER][0] = 0;
	pp[SHOULDER][1] = 0;
	pp[SHOULDER][2] = L01;

	pp[ELBOW][0] = T04[0][3];
	pp[ELBOW][1] = T04[1][3];
	pp[ELBOW][2] = T04[2][3];

	pp[WRIST][0] = T05[0][3];
	pp[WRIST][1] = T05[1][3];
	pp[WRIST][2] = T05[2][3];

	pp[FLANGE][0] = T0t[0][3];
	pp[FLANGE][1] = T0t[1][3];
	pp[FLANGE][2] = T0t[2][3];

	pp[REFERENCE][0] = T04_0[0][3];
	pp[REFERENCE][1] = T04_0[1][3];
	pp[REFERENCE][2] = T04_0[2][3];


	return 0;
}


/*****************************************************************************
 * SRS����7���ɶȻ�е���˶�ѧ����
 * �˶�ѧDH��������Ϊ4�����˳��ȱ�ʾ
 *
 * Input:
 * 		Pd:        	ĩ�˹�������ϵ{t}�����ڻ�����λ��
 * 		Rd��      	 	ĩ�˹�������ϵ{t}������ԭ����̬����
 * 		phai��     	�����ǣ���-��-�� ����ƽ�����ο�ƽ���нǣ��ο�ƽ��Ϊjoint3=0ʱ�ļ�����ƽ�棩
 * 		pp:        	{1,2,3}����ԭ�㣬{4}����ԭ�㣬{5,6,7}����ԭ�㣬�ο�ƽ����{4}����ԭ��
 * 		l[]:       	���˳���l(1)-l(4)�ֱ���ʾ��
 *               		L01��������1�ؽ�(2,3�ؽ�)ԭ�㣻
 *                     	L34��3�ؽ�(1,2�ؽ�)��4�ؽ�ԭ�㣻
 *                      L45��4�ؽڵ�5�ؽ�(6,7�ؽ�)ԭ��L45��
 *                      L7t��7�ؽ�(5,6�ؽ�)������ĩ��L7t��
 *  r_lim:     		�ؽڽǷ�Χ����������С����
 *  r_ref:     		�ο��ؽڽǣ���ѡ����һʱ�̵ĹؽڽǶȣ�
 *
 *  Output:
 *  	r[]��      		�ؽڽ�r(1)-r(7)�ֱ���ʾjoint1-joint7��ת�Ƕȣ���λ�����ȣ�
 * ***********************************************************************************/
int kinematics_SRS_inverse(double* r, double Pd[3], double Rd[3][3], double* phai, double* l, double* r_ref, double r_lim[][2])
{
    int i,j,k;

	double L01 = l[0];
    double L34 = l[1];
    double L45 = l[2];
    double L7t = l[3];

    double r1_min,r1_max,r2_min,r2_max,r3_min,r3_max,r4_min,r4_max,r5_min,r5_max,r6_min,r6_max,r7_min,r7_max;
    double r1,r2,r3,r4,r5,r6,r7;

    if(r_lim==NULL){
        r1_min = -PI;    r1_max = PI;
        r2_min = -PI;    r2_max = PI;
        r3_min = -PI;    r3_max = PI;
        r4_min = -PI;    r4_max = PI;
        r5_min = -PI;    r5_max = PI;
        r6_min = -PI;    r6_max = PI;
        r7_min = -PI;    r7_max = PI;
    }
    else{
        r1_min = r_lim[0][0];    r1_max = r_lim[0][1];
        r2_min = r_lim[0][0];    r2_max = r_lim[0][1];
        r3_min = r_lim[0][0];    r3_max = r_lim[0][1];
        r4_min = r_lim[0][0];    r4_max = r_lim[0][1];
        r5_min = r_lim[0][0];    r5_max = r_lim[0][1];
        r6_min = r_lim[0][0];    r6_max = r_lim[0][1];
        r7_min = r_lim[0][0];    r7_max = r_lim[0][1];
    }

    if(r_ref==NULL){
    	for(i=0;i<7;i++){
    		r_ref[i] = 0.0;
    	}
    }


	//*******************************
	// ����ȫ���̣�step1����step4
	//*******************************

    // step 1: �����ؽڽ�joint4
	double Xsw[3];
	double vXsw[3];
	double EXsw[3][3];
	double nXsw;
	/*�������ؽ����ġ������ؽ����ĵ�����
	 Xsw[3] 		����
	 nXsw[3] 		��������
	 vXsw[3] 		Xsw����������
	 EXsw[3][3] 	Xsw�������� */

	Xsw[0] = Pd[0] - Rd[0][2]*L7t;
	Xsw[1] = Pd[1] - Rd[1][2]*L7t;
	Xsw[2] = Pd[2] - L01 - Rd[2][2]*L7t;

	nXsw = sqrt(Xsw[0]*Xsw[0] + Xsw[1]*Xsw[1] + Xsw[2]*Xsw[2]);

	vXsw[0] = Xsw[0]/nXsw;
	vXsw[1] = Xsw[1]/nXsw;
	vXsw[2] = Xsw[2]/nXsw;

	EXsw[0][0] = 0;				EXsw[0][1] = -vXsw[2];		EXsw[0][2] = vXsw[1];
	EXsw[1][0] = vXsw[2];		EXsw[1][1] = 0;				EXsw[1][2] = -vXsw[0];
	EXsw[2][0] = -vXsw[1];		EXsw[2][1] = vXsw[0];		EXsw[2][2] = 0;

	if ( nXsw > (L34 + L45) ){
		// ĩ�������������ռ�����
		//DEBUG("Error in <kinematics_SRS_inverse> for unreachable work space.\n");
		return -1;
	}
	// �������Ҷ�������4�ؽڽǶ�
    double c4 = (nXsw*nXsw - L34*L34 - L45*L45)/(2*L34*L45);

    kinematics_SRS_check_c(&r4,c4,r4_max,r4_min,-r_ref[3]);// iiwa�ĵ�4�᷽���Ƿ���

    // step 2: �ο�ƽ���� joint1��joint2�Ĺؽڽ��Լ��ο����� R03_0 while{r3=0}

    double R03_0[3][3];
    kinematics_SRS_reference(R03_0,Xsw,r_ref,l);

    // step 3: ���� r1,r2,r3
    double As[3][3], Bs[3][3], Cs[3][3];
    double EXsw_EXsw[3][3];
    memset(As,0,sizeof(As));
    memset(Bs,0,sizeof(Bs));
    memset(Cs,0,sizeof(Cs));
    memset(EXsw_EXsw,0,sizeof(EXsw_EXsw));
    // ����As
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				As[i][j] = As[i][j] + EXsw[i][k]*R03_0[k][j];
			}
		}
	}
	// ����Bs
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				EXsw_EXsw[i][j] = EXsw_EXsw[i][j] + EXsw[i][k]*EXsw[k][j];
			}
		}
	}

	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				Bs[i][j] = Bs[i][j] - EXsw_EXsw[i][k]*R03_0[k][j];
			}
		}
	}
	// ����Cs
    double vXsw_vXsw[3][3];
    memset(vXsw_vXsw,0,sizeof(vXsw_vXsw));

    vXsw_vXsw[0][0] = vXsw[0]*vXsw[0];		vXsw_vXsw[0][1] = vXsw[0]*vXsw[1];		vXsw_vXsw[0][2] = vXsw[0]*vXsw[2];
    vXsw_vXsw[1][0] = vXsw[1]*vXsw[0];		vXsw_vXsw[1][1] = vXsw[1]*vXsw[1];		vXsw_vXsw[1][2] = vXsw[1]*vXsw[2];
    vXsw_vXsw[2][0] = vXsw[2]*vXsw[0];		vXsw_vXsw[2][1] = vXsw[2]*vXsw[1];		vXsw_vXsw[2][2] = vXsw[2]*vXsw[2];

	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				Cs[i][j] = Cs[i][j] + vXsw_vXsw[i][k]*R03_0[k][j];
			}
		}
	}
	// ����R03��3����ϵ������0����ϵ����ת���󣩣�R03 = As*sin(phai)+Bs*cos(phai)+Cs������phai����Ϊ������ƽ�����ο�ƽ���нǡ�
    double R03[3][3];
    memset(R03,0,sizeof(R03));

    for (i=0;i<3;i++){
    	for (j=0;j<3;j++){
    		R03[i][j] = As[i][j]*sin(*phai) + Bs[i][j]*cos(*phai) + Cs[i][j];
    	}
    }
    // ��ʼ���ݾ�����Ӧλ�����Ǻ���Լ����ϵ����r1,r2,r3
    double c2 = R03[2][2];

    kinematics_SRS_check_c(&r2,c2,r2_max,r2_min,r_ref[1]);

    if (fabs(r2) <= ACCURACY_FACTOR){
        // r1��r3��ȡ����ֵ����ȡ�̶�1����ת3�᷽��
        r2 = 0;
        r1 = atan2( R03[1][0], R03[1][1]) - r_ref[2];
        r3 = r_ref[2];
        if ( r1 < -PI || r1 > PI ){
            if (r1 < -PI)
                r1 = r1+2*PI;
            else
                r1 = r1-2*PI;
        }
    }
    else{
        if (sin(r2) > 0){
            r1 = atan2( R03[1][2], R03[0][2]);
            r3 = atan2( R03[2][1], -R03[2][0]);
        }
        else{
            r1 = atan2( -R03[1][2], -R03[0][2]);
            r3 = atan2( -R03[2][1], R03[2][0]);
        }
		/* ��Խ������joint2=0��r1��r3����ͻ�䣬�ڴ��ж��Ƿ�����ͻ��*/
    }

    if ( r1 < r1_min || r1 > r1_max ){
        //DEBUG("error: invalid joint1 for exceed max/min limit in <kinematic_SRS_inverse>\n");
        return -1;
    }

    if ( r3 < r3_min || r3 > r3_max ){
    	//DEBUG("error: invalid joint3 for exceed max/min limit in <kinematic_SRS_inverse>\n");
        return -1;
    }

    // step 4: ���� r5,r6,r7
    double R34[3][3];
    R34[0][0] = sin(r4);	R34[0][1] = cos(r4);	R34[0][2] = 0;
    R34[1][0] = 0;			R34[1][1] = 0;			R34[1][2] = 1;
    R34[2][0] = cos(r4);	R34[2][1] = -sin(r4);	R34[2][2] = 0;

    double R34_rot[3][3];				// R34ת��
    double As_rot[3][3], Bs_rot[3][3], Cs_rot[3][3];
    double Aw_tmp[3][3], Bw_tmp[3][3], Cw_tmp[3][3];
    double Aw[3][3], Bw[3][3], Cw[3][3];
    memset(R34_rot,0,sizeof(R34_rot));
    memset(As_rot,0,sizeof(As_rot));
    memset(Bs_rot,0,sizeof(Bs_rot));
    memset(Cs_rot,0,sizeof(Cs_rot));
    memset(Aw_tmp,0,sizeof(Aw_tmp));
    memset(Bw_tmp,0,sizeof(Bw_tmp));
    memset(Cw_tmp,0,sizeof(Cw_tmp));
    memset(Aw,0,sizeof(Aw));
    memset(Bw,0,sizeof(Bw));
    memset(Cw,0,sizeof(Cw));
    // ����ת�þ���R34',As',Bs',Cs'
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			R34_rot[i][j] = R34[j][i];
			As_rot[i][j] = As[j][i];
			Bs_rot[i][j] = Bs[j][i];
			Cs_rot[i][j] = Cs[j][i];
		}
	}
	// ����ת�þ���Aw_tmp = R34'*As', Bw_tmp = R34'*Bs', Cw_tmp = R34'*Cs'
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				Aw_tmp[i][j] = Aw_tmp[i][j] + R34_rot[i][k]*As_rot[k][j];
				Bw_tmp[i][j] = Bw_tmp[i][j] + R34_rot[i][k]*Bs_rot[k][j];
				Cw_tmp[i][j] = Cw_tmp[i][j] + R34_rot[i][k]*Cs_rot[k][j];
			}
		}
	}
	// ����ת�þ���Aw = Aw_tmp*Rd, Bw = Bw_tmp*Rd, Cw = Cw_tmp*Rd
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				Aw[i][j] = Aw[i][j] + Aw_tmp[i][k]*Rd[k][j];
				Bw[i][j] = Bw[i][j] + Bw_tmp[i][k]*Rd[k][j];
				Cw[i][j] = Cw[i][j] + Cw_tmp[i][k]*Rd[k][j];
			}
		}
	}

    double R47[3][3];
    memset(R47,0,sizeof(R47));
    for (i=0;i<3;i++){
    	for (j=0;j<3;j++){
    		R47[i][j] = Aw[i][j]*sin(*phai) + Bw[i][j]*cos(*phai) + Cw[i][j];
    	}
    }

    // ��ʼ���ݾ�����Ӧλ�����Ǻ���Լ����ϵ����r5,r6,r7
    double c6 = R47[0][2];

    kinematics_SRS_check_c(&r6,c6,r6_max,r6_min,r_ref[5]);

    if (fabs(r6)<=ACCURACY_FACTOR){
        // joint5 �� joint7 ��ת�Ṳ�ߣ�������ѡȡ
        r6 = 0;
        r5 = atan2( R47[2][0], R47[2][1])-r_ref[6];
        r7 = r_ref[6];
        if ( r5 < -PI || r5 > PI ){
            if ( r5 < -PI )
                r5 = r5+2*PI;
            else
                r5 = r5-2*PI;
        }
    }
    else{
        if ( sin(r6) > 0 ){
            r5 = atan2( R47[2][2], R47[1][2]);
            r7 = atan2( R47[0][1], -R47[0][0]);
        }
        else{
            r5 = atan2( -R47[2][2], -R47[1][2]);
            r7 = atan2( -R47[0][1], R47[0][0]);
        }
    }

    if ( r5 < r5_min || r5 > r5_max ){
        //DEBUG("error: invalid joint5 for exceed max/min limit in <kinematic_SRS_inverse>\n");
        return -1;
    }

    if ( r7 < r7_min || r7 > r7_max ){
    	//DEBUG("error: invalid joint7 for exceed max/min limit in <kinematic_SRS_inverse>\n");
        return -1;
    }

    // ���йؽڽǶȼ�������
    r[0] = r1;
    r[1] = r2;
    r[2] = r3;
    r[3] = -r4;  // iiwa�ĵ�4���Ƿ���
    r[4] = r5;
    r[5] = r6;
    r[6] = r7;


	return 0;
}


int kinematics_SRS_jacobi0(double Jacobi0[ ][DOF], double Tt0[4][4], double* r, double* l)
{
	double L01 = l[0];
	double L34 = l[1];
	double L45 = l[2];
	double L7t = l[3];

	double T01[4][4],T12[4][4],T23[4][4],T34[4][4],T45[4][4],T56[4][4],T67[4][4],T7t[4][4];
	double T02[4][4],T03[4][4],T04[4][4],T05[4][4],T06[4][4],T07[4][4],T0t[4][4];
	double T6t[4][4],T5t[4][4],T4t[4][4],T3t[4][4],T2t[4][4],T1t[4][4];

	double l_tmp[3];
	double R1_tmp[4][4],R2_tmp[4][4],R3_tmp[4][4],R4_tmp[4][4];

	// step 1: �����������α任����
	// ����T01:	T01 = trans(0,0,l_01)*rot_z(r(1));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L01;
	trans(R1_tmp,l_tmp);
	rot_z(R2_tmp,r[0]);
	M4p4(T01,R1_tmp,R2_tmp);
	// ����T12: T12 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(2));
	rot_x(R1_tmp,-PI/2);
	rot_z(R2_tmp,-PI/2);
	rot_z(R3_tmp,r[1]);
	M4p4p4(T12,R1_tmp,R2_tmp,R3_tmp);
	// ����T23: T23 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(3));
	rot_y(R1_tmp,PI/2);
	rot_z(R2_tmp,PI/2);
	rot_z(R3_tmp,r[2]);
	M4p4p4(T23,R1_tmp,R2_tmp,R3_tmp);
	// ����T34:	T34 = trans(0,0,l_34)*rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(4));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L34;
	trans(R1_tmp,l_tmp);
	rot_x(R2_tmp,-PI/2);
	rot_z(R3_tmp,-PI/2);
	M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
	rot_z(R1_tmp,-r[3]);// iiwa�ĵ������������Ƿ���
	M4p4(T34,R4_tmp,R1_tmp);
	// ����T45:	T45 = trans(l_45,0,0)*rot_y(pi/2)*rot_z(pi/2)*rot_z(r(5));
	l_tmp[0]=L45, l_tmp[1]=0, l_tmp[2]=0;
	trans(R1_tmp,l_tmp);
	rot_y(R2_tmp,PI/2);
	rot_z(R3_tmp,PI/2);
	M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
	rot_z(R1_tmp,r[4]);
	M4p4(T45,R4_tmp,R1_tmp);
	// ����T56:	T56 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(6));
	rot_x(R1_tmp,-PI/2);
	rot_z(R2_tmp,-PI/2);
	rot_z(R3_tmp,r[5]);
	M4p4p4(T56,R1_tmp,R2_tmp,R3_tmp);
	// ����T67:	T67 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(7));
	rot_y(R1_tmp,PI/2);
	rot_z(R2_tmp,PI/2);
	rot_z(R3_tmp,r[6]);
	M4p4p4(T67,R1_tmp,R2_tmp,R3_tmp);
	// ����T7t:	T7t = trans(0,0,l_7t);
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L7t;
	trans(T7t,l_tmp);

	// ����T02��T03��T04��T05��T06��T07��T0t
	M4p4(T02,T01,T12);
	M4p4(T03,T02,T23);
	M4p4(T04,T03,T34);
	M4p4(T05,T04,T45);
	M4p4(T06,T05,T56);
	M4p4(T07,T06,T67);
	M4p4(T0t,T07,T7t);

	if (Tt0 != NULL)
	{
		memcpy(Tt0,T0t,sizeof(T0t));
	}

	// ����T1t��T2t��T3t��T4t��T5t��T6t
	M4p4(T6t,T67,T7t);
	M4p4(T5t,T56,T6t);
	M4p4(T4t,T45,T5t);
	M4p4(T3t,T34,T4t);
	M4p4(T2t,T23,T3t);
	M4p4(T1t,T12,T2t);

	//method for Jacobin matrix in word coordinates by Whitney in 1972
	//	J = [J1,J2,...,Jn]
	//Ji = [cross(Zi0,Pni0);Zi0] = [cross(Zi0,Ri0*Pni);Zi0]
	//which, Pni0 is projection in word coordinates of iPn
	//	Pni is the end effector origin coord in i coordinate
	//	Ri0 is i coordinate rotation according to word coordinate
	//	Zi0 = Ri0*[0;0;1]; Pni0 = Ri0*Pni = Ri0*Tni(1:3,4)

	double Zi[3] = {0.0};
	double Pni[3] = {0.0};
	double tempT[3] = {0.0};
	double tempR[3][3] = {{0.0}};

	double J[DOF][6] = {{0.0}};

	// ���� J1
	Zi[0] = T01[0][2];
	Zi[1] = T01[1][2];
	Zi[2] = T01[2][2];
	homogeneous2rot(T01,tempR);
	tempT[0] = T1t[0][3];
	tempT[1] = T1t[1][3];
	tempT[2] = T1t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[0],Zi,Pni);
	memcpy(J[0]+3,Zi,sizeof(Zi));

	// ���� J2
	Zi[0] = T02[0][2];
	Zi[1] = T02[1][2];
	Zi[2] = T02[2][2];
	homogeneous2rot(T02,tempR);
	tempT[0] = T2t[0][3];
	tempT[1] = T2t[1][3];
	tempT[2] = T2t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[1],Zi,Pni);
	memcpy(J[1]+3,Zi,sizeof(Zi));

	// ���� J3
	Zi[0] = T03[0][2];
	Zi[1] = T03[1][2];
	Zi[2] = T03[2][2];
	homogeneous2rot(T03,tempR);
	tempT[0] = T3t[0][3];
	tempT[1] = T3t[1][3];
	tempT[2] = T3t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[2],Zi,Pni);
	memcpy(J[2]+3,Zi,sizeof(Zi));

	// ���� J4
	Zi[0] = T04[0][2];
	Zi[1] = T04[1][2];
	Zi[2] = T04[2][2];
	homogeneous2rot(T04,tempR);
	tempT[0] = T4t[0][3];
	tempT[1] = T4t[1][3];
	tempT[2] = T4t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[3],Zi,Pni);
	memcpy(J[3]+3,Zi,sizeof(Zi));
	
	// iiwa�ĵ������������Ƿ���
	for(int i=0; i<6; i++)
		J[3][i] = - J[3][i];

	// ���� J5
	Zi[0] = T05[0][2];
	Zi[1] = T05[1][2];
	Zi[2] = T05[2][2];
	homogeneous2rot(T05,tempR);
	tempT[0] = T5t[0][3];
	tempT[1] = T5t[1][3];
	tempT[2] = T5t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[4],Zi,Pni);
	memcpy(J[4]+3,Zi,sizeof(Zi));

	// ���� J6
	Zi[0] = T06[0][2];
	Zi[1] = T06[1][2];
	Zi[2] = T06[2][2];
	homogeneous2rot(T06,tempR);
	tempT[0] = T6t[0][3];
	tempT[1] = T6t[1][3];
	tempT[2] = T6t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[5],Zi,Pni);
	memcpy(J[5]+3,Zi,sizeof(Zi));

	// ���� J7
	Zi[0] = T07[0][2];
	Zi[1] = T07[1][2];
	Zi[2] = T07[2][2];
	homogeneous2rot(T07,tempR);
	tempT[0] = T7t[0][3];
	tempT[1] = T7t[1][3];
	tempT[2] = T7t[2][3];
	M3p3(Pni,tempR,tempT);
	cross3(J[6],Zi,Pni);
	memcpy(J[6]+3,Zi,sizeof(Zi));


	for (int i=0; i<6; i++)
	{
		for (int j=0; j<DOF; j++)
		{
			Jacobi0[i][j] = J[j][i];
		}
	}

	return 0;
}






int kinematics_SRS_dot_jacobi0(double dJ[6][DOF],double* q, double* qv, double* l)
{
	double d2 = l[1], d3 = l[2], d4 = l[3];
	double dq1 = qv[0], dq2 = qv[1], dq3 = qv[2]; 
	double dq4 = qv[3], dq5 = qv[4], dq6 = qv[5];

	double s1 = sin(q[0]), c1 = cos(q[0]);
	double s2 = sin(q[1]), c2 = cos(q[1]);
	double s3 = sin(q[2]), c3 = cos(q[2]);
	double s4 = sin(q[3]), c4 = cos(q[3]);
	double s5 = sin(q[4]), c5 = cos(q[4]);
	double s6 = sin(q[5]), c6 = cos(q[5]);

	double T1 = (d4*(c4*c6*s3 + c5*s3*s4*s6) + d3*c4*s3);
	double T2 = (s2*s4 + c2*c3*c4);
	double T3 = (c4*s2 - c2*c3*s4);
	double T4 = (s3*s5 - c3*c4*c5);
	double T5 = (d4*(s6*T4 + c3*c6*s4) + d3*c3*s4);
	double T6 = (c3*s5 + c4*c5*s3);
	double T7 = (d4*(s6*T6 - c6*s3*s4) - d3*s3*s4);
	double T8 = (d4*(s6*T6 - c6*s3*s4) - d3*s3*s4);
	double T9 = (c3*c5 - c4*s3*s5);
	double T10 = (c5*T2 - c2*s3*s5);
	double T11 = (c2*s4 - c3*c4*s2);
	double T12 = (c2*c4 + c3*s2*s4);
	double T13 = (d4*(c6*s4 - c4*c5*s6) + d3*s4);
	double T14 = (c4*c6 + c5*s4*s6);
	double T15 = (c4*s6 - c5*c6*s4);
	double T16 = (c1*s3 + c2*c3*s1);
	double T17 = (d4*T14 + d3*c4);
	double T18 = (s1*s3 - c1*c2*c3);
	double T19 = (c3*s1 + c1*c2*s3);
	double T20 = d4*s2*s3*s5*s6;
	double T21 = (c1*c3 - c2*s1*s3);
	double T22 = (c4*T16 + s1*s2*s4);
	double T23 = (d3 + d4*c6);
	double T24 = (s4*T16 - c4*s1*s2);
	double T25 = (c4*T18 - c1*s2*s4);
	double T26 = (s4*T18 + c1*c4*s2);
	double T27 = (c2*c4*s1 + c3*s1*s2*s4);
	double T28 = (c2*s1*s4 - c3*c4*s1*s2);
	double T29 = (c5*T11 + s2*s3*s5);
	double T30 = (c5*T25 + s5*T19);
	double T31 = (c5*T22 + s5*T21);
	double T32 = (s5*T22 - c5*T21);
	double T33 = (s5*T11 - c5*s2*s3);
	double T34 = (s5*T25 - c5*T19);
	double T35 = (s6*T29 + c6*T12);
	double T36 = (c6*T26 - s6*T30);
	double T37 = (c6*T24 - s6*T31);

	dJ[0][0] = dq4*(c1*T1 + s1*(d3*T2 + d4*(c6*T2 - c5*s6*T3))) + 
		dq3*(s1*(d4*(s6*(c2*c3*s5 + c2*c4*c5*s3) - c2*c6*s3*s4) - 
		d3*c2*s3*s4) + c1*T5) - dq1*(c1*(d3*T3 + d2*s2 + d4*(s6*T10 +
		c6*T3)) - s1*T7) + dq5*(d4*s1*s6*T8 - d4*c1*s6*T9) - 
		dq6*(d4*c1*(c6*T6 + s3*s4*s6) + d4*s1*(c6*T10 - s6*T3)) - 
		dq2*s1*(d4*T35 + d3*T12 + d2*c2);

	dJ[0][1] = dq5*c1*(d4*s2*s6*(c5*s3 + c3*c4*s5) - d4*c2*s4*s5*s6) -
		dq4*c1*(c2*T13 - s2*(d4*(c3*c4*c6 + c3*c5*s4*s6) + d3*c3*c4)) - 
		dq2*c1*(s2*(d2 + d4*T14 + d3*c4) - c2*T5) + 
		dq6*c1*(d4*s2*(c6*T4 - c3*s4*s6) - d4*c2*T15) - 
		dq1*s1*(c2*(d2 + d4*T14 + d3*c4) + s2*T5) + dq3*c1*s2*T7;

	dJ[0][2] = dq4*(c2*(T17*T16 + s1*s2*T13) - s1*s2*(c2*T13 - c3*s2*T17)) -
		dq1*(c2*(T13*T18 + c1*s2*T17 - d4*s5*s6*T19) - c1*s2*(c2*T17 +
		c3*s2*T13 + T20)) - dq6*(c2*(d4*T16*(s4*s6 + c4*c5*c6) +
		d4*c6*s5*T21 - d4*s1*s2*T15) + s1*s2*(d4*c2*T15 + d4*c3*s2*(s4*s6 +
		c4*c5*c6) - d4*c6*s2*s3*s5)) + dq3*(c2*(T13*T21 + d4*s5*s6*T16) -
		s1*s2*(s2*s3*T13 - d4*c3*s2*s5*s6)) + dq5*(c2*(d4*c4*s5*s6*T16 -
		d4*c5*s6*T21 + d4*s1*s2*s4*s5*s6) + s1*s2*(d4*c5*s2*s3*s6 - 
		d4*c2*s4*s5*s6 + d4*c3*c4*s2*s5*s6)) + dq2*(s2*(s1*s2*T17 - 
		T13*T16 + d4*s5*s6*T21) - c2*(c2*s1*T17 + c3*s1*s2*T13 + 
		d4*s1*s2*s3*s5*s6) + c2*s1*(c2*T17 + c3*s2*T13 + T20) + 
		s1*s2*(c2*c3*T13 - s2*T17 + d4*c2*s3*s5*s6));

	dJ[0][3] = dq4*((T23*T11 - d4*c5*s6*T12)*T21 - s2*s3*(T22*T23 + 
		d4*c5*s6*T24)) + dq1*(T19*(T23*T12 + d4*c5*s6*T11 + T20) -
		s2*s3*(d4*c5*s6*T25 - T23*T26 + d4*s5*s6*T19)) + 
		dq2*(T21*(T23*T3 + d4*c5*s6*T2 - d4*c2*s3*s5*s6) + 
		s2*s3*(T27*T23 + d4*c5*s6*T28 + d4*s1*s2*s3*s5*s6) + 
		c2*s3*(d4*c5*s6*T22 - T24*T23 + d4*s5*s6*T21) - 
		s1*s2*s3*(T23*T12 + d4*c5*s6*T11 + T20)) + dq3*(T16*(T23*T12 +
		d4*c5*s6*T11 + T20) - T21*(d4*c3*s2*s5*s6 - s2*s3*s4*T23 +
		d4*c4*c5*s2*s3*s6) - s2*s3*(s4*T23*T21 + d4*s5*s6*T16 - 
		d4*c4*c5*s6*T21) + c3*s2*(d4*c5*s6*T22 - T24*T23 + 
		d4*s5*s6*T21)) + dq5*(T21*(d4*s5*s6*T11 - d4*c5*s2*s3*s6) - 
		s2*s3*(d4*s5*s6*T22 - d4*c5*s6*T21)) - dq6*(T21*(d4*c5*c6*T11 -
		d4*s6*T12 + d4*c6*s2*s3*s5) - s2*s3*(d4*s6*T24 +
		d4*c5*c6*T22 + d4*c6*s5*T21));

	dJ[0][4] = dq1*((d4*s6*T29 + d4*c6*T12)*T26 - T12*(d4*c6*T26 -
		d4*s6*T30)) + dq4*(T12*(d4*c6*T22 + d4*c5*s6*T24) - 
		T22*(d4*s6*T29 + d4*c6*T12) + T24*(d4*c6*T11 - d4*c5*s6*T12) -
		T11*(d4*c6*T24 - d4*s6*T31)) + dq5*(d4*s6*T32*T12 + 
		d4*s6*T24*T33) + dq6*(T24*(d4*s6*T12 - d4*c6*T29) - 
		T12*(d4*s6*T24 + d4*c6*T31)) - dq3*(T24*(d4*s6*(c3*s2*s5 +
		c4*c5*s2*s3) - d4*c6*s2*s3*s4) - (d4*s6*(s5*T16 - c4*c5*T21) +
		d4*c6*s4*T21)*T12 + s4*(d4*s6*T29 + d4*c6*T12)*T21 + 
		s2*s3*s4*(d4*c6*T24 - d4*s6*T31)) - dq2*(T12*(d4*s6*(c5*T28 +
		s1*s2*s3*s5) + d4*c6*T27) - T24*(d4*s6*T10 + d4*c6*T3) -
		(d4*s6*T29 + d4*c6*T12)*T27 + T3*(d4*c6*T24 - d4*s6*T31));

	dJ[0][5] = dq1*(d4*T34*T35 + d4*T33*T36) + dq4*(d4*T32*(c6*T11 -
		c5*s6*T12) - d4*T33*(c6*T22 + c5*s6*T24) - d4*s5*T37*T12 +
		d4*s5*T24*T35) - dq5*(d4*T31*T35 + d4*T29*T37) - 
		dq6*(d4*T32*(c6*T29 - s6*T12) - d4*T33*(s6*T24 + c6*T31)) +
		dq2*(d4*(s6*T10 + c6*T3)*T32 + d4*T33*(c6*T27 + s6*(c5*T28 +
		s1*s2*s3*s5)) - d4*T35*(s5*T28 - c5*s1*s2*s3) + d4*T37*T8) -
		dq3*(d4*T33*(s6*(s5*T16 - c4*c5*T21) + c6*s4*T21) - 
		d4*T37*(c3*c5*s2 - c4*s2*s3*s5) + d4*T35*(c5*T16 + c4*s5*T21) +
		d4*T32*(s6*(c3*s2*s5 + c4*c5*s2*s3) - c6*s2*s3*s4));

	dJ[0][6] = 0;

	dJ[1][0] = dq4*(s1*T1 - c1*(d3*T2 + d4*(c6*T2 - c5*s6*T3))) -
		dq3*(c1*(d4*(s6*(c2*c3*s5 + c2*c4*c5*s3) - c2*c6*s3*s4) -
		d3*c2*s3*s4) - s1*T5) - dq1*(s1*(d3*T3 + d2*s2 + d4*(s6*T10 +
		c6*T3)) + c1*T7) - dq5*(d4*c1*s6*T8 + d4*s1*s6*T9) - 
		dq6*(d4*s1*(c6*T6 + s3*s4*s6) - d4*c1*(c6*T10 - s6*T3)) + 
		dq2*c1*(d4*T35 + d3*T12 + d2*c2);

	dJ[1][1] = dq5*s1*(d4*s2*s6*(c5*s3 + c3*c4*s5) - d4*c2*s4*s5*s6) - 
		dq4*s1*(c2*T13 - s2*(d4*(c3*c4*c6 + c3*c5*s4*s6) + d3*c3*c4)) + 
		dq1*c1*(c2*(d2 + d4*T14 + d3*c4) + s2*T5) - dq2*s1*(s2*(d2 +
		d4*T14 + d3*c4) - c2*T5) + dq6*s1*(d4*s2*(c6*T4 - c3*s4*s6) -
		d4*c2*T15) + dq3*s1*s2*T7;

	dJ[1][2] = dq3*(c2*(T13*T19 + d4*s5*s6*T18) + c1*s2*(s2*s3*T13 -
		d4*c3*s2*s5*s6)) - dq2*(s2*(T13*T18 + c1*s2*T17 - d4*s5*s6*T19) -
		c2*(c1*c2*T17 + c1*c3*s2*T13 + d4*c1*s2*s3*s5*s6) +
		c1*c2*(c2*T17 + c3*s2*T13 + T20) + c1*s2*(c2*c3*T13 -
		s2*T17 + d4*c2*s3*s5*s6)) - dq6*(c2*(d4*T18*(s4*s6 + 
		c4*c5*c6) + d4*c6*s5*T19 + d4*c1*s2*T15) - c1*s2*(d4*c2*T15 +
		d4*c3*s2*(s4*s6 + c4*c5*c6) - d4*c6*s2*s3*s5)) - 
		dq5*(c2*(d4*c5*s6*T19 - d4*c4*s5*s6*T18 + d4*c1*s2*s4*s5*s6) +
		c1*s2*(d4*c5*s2*s3*s6 - d4*c2*s4*s5*s6 + d4*c3*c4*s2*s5*s6)) - 
		dq1*(c2*(s1*s2*T17 - T13*T16 + d4*s5*s6*T21) - s1*s2*(c2*T17 +
		c3*s2*T13 + T20)) + dq4*(c2*(T17*T18 - c1*s2*T13) +
		c1*s2*(c2*T13 - c3*s2*T17));

	dJ[1][3] = dq3*(T18*(T23*T12 + d4*c5*s6*T11 + T20) - 
		T19*(d4*c3*s2*s5*s6 - s2*s3*s4*T23 + d4*c4*c5*s2*s3*s6) -
		s2*s3*(s4*T23*T19 + d4*s5*s6*T18 - d4*c4*c5*s6*T19) +
		c3*s2*(d4*c5*s6*T25 - T23*T26 + d4*s5*s6*T19)) - 
		dq1*(T21*(T23*T12 + d4*c5*s6*T11 + T20) - s2*s3*(d4*c5*s6*T22 -
		T24*T23 + d4*s5*s6*T21)) + dq5*(T19*(d4*s5*s6*T11 - 
		d4*c5*s2*s3*s6) - s2*s3*(d4*s5*s6*T25 - d4*c5*s6*T19)) +
		dq2*(T19*(T23*T3 + d4*c5*s6*T2 - d4*c2*s3*s5*s6) - 
		s2*s3*((c1*c2*c4 + c1*c3*s2*s4)*T23 + d4*c5*s6*(c1*c2*s4 -
		c1*c3*c4*s2) + d4*c1*s2*s3*s5*s6) + c2*s3*(d4*c5*s6*T25 -
		T23*T26 + d4*s5*s6*T19) + c1*s2*s3*(T23*T12 + d4*c5*s6*T11 +
		T20)) - dq6*(T19*(d4*c5*c6*T11 - d4*s6*T12 + d4*c6*s2*s3*s5) -
		s2*s3*(d4*s6*T26 + d4*c5*c6*T25 + d4*c6*s5*T19)) + dq4*((T23*T11
		- d4*c5*s6*T12)*T19 - s2*s3*(T23*T25 + d4*c5*s6*T26));

	dJ[1][4] = dq5*(d4*s6*T34*T12 + d4*s6*T33*T26) + dq6*((d4*s6*T12 -
		d4*c6*T29)*T26 - T12*(d4*s6*T26 + d4*c6*T30)) - 
		dq1*(T24*(d4*s6*T29 + d4*c6*T12) - T12*(d4*c6*T24 - 
		d4*s6*T31)) + dq2*(T26*(d4*s6*T10 + d4*c6*T3) - (d4*s6*T29 + 
		d4*c6*T12)*(c1*c2*c4 + c1*c3*s2*s4) - T3*(d4*c6*T26 - 
		d4*s6*T30) + (d4*s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + 
		c1*s2*s3*s5) + d4*c6*(c1*c2*c4 + c1*c3*s2*s4))*T12) -
		dq3*((d4*s6*(c3*s2*s5 + c4*c5*s2*s3) - d4*c6*s2*s3*s4)*T26 -
		(d4*s6*(s5*T18 - c4*c5*T19) + d4*c6*s4*T19)*T12 + 
		s4*(d4*s6*T29 + d4*c6*T12)*T19 + s2*s3*s4*(d4*c6*T26 -
		d4*s6*T30)) + dq4*((d4*c6*T25 + d4*c5*s6*T26)*T12 - 
		(d4*s6*T29 + d4*c6*T12)*T25 - T11*(d4*c6*T26 - d4*s6*T30) +
		(d4*c6*T11 - d4*c5*s6*T12)*T26);

	dJ[1][5] = dq2*(d4*T34*(s6*T10 + c6*T3) - d4*T33*(c6*(c1*c2*c4 +
		c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5)) +
		d4*T35*(s5*(c1*c2*s4 - c1*c3*c4*s2) - c1*c5*s2*s3) + d4*T8*T36) -
		dq6*(d4*T34*(c6*T29 - s6*T12) - d4*T33*(s6*T26 + c6*T30)) -
		dq1*(d4*T32*T35 + d4*T33*T37) - dq3*(d4*T33*(s6*(s5*T18 -
		c4*c5*T19) + c6*s4*T19) + d4*T35*(c5*T18 + c4*s5*T19) + 
		d4*T34*(s6*(c3*s2*s5 + c4*c5*s2*s3) - c6*s2*s3*s4) - 
		d4*(c3*c5*s2 - c4*s2*s3*s5)*T36) - dq5*(d4*T30*T35 + 
		d4*T29*T36) - dq4*(d4*T33*(c6*T25 + c5*s6*T26) - d4*T34*(c6*T11 -
		c5*s6*T12) + d4*s5*T12*T36 - d4*s5*T35*T26);

	dJ[1][6] = 0;

	dJ[2][0] = 0;

	dJ[2][1] = dq5*(c1*(d4*s1*s6*T9 + d4*c1*c2*s6*(c5*s3 + c3*c4*s5) +
		d4*c1*s2*s4*s5*s6) + s1*(d4*c2*s1*s6*(c5*s3 + c3*c4*s5) - 
		d4*c1*s6*T9 + d4*s1*s2*s4*s5*s6)) + dq6*(c1*(d4*s1*(c6*T6 +
		s3*s4*s6) + d4*c1*c2*(c6*T4 - c3*s4*s6) + d4*c1*s2*T15) + 
		s1*(d4*c2*s1*(c6*T4 - c3*s4*s6) - d4*c1*(c6*T6 + s3*s4*s6) +
		d4*s1*s2*T15)) + dq4*(c1*(c1*s2*T13 - s1*T1 + 
		c1*c2*(d4*(c3*c4*c6 + c3*c5*s4*s6) + d3*c3*c4)) + 
		s1*(c1*T1 + s1*s2*T13 + c2*s1*(d4*(c3*c4*c6 + c3*c5*s4*s6) +
		d3*c3*c4))) - dq3*(c1*(s1*T5 - c1*c2*T7) - 
		s1*(c1*T5 + c2*s1*T7)) - dq2*(c1*(c1*c2*(d2 + d4*T14 + d3*c4) +
		c1*s2*T5) + s1*(c2*s1*(d2 + d4*T14 + d3*c4) + s1*s2*T5));

	dJ[2][2] = dq2*(c1*s2*(c2*s1*T17 + c3*s1*s2*T13 + d4*s1*s2*s3*s5*s6) +
		c1*c2*(s1*s2*T17 - T13*T16 + d4*s5*s6*T21) - c2*s1*(T13*T18 +
		c1*s2*T17 - d4*s5*s6*T19) - s1*s2*(c1*c2*T17 + c1*c3*s2*T13 +
		d4*c1*s2*s3*s5*s6)) - dq4*(c1*s2*(T17*T16 + s1*s2*T13) +
		s1*s2*(T17*T18 - c1*s2*T13)) + dq6*(s1*s2*(d4*T18*(s4*s6 + 
		c4*c5*c6) + d4*c6*s5*T19 + d4*c1*s2*T15) + c1*s2*(d4*T16*(s4*s6 +
		c4*c5*c6) + d4*c6*s5*T21 - d4*s1*s2*T15)) - 
		dq5*(c1*s2*(d4*c4*s5*s6*T16 - d4*c5*s6*T21 + d4*s1*s2*s4*s5*s6) -
		s1*s2*(d4*c5*s6*T19 - d4*c4*s5*s6*T18 + d4*c1*s2*s4*s5*s6)) -
		dq3*(c1*s2*(T13*T21 + d4*s5*s6*T16) +
		s1*s2*(T13*T19 + d4*s5*s6*T18));

	dJ[2][3] = dq5*(T21*(d4*s5*s6*T25 - d4*c5*s6*T19) - 
		T19*(d4*s5*s6*T22 - d4*c5*s6*T21)) + dq3*(T16*(d4*c5*s6*T25 -
		T23*T26 + d4*s5*s6*T19) - T18*(d4*c5*s6*T22 - T24*T23 + 
		d4*s5*s6*T21) - T19*(s4*T23*T21 + d4*s5*s6*T16 - 
		d4*c4*c5*s6*T21) + T21*(s4*T23*T19 + d4*s5*s6*T18 -
		d4*c4*c5*s6*T19)) - dq6*(T21*(d4*s6*T26 + d4*c5*c6*T25 +
		d4*c6*s5*T19) - T19*(d4*s6*T24 + d4*c5*c6*T22 + 
		d4*c6*s5*T21)) + dq2*(T21*((c1*c2*c4 + c1*c3*s2*s4)*T23 + 
		d4*c5*s6*(c1*c2*s4 - c1*c3*c4*s2) + d4*c1*s2*s3*s5*s6) +
		T19*(T27*T23 + d4*c5*s6*T28 + d4*s1*s2*s3*s5*s6) - 
		s1*s2*s3*(d4*c5*s6*T25 - T23*T26 + d4*s5*s6*T19) - 
		c1*s2*s3*(d4*c5*s6*T22 - T24*T23 + d4*s5*s6*T21)) + 
		dq4*(T21*(T23*T25 + d4*c5*s6*T26) - T19*(T22*T23 + d4*c5*s6*T24));

	dJ[2][4] = dq4*(T22*(d4*c6*T26 - d4*s6*T30) - (d4*c6*T24 - 
		d4*s6*T31)*T25 + T24*(d4*c6*T25 + d4*c5*s6*T26) - (d4*c6*T22 +
		d4*c5*s6*T24)*T26) - dq2*((c1*c2*c4 + c1*c3*s2*s4)*(d4*c6*T24 -
		d4*s6*T31) + T27*(d4*c6*T26 - d4*s6*T30) - 
		T24*(d4*s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5) +
		d4*c6*(c1*c2*c4 + c1*c3*s2*s4)) - (d4*s6*(c5*T28 + 
		s1*s2*s3*s5) + d4*c6*T27)*T26) - dq6*(T24*(d4*s6*T26 + 
		d4*c6*T30) - (d4*s6*T24 + d4*c6*T31)*T26) - dq3*((d4*s6*(s5*T16 -
		c4*c5*T21) + d4*c6*s4*T21)*T26 - T24*(d4*s6*(s5*T18 - 
		c4*c5*T19) + d4*c6*s4*T19) - s4*T21*(d4*c6*T26 - d4*s6*T30) +
		s4*T19*(d4*c6*T24 - d4*s6*T31)) - dq5*(d4*s6*T32*T26 - d4*s6*T24*T34);

	dJ[2][5] = dq2*(d4*T37*(s5*(c1*c2*s4 - c1*c3*c4*s2) - c1*c5*s2*s3) +
		d4*T32*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 -
		c1*c3*c4*s2) + c1*s2*s3*s5)) + d4*(s5*T28 - c5*s1*s2*s3)*T36 +
		d4*T34*(c6*T27 + s6*(c5*T28 + s1*s2*s3*s5))) - dq5*(d4*T37*T30 -
		d4*T31*T36) + dq6*(d4*(s6*T24 + c6*T31)*T34 - d4*T32*(s6*T26 +
		c6*T30)) + dq4*(d4*T32*(c6*T25 + c5*s6*T26) - d4*T34*(c6*T22 +
		c5*s6*T24) - d4*s5*T24*T36 + d4*s5*T37*T26) + dq3*(d4*(c5*T16 +
		c4*s5*T21)*T36 - d4*T37*(c5*T18 + c4*s5*T19) - 
		d4*T34*(s6*(s5*T16 - c4*c5*T21) + c6*s4*T21) +
		d4*T32*(s6*(s5*T18 - c4*c5*T19) + c6*s4*T19));

	dJ[2][6] = 0;

	dJ[3][0] = 0;

	dJ[3][1] = -dq1*c1;

	dJ[3][2] = dq2*c1*c2 - dq1*s1*s2;

	dJ[3][3] = dq1*T21 - dq3*T18 - dq2*c1*s2*s3;

	dJ[3][4] = dq4*T25 + dq1*T24 + dq2*(c1*c2*c4 + c1*c3*s2*s4) + dq3*s4*T19;

	dJ[3][5] = dq1*T32 + dq3*(c5*T18 + c4*s5*T19) - dq2*(s5*(c1*c2*s4 -
		c1*c3*c4*s2) - c1*c5*s2*s3) + dq5*T30 - dq4*s5*T26;

	dJ[3][6] = dq2*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 -
		c1*c3*c4*s2) + c1*s2*s3*s5)) + dq3*(s6*(s5*T18 - c4*c5*T19) +
		c6*s4*T19) + dq4*(c6*T25 + c5*s6*T26) - dq6*(s6*T26 +
		c6*T30) + dq1*T37 + dq5*s6*T34;

	dJ[4][0] = 0;

	dJ[4][1] = -dq1*s1;

	dJ[4][2] = dq1*c1*s2 + dq2*c2*s1;

	dJ[4][3] = dq1*T19 + dq3*T16 - dq2*s1*s2*s3;

	dJ[4][4] = dq1*T26 - dq4*T22 + dq2*T27 - dq3*s4*T21;

	dJ[4][5] = dq1*T34 - dq3*(c5*T16 + c4*s5*T21) - 
		dq2*(s5*T28 - c5*s1*s2*s3) - dq5*T31 + dq4*s5*T24;

	dJ[4][6] = dq2*(c6*T27 + s6*(c5*T28 + s1*s2*s3*s5)) - 
		dq4*(c6*T22 + c5*s6*T24) - dq3*(s6*(s5*T16 - c4*c5*T21) +
		c6*s4*T21) + dq1*T36 + dq6*(s6*T24 + c6*T31) - dq5*s6*T32;

	dJ[5][0] = 0;

	dJ[5][1] = 0;

	dJ[5][2] = -dq2*s2;

	dJ[5][3] = - dq2*c2*s3 - dq3*c3*s2;

	dJ[5][4] = - dq2*T3 - dq4*T11 - dq3*s2*s3*s4;

	dJ[5][5] = dq2*T8 - dq5*T29 + dq3*(c3*c5*s2 - c4*s2*s3*s5) - dq4*s5*T12;

	dJ[5][6] = dq6*(c6*T29 - s6*T12) - dq4*(c6*T11 - c5*s6*T12) +
		dq3*(s6*(c3*s2*s5 + c4*c5*s2*s3) - c6*s2*s3*s4) -
		dq2*(s6*T10 + c6*T3) - dq5*s6*T33;


	return 0;
}





int get_link_point_jacobi0(double Jacobi0[6][DOF],double* r,LINK_POINT lp,double dis2joint,double l[4])
{
	double L01 = 0;
	double L34 = 0;
	double L45 = 0;
	double L7t = 0;

	// step 1: �����������α任����

	// ����T01:	T01 = trans(0,0,l_01)*rot_z(r(1));
	// ����T12: T12 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(2));
	// ����T23: T23 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(3));

	// ����T34:	T34 = trans(0,0,l_34)*rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(4));

	// ����T45:	T45 = trans(l_45,0,0)*rot_y(pi/2)*rot_z(pi/2)*rot_z(r(5));
	// ����T56:	T56 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(6));
	// ����T67:	T67 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(7));
	// ����T7t:	T7t = trans(0,0,l_7t);

	switch(lp)
	{
	case BASE:
		break;
	case REFERENCE:
		break;
	case FLANGE:
		L7t = l[3] + dis2joint;
		L45 = l[2];
		L34 = l[1];
		L01 = l[0];
		break;
	case WRIST:
		L7t = dis2joint;
		L45 = l[2];
		L34 = l[1];
		L01 = l[0];
		break;
	case ELBOW:
		L7t = 0;
		L45 = dis2joint;
		L34 = l[1];
		L01 = l[0];
		break;
	case SHOULDER:
		L7t = 0;
		L45 = 0;
		L34 = dis2joint;
		L01 = l[0];
		break;
	}


	double rotAngle[DOF][3] = {0.0};

	switch(lp)
	{
	case BASE:
		break;
	case REFERENCE:
		break;

	case FLANGE:

	case WRIST:
		
		rotAngle[4][0] = PI/2;
		rotAngle[4][1] = PI/2;
		rotAngle[4][2] = r[4];

		rotAngle[5][0] = -PI/2;
		rotAngle[5][1] = -PI/2;
		rotAngle[5][2] = r[5];

		rotAngle[6][0] = PI/2;
		rotAngle[6][1] = PI/2;
		rotAngle[6][2] = r[6];

	case ELBOW:
		
		rotAngle[3][0] = -PI/2;
		rotAngle[3][1] = -PI/2;
		rotAngle[3][2] = r[3];

	case SHOULDER:
		
		rotAngle[0][2] = r[0];

		rotAngle[1][0] = -PI/2;
		rotAngle[1][1] = -PI/2;
		rotAngle[1][2] = r[1];

		rotAngle[2][0] = PI/2;
		rotAngle[2][1] = PI/2;
		rotAngle[2][2] = r[2];
	}

	double T01[4][4],T12[4][4],T23[4][4],T34[4][4],T45[4][4],T56[4][4],T67[4][4],T7t[4][4];
	double T02[4][4],T03[4][4],T04[4][4],T05[4][4],T06[4][4],T07[4][4],T0t[4][4];
	double T6t[4][4],T5t[4][4],T4t[4][4],T3t[4][4],T2t[4][4],T1t[4][4];

	double l_tmp[3];
	double R1_tmp[4][4],R2_tmp[4][4],R3_tmp[4][4],R4_tmp[4][4];

	// ����T01:	T01 = trans(0,0,l_01)*rot_z(r(1));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L01;
	trans(R1_tmp,l_tmp);
	rot_z(R2_tmp,rotAngle[0][2]);
	M4p4(T01,R1_tmp,R2_tmp);

	// ����T12: T12 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(2));
	rot_x(R1_tmp,rotAngle[1][0]);
	rot_z(R2_tmp,rotAngle[1][1]);
	rot_z(R3_tmp,rotAngle[1][2]);
	M4p4p4(T12,R1_tmp,R2_tmp,R3_tmp);

	// ����T23: T23 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(3));
	rot_y(R1_tmp,rotAngle[2][0]);
	rot_z(R2_tmp,rotAngle[2][1]);
	rot_z(R3_tmp,rotAngle[2][2]);
	M4p4p4(T23,R1_tmp,R2_tmp,R3_tmp);

	// ����T34:	T34 = trans(0,0,l_34)*rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(4));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L34;
	trans(R1_tmp,l_tmp);
	rot_x(R2_tmp,rotAngle[3][0]);
	rot_z(R3_tmp,rotAngle[3][1]);
	M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
	rot_z(R1_tmp,-rotAngle[3][2]);// iiwa�ĵ������������Ƿ���
	M4p4(T34,R4_tmp,R1_tmp);

	// ����T45:	T45 = trans(l_45,0,0)*rot_y(pi/2)*rot_z(pi/2)*rot_z(r(5));
	l_tmp[0]=L45, l_tmp[1]=0, l_tmp[2]=0;
	trans(R1_tmp,l_tmp);
	rot_y(R2_tmp,rotAngle[4][0]);
	rot_z(R3_tmp,rotAngle[4][1]);
	M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
	rot_z(R1_tmp,rotAngle[4][2]);
	M4p4(T45,R4_tmp,R1_tmp);

	// ����T56:	T56 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(6));
	rot_x(R1_tmp,rotAngle[5][0]);
	rot_z(R2_tmp,rotAngle[5][1]);
	rot_z(R3_tmp,rotAngle[5][2]);
	M4p4p4(T56,R1_tmp,R2_tmp,R3_tmp);

	// ����T67:	T67 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(7));
	rot_y(R1_tmp,rotAngle[6][0]);
	rot_z(R2_tmp,rotAngle[6][1]);
	rot_z(R3_tmp,rotAngle[6][2]);
	M4p4p4(T67,R1_tmp,R2_tmp,R3_tmp);

	// ����T7t:	T7t = trans(0,0,l_7t);
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L7t;
	trans(T7t,l_tmp);

	// ����T02��T03��T04��T05��T06��T07��T0t
	M4p4(T02,T01,T12);
	M4p4(T03,T02,T23);
	M4p4(T04,T03,T34);
	M4p4(T05,T04,T45);
	M4p4(T06,T05,T56);
	M4p4(T07,T06,T67);
	M4p4(T0t,T07,T7t);

	// ����T1t��T2t��T3t��T4t��T5t��T6t
	M4p4(T6t,T67,T7t);
	M4p4(T5t,T56,T6t);
	M4p4(T4t,T45,T5t);
	M4p4(T3t,T34,T4t);
	M4p4(T2t,T23,T3t);
	M4p4(T1t,T12,T2t);

	//-------------------------------------------------------------------------------------------------------------

	//method for Jacobin matrix in word coordinates by Whitney in 1972
	//	J = [J1,J2,...,Jn]
	//Ji = [cross(Zi0,Pni0);Zi0] = [cross(Zi0,Ri0*Pni);Zi0]
	//which, Pni0 is projection in word coordinates of iPn
	//	Pni is the end effector origin coord in i coordinate
	//	Ri0 is i coordinate rotation according to word coordinate
	//	Zi0 = Ri0*[0;0;1]; Pni0 = Ri0*Pni = Ri0*Tni(1:3,4)

	double Zi[3] = {0.0};
	double Pni[3] = {0.0};
	double tempT[3] = {0.0};
	double tempR[3][3] = {{0.0}};

	double J[DOF][6] = {{0.0}};

	int index = 0;

	switch(lp)
	{
	case BASE:
		break;
	case REFERENCE:
		break;

	case FLANGE:
	case WRIST:
		index = 7;
		break;
	case ELBOW:
		index = 4;
		break;
	case SHOULDER:
		index = 3;
		break;
	}
	if (index >= 3)
	{
		// ���� J1
		Zi[0] = T01[0][2];
		Zi[1] = T01[1][2];
		Zi[2] = T01[2][2];
		homogeneous2rot(T01,tempR);
		tempT[0] = T1t[0][3];
		tempT[1] = T1t[1][3];
		tempT[2] = T1t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[0],Zi,Pni);
		memcpy(J[0]+3,Zi,sizeof(Zi));

		// ���� J2
		Zi[0] = T02[0][2];
		Zi[1] = T02[1][2];
		Zi[2] = T02[2][2];
		homogeneous2rot(T02,tempR);
		tempT[0] = T2t[0][3];
		tempT[1] = T2t[1][3];
		tempT[2] = T2t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[1],Zi,Pni);
		memcpy(J[1]+3,Zi,sizeof(Zi));

		// ���� J3
		Zi[0] = T03[0][2];
		Zi[1] = T03[1][2];
		Zi[2] = T03[2][2];
		homogeneous2rot(T03,tempR);
		tempT[0] = T3t[0][3];
		tempT[1] = T3t[1][3];
		tempT[2] = T3t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[2],Zi,Pni);
		memcpy(J[2]+3,Zi,sizeof(Zi));
	}

	if (index >= 4)
	{
		// ���� J4
		Zi[0] = T04[0][2];
		Zi[1] = T04[1][2];
		Zi[2] = T04[2][2];
		homogeneous2rot(T04,tempR);
		tempT[0] = T4t[0][3];
		tempT[1] = T4t[1][3];
		tempT[2] = T4t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[3],Zi,Pni);
		memcpy(J[3]+3,Zi,sizeof(Zi));

		// iiwa�ĵ������������Ƿ���
		for(int i=0; i<6; i++)
			J[3][i] = - J[3][i];
	}

	if (index >= 7)
	{
		// ���� J5
		Zi[0] = T05[0][2];
		Zi[1] = T05[1][2];
		Zi[2] = T05[2][2];
		homogeneous2rot(T05,tempR);
		tempT[0] = T5t[0][3];
		tempT[1] = T5t[1][3];
		tempT[2] = T5t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[4],Zi,Pni);
		memcpy(J[4]+3,Zi,sizeof(Zi));

		// ���� J6
		Zi[0] = T06[0][2];
		Zi[1] = T06[1][2];
		Zi[2] = T06[2][2];
		homogeneous2rot(T06,tempR);
		tempT[0] = T6t[0][3];
		tempT[1] = T6t[1][3];
		tempT[2] = T6t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[5],Zi,Pni);
		memcpy(J[5]+3,Zi,sizeof(Zi));

		// ���� J7
		Zi[0] = T07[0][2];
		Zi[1] = T07[1][2];
		Zi[2] = T07[2][2];
		homogeneous2rot(T07,tempR);
		tempT[0] = T7t[0][3];
		tempT[1] = T7t[1][3];
		tempT[2] = T7t[2][3];
		M3p3(Pni,tempR,tempT);
		cross3(J[6],Zi,Pni);
		memcpy(J[6]+3,Zi,sizeof(Zi));
	}

	for (int i=0; i<6; i++)
	{
		for (int j=0; j<DOF; j++)
		{
			Jacobi0[i][j] = J[j][i];
		}
	}

	return 0;
}







double get_phai(double Tt0[4][4], double* r, double L[4])
{

	double L01 = L[0];
	double L34 = L[1];
	double L45 = L[2];
	double L7t = L[3];

	double T01[4][4],T12[4][4],T23[4][4];
	double T03[4][4];

	double l_tmp[3];
	double R1_tmp[4][4],R2_tmp[4][4],R3_tmp[4][4];

	// ����T01:	T01 = trans(0,0,l_01)*rot_z(r(1));
	l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L01;
	trans(R1_tmp,l_tmp);
	rot_z(R2_tmp,r[0]);
	M4p4(T01,R1_tmp,R2_tmp);
	// ����T12: T12 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(2));
	rot_x(R1_tmp,-PI/2);
	rot_z(R2_tmp,-PI/2);
	rot_z(R3_tmp,r[1]);
	M4p4p4(T12,R1_tmp,R2_tmp,R3_tmp);
	// ����T23: T23 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(3));
	rot_y(R1_tmp,PI/2);
	rot_z(R2_tmp,PI/2);
	rot_z(R3_tmp,r[2]);
	M4p4p4(T23,R1_tmp,R2_tmp,R3_tmp);
	// ����T03��T04��T05��T0t
	M4p4p4(T03,T01,T12,T23);

	double phai = 0;

	double R03_0_T[3][3],Rphai0[3][3],R03[3][3];
	homogeneous2rot(T03,R03);
	M3p3(Rphai0,R03,R03_0_T);

	// step 2������������
	double Xsw[3],vXsw[3];
	Xsw[0] = Tt0[0][3] - Tt0[0][2]*L7t;
	Xsw[1] = Tt0[1][3] - Tt0[1][2]*L7t;
	Xsw[2] = Tt0[2][3] - Tt0[2][2]*L7t - L01;

	double dd = sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]+Xsw[2]*Xsw[2]);

	vXsw[0] = Xsw[0]/dd;
	vXsw[1] = Xsw[1]/dd;
	vXsw[2] = Xsw[2]/dd;

	matrix2rot(Rphai0,vXsw,&phai);

	return phai;
}

int get_coefficient(double coe[6][7], double Tt0[4][4], double* r, double L[4])
{

	double dbs = L[0],dwt = L[3];

	double EndPose[6] = {0.0};
	homogeneous2pose(Tt0,EndPose);

	double a = EndPose[3], b = EndPose[4], g = EndPose[5];
	double xd = EndPose[0],yd = EndPose[1],zd = EndPose[2];

	double Ca = cos(a),		Cb = cos(b),	Cg = cos(g);
	double Sa = sin(a),		Sb = sin(b),	Sg = sin(g);

	double T1 = xd - Ca*Sb*dwt;
	double T2 = dbs - zd + Cb*dwt;
	double T3 = yd - Sa*Sb*dwt;
	double T4 = T1*T1 + T2*T2 + T3*T3;	
	double T5 = sqrt(T4);

	double R03_0[3][3],Xsw[3] = {T1,T2,T3},vXsw[3];

	vXsw[0] = Xsw[0]/T5;
	vXsw[1] = Xsw[1]/T5;
	vXsw[2] = Xsw[2]/T5;

	kinematics_SRS_reference(R03_0, Xsw, r, L);

	double S1 = - R03_0[0][1];
	double S2 = - R03_0[2][0];
	
	double C1 =	  R03_0[1][1];
	double C2 =   R03_0[2][2];

	double S4 = sin(r[3]);
	double C4 = cos(r[3]);

	double T6 = S1*S2*(T1*T1+T2*T2)/T4;
	double T7 = (C2*T3*T2-C1*S2*T3*T1)/T4;
	double T8 = C1*S2*(T3*T3 + T2*T2)/T4;
	double T9 = (C2*T1*T2 - S1*S2*T3*T1)/T4;
	double T10 = (S1*T1*T2 - C1*T3*T2)/T4;
	double T11 = (T1*T1+T2*T2)/T4;
	double T12 = (T2*T2+T3*T3)/T4;
	double T13 = (T1*T1+T3*T3)/T4;
	double T14 = (C2*S1*T1*T3)/T4;
	double T15 = (S1*S2*T1*T3)/T4;
	double T16 = (C1*C2*T1*T2)/T4;
	double T17 = (C2*S1*T3*T2)/T4;
	double T18 = (C1*S2*T1*T2)/T4;
	double T19 = (S1*S2*T3*T2)/T4;
	double T20 = (C1*C2*T1*T3)/T4;
	double T21 = (C1*S2*T1*T3)/T4;
	double T22 = ((S2*T1*T2)/T4 - C1*C2*(T12) + T14);
	double T23 = (C1*S2*(T12) + (C2*T1*T2)/T4 - T15);

	// theta1
	coe[0][0] = -((C2*T1)/T5 + (C1*S2*T2)/T5);
	coe[1][0] = T6+T7;
	coe[2][0] = -(S1*S2*(T11 - 1.0) + T7);
	coe[3][0] = ((C2*T3)/T5 + (S1*S2*T2)/T5);
	coe[4][0] = T8 + T9;
	coe[5][0] = - C1*S2*(T12 - 1.0) - (C2*T1*T2)/T4 + (S1*S2*T3*T1)/T4;
	// theta3
	coe[0][2] = -((C1*T1)/T5 + (S1*T3)/T5);
	coe[1][2] = T10;
	coe[2][2] = - (S1*T1*T2)/T4 + (C1*T3*T2)/T4;
	coe[3][2] = ((C2*S1*T1)/T5 - (C1*C2*T3)/T5);
	coe[4][2] = (T16 - S2*T13 + T17);
	coe[5][2] = S2*(T13 - 1.0) - T16 - T17;
	// theta5
	coe[0][4] = (Cb*((1/T5)*C1*T1 + (1/T5)*S1*T3) + (1/T5)*C1*Ca*Sb*T2 + (1/T5)*S1*Sa*Sb*T2);
	coe[1][4] = -((Cb*(T10) - Sa*Sb*(C1*(T11) + (S1*T1*T3)/T4) + Ca*Sb*(S1*(T12) + (C1*T1*T3)/T4)));
	coe[2][4] = (Cb*(T10) - Sa*Sb*(C1*(T11 - 1) + (S1*T1*T3)/T4) + Ca*Sb*(S1*(T12 - 1) + (C1*T1*T3)/T4));
	coe[3][4] = (1/T5)*(Cb*(C4*(C2*S1*T1 - C1*C2*T3) - S4*(S1*S2*T1 - C1*S2*T3)) + Sa*Sb*(C4*(S2*T1 -
		C1*C2*T2) + S4*(C2*T1 + C1*S2*T2)) - Ca*Sb*(C4*(S2*T3 - C2*S1*T2) + S4*(C2*T3 + S1*S2*T2)));
	coe[4][4] = -((Ca*Sb*(C4*T22 + S4*T23) - Cb*(C4*(T16 - S2*(T13) + T17) - S4*(C2*(T13) + T18 + T19)) +
		Sa*Sb*(C4*((S2*T3*T2)/T4 - C2*S1*(T11) + T20) + S4*(S1*S2*(T11) + (C2*T3*T2)/T4 - T21))));
	coe[5][4] = - Cb*(C4*(T16 - S2*(T13 - 1) + T17) - S4*(C2*(T13 - 1) + T18 + T19)) + Ca*Sb*(C4*((S2*T1*T2)/T4 -
		C1*C2*(T12 - 1) + T14) + S4*(C1*S2*(T12 - 1) + (C2*T1*T2)/T4 - T15)) + Sa*Sb*(C4*((S2*T3*T2)/T4 -
		C2*S1*(T11 - 1) + T20) + S4*(S1*S2*(T11 - 1) + (C2*T3*T2)/T4 - T21));
	// theta7
	coe[0][6] = -(1/T5)*((C4*(C2*T1 + C1*S2*T2) - S4*(S2*T1 - C1*C2*T2))*(Ca*Cg - Cb*Sa*Sg) +
		(Cg*Sa + Ca*Cb*Sg)*(C4*(C2*T3 + S1*S2*T2) - S4*(S2*T3 - C2*S1*T2)) -
		Sb*Sg*(S4*(C2*S1*T1 - C1*C2*T3) + C4*(S1*S2*T1 - C1*S2*T3)));
	coe[1][6] = ((Ca*Cg - Cb*Sa*Sg)*(C4*(S1*S2*(T11) + (C2*T3*T2)/T4 - T21) -
		S4*((S2*T3*T2)/T4 - C2*S1*(T11) + T20)) - (Cg*Sa + Ca*Cb*Sg)*(C4*T23 -
		S4*T22) + Sb*Sg*(C4*(C2*(T13) + T18 + T19) + S4*(T16 - S2*(T13) + T17)));
	coe[2][6] = ((Cg*Sa + Ca*Cb*Sg)*(C4*(C1*S2*(T12 - 1) + (C2*T1*T2)/T4 - T15)
		- S4*((S2*T1*T2)/T4 - C1*C2*(T12 - 1) + T14)) - (Ca*Cg - Cb*Sa*Sg)*(C4*(S1*S2*(T11 - 1) +
		(C2*T3*T2)/T4 - T21) - S4*((S2*T3*T2)/T4 - C2*S1*(T11 - 1) + T20)) -
		Sb*Sg*(C4*(C2*(T13 - 1) + T18 + T19) + S4*(T16 - S2*(T13 - 1) + T17)));
	coe[3][6] = ((C4*((1/T5)*C2*T1 + (1/T5)*C1*S2*T2) - S4*((1/T5)*S2*T1 -
		(1/T5)*C1*C2*T2))*(Ca*Sg + Cb*Cg*Sa) + (Sa*Sg - Ca*Cb*Cg)*(C4*((1/T5)*C2*T3 +
		(1/T5)*S1*S2*T2) - S4*((1/T5)*S2*T3 - (1/T5)*C2*S1*T2)) + Cg*Sb*(S4*((1/T5)*C2*S1*T1 -
		(1/T5)*C1*C2*T3) + C4*((1/T5)*S1*S2*T1 - (1/T5)*C1*S2*T3)));
	coe[4][6] = ((Sa*Sg - Ca*Cb*Cg)*(C4*T23 - S4*T22) - (Ca*Sg + Cb*Cg*Sa)*(C4*(S1*S2*(T11) +
		(C2*T3*T2)/T4 - T21) - S4*((S2*T3*T2)/T4 - C2*S1*(T11) + T20)) +
		Cg*Sb*(C4*(C2*(T13) + T18 + T19) + S4*(T16 - S2*(T13) + T17)));
	coe[5][6] = (Ca*Sg + Cb*Cg*Sa)*(C4*(S1*S2*(T11 - 1) + (C2*T3*T2)/T4 - T21) -
		S4*((S2*T3*T2)/T4 - C2*S1*(T11 - 1) + T20)) - (Sa*Sg - Ca*Cb*Cg)*(C4*(C1*S2*(T12 - 1) +
		(C2*T1*T2)/T4 - T15) - S4*((S2*T1*T2)/T4 - C1*C2*(T12 - 1) + T14)) -
		Cg*Sb*(C4*(C2*(T13 - 1) + T18 + T19) + S4*(T16 - S2*(T13 - 1) + T17));
	// theta2
	coe[0][1] = ((S1*S2*T1)/T5 - (C1*S2*T3)/T5);
	coe[1][1] = (C2*(T13) + T18 + T19);
	coe[2][1] = - C2*(T13 - 1.0) - T18 - T19;
	// theta6
	coe[0][5] = (Cb*(S4*((1/T5)*C2*S1*T1 - (1/T5)*C1*C2*T3) + C4*((1/T5)*S1*S2*T1 -
		(1/T5)*C1*S2*T3)) - Sa*Sb*(C4*((1/T5)*C2*T1 + (1/T5)*C1*S2*T2) -
		S4*((1/T5)*S2*T1 - (1/T5)*C1*C2*T2)) + Ca*Sb*(C4*((1/T5)*C2*T3 +
		(1/T5)*S1*S2*T2) - S4*((1/T5)*S2*T3 - (1/T5)*C2*S1*T2)));
	coe[1][5] = (Cb*(C4*(C2*(T13) + T18 + T19) + S4*(T16 - S2*(T13) + T17)) +
		Ca*Sb*(C4*T23 - S4*T22) + Sa*Sb*(C4*(S1*S2*(T11) + (C2*T3*T2)/T4 -
		T21) - S4*((S2*T3*T2)/T4 - C2*S1*(T11) + T20)));
	coe[2][5] = - Cb*(C4*(C2*(T13 - 1) + T18 + T19) + S4*(T16 - S2*(T13 - 1) +
		T17)) - Ca*Sb*(C4*(C1*S2*(T12 - 1) + (C2*T1*T2)/T4 - T15) -
		S4*((S2*T1*T2)/T4 - C1*C2*(T12 - 1) + T14)) - Sa*Sb*(C4*(S1*S2*(T11 - 1) +
		(C2*T3*T2)/T4 - T21) - S4*((S2*T3*T2)/T4 - C2*S1*(T11 - 1) + T20));


	return 0;
}

int kinematics_SRS_redundant_jacobi0(double Jacobi0[7][DOF],double* r, double L[4])
{
	double Tt0[4][4] = {{0.0}};
	kinematics_SRS_jacobi0(Jacobi0,Tt0,r,L);

	double phai = get_phai(Tt0,r,L);	
	double Cphi = cos(phai);
	double Sphi = sin(phai);

	double coe[6][7] = {{0.0}};
	get_coefficient(coe,Tt0,r,L);


	Jacobi0[6][3] = 0;

	for (int i=0; i<DOF; i++)
	{
		if (i == 0 || i == 2 || i == 4 || i == 6)
		{
			double Cr = cos(r[i]);

			double dr_dt = 1.0/(Cr*Cr);

			double gx = coe[0][i]*Sphi + coe[1][i]*Cphi + coe[2][i];
			double hx = coe[3][i]*Sphi + coe[4][i]*Cphi + coe[5][i];
			double dg_dt = coe[0][i]*Cphi - coe[1][i]*Sphi;
			double dh_dt = coe[3][i]*Cphi - coe[4][i]*Sphi;

			double dp_dt = (dg_dt*hx - dh_dt*gx)/(hx*hx);

			Jacobi0[6][i] = dr_dt/dp_dt;
		}

		if (i == 1 || i == 5)
		{
			double Sr = sin(r[i]);

			double dr_dt = -Sphi;

			double dp_dt = coe[0][i]*Cphi - coe[1][i]*Sphi;

			Jacobi0[6][i] = dr_dt/dp_dt;
		}
	}

	return 0;
}