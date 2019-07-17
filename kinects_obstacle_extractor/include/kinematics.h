/*
 * kinematics.h
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "config.h"
#include <string.h>
#include "robotMath.h"

#define LBS (340)
#define LSE (400)
#define LEW (400)
#define LWT (111)

enum LINK_POINT{BASE,SHOULDER,ELBOW,WRIST,FLANGE,REFERENCE};

double get_phai(double Tt0[4][4], double* r, double L[4]);
int get_coefficient(double coe[6][7],double Tt0[4][4],  double* r, double L[4]);
int kinematics_SRS_redundant_jacobi0(double Jacobi0[7][DOF],double* r, double L[4]);

int get_link_point_jacobi0(double Jacobi0[6][DOF],double* r,LINK_POINT lp,double dis2joint,double L[4]);

int kinematics_SRS_jacobi0(double Jacobi0[][DOF],double Tt0[4][4], double* r, double* l);
int kinematics_SRS_dot_jacobi0(double dJacobi0[6][DOF],double* r, double* rv, double* l);

int kinematics_SRS_forward(double P[7], double R[3][3], double* phai, double pp[][3], double* r, double* l);
int kinematics_SRS_inverse(double* r, double Pd[3], double Rd[3][3], double* phai, double* l, double* r_ref, double r_lim[][2]);

class R_KINE
{
public:
	R_KINE(CONFIG* config)
	{
		L[0] = LBS; 
		L[1] = LSE;
		L[2] = LEW;
		L[3] = LWT; 
		_config = config;
		memset(pp,0,sizeof(pp));
	}

	~R_KINE(){;}

	CONFIG* _config;

	double L[4];
	double pp[6][3]; //{1,2,3}����ԭ�㣬{4}����ԭ�㣬{5,6,7}����ԭ�㣬�ο�ƽ����{4}����ԭ��


	/////////////////////////////////////////////////////////////////////////////////////////
	int fkine(double T[4][4], double euler[3], double* phi, double angle[DOF])
	{
		double X[DOF] = {0.0};
		double R[3][3] = {{0.0}};

		int result = kinematics_SRS_forward(X,R,phi,pp,angle,L);

		for (int i=0; i<3; i++)
		{
			for (int j=0; j<3; j++)
			{
				T[i][j] = R[i][j];
			}
			T[i][3] = X[i];

			if (euler!=NULL)
			{
				euler[i] = X[i+3];
			}
		}

		T[3][3] = 1.0;

		return result;
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	int fkine(double pose[DOF],double* phi,double angle[DOF])
	{
		double R[3][3] = {{0.0}};
		return kinematics_SRS_forward(pose,R,phi,pp,angle,L);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////
	int fkine(double pose[DOF],double angle[DOF])
	{
		double phi = 0;
		return fkine(pose,&phi,angle);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////
	int fkine(double Td[4][4],double angle[DOF])
	{
		double phi = 0;
		return fkine(Td, NULL,  &phi,  angle);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////
	int fkine(double Td[4][4],double* phi, double angle[DOF])
	{
		return fkine(Td,NULL,phi,angle);
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	int ikine(double angle[DOF],double Td[4][4],double* phi, double ref[DOF])
	{
		double X[3] = {0.0};
		double R[3][3] = {{0.0}};

		for (int i=0; i<3; i++)
		{
			for (int j=0; j<3; j++)
			{
				R[i][j] = Td[i][j];
			}
			X[i] = Td[i][3];
		}

		double r_lim[DOF][2];
		for (int i=0; i<DOF; i++)
		{
			r_lim[i][0] = _config->JointLimitDown[i];
			r_lim[i][1] = _config->JointLimitsUp[i];
		}

		return kinematics_SRS_inverse(angle,X,R,phi,L,ref,r_lim);
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	int ikine(double angle[DOF], double Xd[3], double EulerD[3], double* phi, double ref[DOF])
	{
		double R[3][3] = {{0.0}};
		zyz2matrix(EulerD,R);

		double r_lim[DOF][2];
		for (int i=0; i<DOF; i++)
		{
			r_lim[i][0] = _config->JointLimitDown[i];
			r_lim[i][1] = _config->JointLimitsUp[i];
		}

		return kinematics_SRS_inverse(angle,Xd,R,phi,L,ref,r_lim);
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	int jacob0(double J[6][DOF],double angle[DOF])
	{
		return kinematics_SRS_jacobi0(J,NULL,angle,L);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////
	int dot_jacob0(double dJ[6][DOF],double angle[DOF],double angle_vel[DOF])
	{
		return kinematics_SRS_dot_jacobi0(dJ,angle,angle_vel,L);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////
	int redundant_jacob0(double J[7][7],double angle[DOF])
	{
		return kinematics_SRS_redundant_jacobi0(J,angle,L);
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	int link_point_jacob0(double J[6][DOF],double angle[DOF],LINK_POINT lp,double distance2point)
	{
		return get_link_point_jacobi0(J,angle,lp,distance2point,L);
	}
};


#endif /* KINEMATICS_H_ */
