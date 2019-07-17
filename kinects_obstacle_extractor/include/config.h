



#ifndef CONFIG_H_
#define CONFIG_H_


#pragma  once
#include "robotMath.h"

#ifndef DOF
#define DOF	(7)
#endif

#define vLinearLimit	(1000)				// maximum linear velocity of the flange, mm/s
#define vAngularLimit	(1)					// maximum angular velocity of the flange, rad/s

#define JOINT_VELOCITY_LIMIT_1	(110)		// maximum joint velocity of joint #1, deg/s
#define JOINT_VELOCITY_LIMIT_2	(110)		// maximum joint velocity of joint #2, deg/s
#define JOINT_VELOCITY_LIMIT_3	(110)		// maximum joint velocity of joint #3, deg/s
#define JOINT_VELOCITY_LIMIT_4	(130)		// maximum joint velocity of joint #4, deg/s
#define JOINT_VELOCITY_LIMIT_5	(130)		// maximum joint velocity of joint #5, deg/s
#define JOINT_VELOCITY_LIMIT_6	(180)		// maximum joint velocity of joint #6, deg/s
#define JOINT_VELOCITY_LIMIT_7	(180)		// maximum joint velocity of joint #7, deg/s

#define JOINT_ACCELERATION_LIMIT_1	(300)	// maximum joint acceleration of joint #1, deg/s/s
#define JOINT_ACCELERATION_LIMIT_2	(300)	// maximum joint acceleration of joint #2, deg/s/s
#define JOINT_ACCELERATION_LIMIT_3	(300)	// maximum joint acceleration of joint #3, deg/s/s
#define JOINT_ACCELERATION_LIMIT_4	(300)	// maximum joint acceleration of joint #4, deg/s/s
#define JOINT_ACCELERATION_LIMIT_5	(300)	// maximum joint acceleration of joint #5, deg/s/s
#define JOINT_ACCELERATION_LIMIT_6	(300)	// maximum joint acceleration of joint #6, deg/s/s
#define JOINT_ACCELERATION_LIMIT_7	(300)	// maximum joint acceleration of joint #7, deg/s/s


// reference coordinates of the movement
enum MOVESPACE{JOINT_COORDINATES,BASE_COORDINATES,FLANGE_COORDINATES};	

// configuration of the robot, 
class CONFIG	
{
public:
	CONFIG()
	{
		VelocityLinearLimit = vLinearLimit;
		VelocityAngularLimit = vAngularLimit;

		VelocityJ[0] = 110.0/180.0*PI;
		VelocityJ[1] = 110.0/180.0*PI;
		VelocityJ[2] = 110.0/180.0*PI;
		VelocityJ[3] = 130.0/180.0*PI;
		VelocityJ[4] = 130.0/180.0*PI;
		VelocityJ[5] = 180.0/180.0*PI;
		VelocityJ[6] = 180.0/180.0*PI;

		for(int i=0; i<DOF; i++)
			AccelerationJ[i] = 300.0/180.0*PI;

		JointLimitDown[0] = -160.0/180.0*PI;
		JointLimitsUp[0] =   160.0/180.0*PI;
		JointLimitDown[1] = -110.0/180.0*PI;
		JointLimitsUp[1] =   110.0/180.0*PI;
		JointLimitDown[2] = -160.0/180.0*PI;
		JointLimitsUp[2] =   160.0/180.0*PI;
		JointLimitDown[3] = -110.0/180.0*PI;
		JointLimitsUp[3] =   110.0/180.0*PI;
		JointLimitDown[4] = -160.0/180.0*PI;
		JointLimitsUp[4] =   160.0/180.0*PI;
		JointLimitDown[5] = -110.0/180.0*PI;
		JointLimitsUp[5] =   110.0/180.0*PI;
		JointLimitDown[6] = -160.0/180.0*PI;
		JointLimitsUp[6] =   160.0/180.0*PI;

		setRatio(0.1);
	}

	~CONFIG(){;}

	bool setRatio(double Ratio)
	{
		if (Ratio >= 1.0)
		{
			Ratio = 1.0;
		}

		if (Ratio <=0)
		{
			Ratio = 0.01;
		}

		ratio = Ratio;

		VelocityF = VelocityLinearLimit*ratio;
		AccelerationF = VelocityF*5.0;
		JerkF = AccelerationF*5.0;
		SnapF = JerkF*5.0;

		VelocityFomega = VelocityAngularLimit*ratio;
		AccelerationFomega = VelocityFomega*5.0;
		JerkFomega = AccelerationFomega*5.0;
		SnapFomega = JerkFomega*5.0;

		VelocityJ[0] = JOINT_VELOCITY_LIMIT_1/180.0*PI*ratio;
		VelocityJ[1] = JOINT_VELOCITY_LIMIT_2/180.0*PI*ratio;
		VelocityJ[2] = JOINT_VELOCITY_LIMIT_3/180.0*PI*ratio;
		VelocityJ[3] = JOINT_VELOCITY_LIMIT_4/180.0*PI*ratio;
		VelocityJ[4] = JOINT_VELOCITY_LIMIT_5/180.0*PI*ratio;
		VelocityJ[5] = JOINT_VELOCITY_LIMIT_6/180.0*PI*ratio;
		VelocityJ[6] = JOINT_VELOCITY_LIMIT_7/180.0*PI*ratio;

		AccelerationJ[0] = JOINT_ACCELERATION_LIMIT_1/180.0*PI*ratio;
		AccelerationJ[1] = JOINT_ACCELERATION_LIMIT_2/180.0*PI*ratio;
		AccelerationJ[2] = JOINT_ACCELERATION_LIMIT_3/180.0*PI*ratio;
		AccelerationJ[3] = JOINT_ACCELERATION_LIMIT_4/180.0*PI*ratio;
		AccelerationJ[4] = JOINT_ACCELERATION_LIMIT_5/180.0*PI*ratio;
		AccelerationJ[5] = JOINT_ACCELERATION_LIMIT_6/180.0*PI*ratio;
		AccelerationJ[6] = JOINT_ACCELERATION_LIMIT_7/180.0*PI*ratio;

		for (int i=0; i<DOF;i++)
		{
			JerkJ[i] = AccelerationJ[i]*3.0;
			SnapJ[i] = JerkJ[i]*3.0;
		}

		return true;
	}

public:
	double		ratio;					// �ٶȰٷֱ�
	double		JointLimitsUp[DOF];		// �ؽ���λ, rad
	double		JointLimitDown[DOF];
	double		VelocityJ[DOF];			// �ؽ�����ٶ�, rad/s
	double		AccelerationJ[DOF];		// �ؽ������ٶ�, rad/s/s
	double		JerkJ[DOF];				// �ؽ�������� rad/s/s/s
	double		SnapJ[DOF];				// rad/s/s/s/s

	double		VelocityF;				// ����XYZ������ٶȣ�mm/s
	double		AccelerationF;			// ����XYZ����߼��ٶ�,mm/s/s
	double		JerkF;					// ����XYZ���Ӽ��ٶ�,mm/s/s/s
	double		SnapF;					// mm/s/s/s/s

	double		VelocityFomega;			// ����ABC�����ٶ� rad/s
	double		AccelerationFomega;		// ����ABC���Ǽ��ٶ� rad/s/s
	double		JerkFomega;				// ����ABC���ǼӼ��ٶ� rad/s/s/s
	double		SnapFomega;				// rad/s/s/s/s

private:
	double		VelocityLinearLimit;	// ������������ٶ�mm/s
	double		VelocityAngularLimit;	// ���������ٶ� rad/s
};



#endif // !CONFIG_H_