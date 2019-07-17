

#ifndef ROBOTMATH_H_
#define ROBOTMATH_H_

#include <math.h>
#include <vector>
#include <string.h>
#include <iostream>

#ifndef ACCURACY_FACTOR
#define ACCURACY_FACTOR   (1.e-6)
#endif // !ACCURACY_FACTOR

#ifndef MIN_VALUE_EQ_ZERO
#define MIN_VALUE_EQ_ZERO (1.e-10)
#endif // MIN_VALUE_EQ_ZERO

#ifndef PI
#define PI 3.141592653589793238
#endif // !PI

#ifndef INVALID
#define	INVALID	10
#endif	//!INVALID

#ifndef DEBUG
#define DEBUG	cout
#endif  //!DEBUG

#ifndef ABS
#define ABS(x)  ((x)<0 ? -(x) : (x))
#endif  //!abs

#ifndef MAX
#define MAX(a,b) ((a)>(b) ? (a) : (b))
#endif // max

#ifndef MIN
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#endif // min

using namespace std;

/// Returns the nearest integer of X
//int round(double x);

/// Returns 0 for x==0,1 for x>0, and -1 for x<0.
inline int sign(double x) { return (x > 0) - (x < 0); };

/// Return x for x>y, y for x<=y
inline double fmax(double x, double y){return (x>y)?x:y;};

/// Return y for x>y, x for x<=y
inline double fmin(double x, double y){return (x>y)?y:x;};

/// Returns true if x is close to zero.
inline bool isZero(double x) { return fabs(x) < MIN_VALUE_EQ_ZERO; };

/// Returns true if x is close to or greater than zero.
inline bool isPositive(double x) { return x > -MIN_VALUE_EQ_ZERO; };

/// Returns true if x is close to or smaller than zero.
inline bool isNegative(double x) { return x < MIN_VALUE_EQ_ZERO; };

/// Returns X!
int factorial(int x);

/// Returns norm distance in R3
double norm3(double* a);

/// Returns Euclidean distance in R3
double dis3(double* a, double* b);

/// Returns dot multiplication y=x1.x2
double dot3(double* x1, double* x2);

/// Returns cross multiplication y=x1��x2
void cross3(double* y, double* x1, double* x2);

/// Returns roots of the quadratic equation
int root2(double a, double b, double c, double* x1, double* x2);

/// Returns roots of the cubic equation
int root3(double a, double b, double c, double d, double* z);

/// Returns roots of the quartic equation
int root4(double a0,double b0,double c0,double d0,double e0,double* x);

int crout(double* mid, double* up, double* down, double* b, double* x, int n);

int homogeneous2rot(double T[4][4],double R[3][3]);
int homogeneous2trans(double T[4][4],double X[3]);
int homogeneous2pose(double T[4][4], double X[6]);

int pose2homogeneous(double X[6],double T[4][4]);
int trans2homogeneous(double X[3],double T[4][4]);
int rot2homogeneous(double R[3][3],double T[4][4]);

/// ��ת������ŷ����ת��
int matrix2zyz(double R[3][3], double* zyz);
int zyz2matrix(double* zyz, double R[3][3]);

/// ��Чת��ת������ת�����ı任
int matrix2rot(double R[3][3], double* k, double* t);
int rot2matrix(double R[3][3], double* k, double* t);

/// ��ת��ƽ������
void rot_x(double R[4][4], double t);
void rot_y(double R[4][4], double t);
void rot_z(double R[4][4], double t);
void trans(double R[4][4], double l[3]);
int rotation(double R[3][3],double angle,int axis);

/// �����˷�
void M4p4(double R[4][4], double R1[4][4], double R2[4][4]);
void M3p3(double R[3][3], double R1[3][3], double R2[3][3]);
void M3p3(double R[3], double R1[3][3], double R2[3]);
void M4p4p4(double R[4][4], double R1[4][4], double R2[4][4], double R3[4][4]);

/// ����ת��
void Transpose3(double R[3][3], double R1[3][3]);

#endif // !ROBOTMATH_H_