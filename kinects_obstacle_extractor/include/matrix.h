


#if !defined(MATRIX_)
#define MATRIX_

#define MAX_ITERA 100
#define MIN_DOUBLE (1e-30)

#include <stdio.h>
#include <iostream>
#include "robotMath.h"

// ��̬����ģ�� Matrix
template<typename Type>class Matrix
{
public:
	Matrix();												// Ĭ�Ϲ��캯����1x1
	Matrix(int nRows, int nCols);							// ָ�����й��캯��
	Matrix(const Matrix<Type>& other);						// �������캯��
	Matrix(int nRows, int nCols, Type value[]);				// ָ�����ݹ��캯��
	~Matrix();												// ��������
		
	bool	ZeroInitialize();								// ����ȫ����ʼ��
	bool	IdentityInitialize();							// ��ʼ��Ϊ��λ����
	bool	Init(int nRows, int nCols);						// ��ʼ��Ϊȫ������	
	
	bool    SetData(Type value[]);							// ���þ�����ֵ
	bool	SetElement(int nRow, int nCol, Type value);		// ����ָ��Ԫ�ص�ֵ	

	void	printf() const;									// ��ӡ����������
	Type*   GetData() const;								// ��ȡ����������
	int		GetNumRows() const;								// ��ȡ����������
	int		GetNumColumns() const;							// ��ȡ����������
	Type	GetElement(int nRow, int nCol) const;			// ��ȡָ��Ԫ�ص�ֵ
	int     GetRowVector(int nRow, Type* pVector) const;	// ��ȡ������ָ���о���
	int     GetColVector(int nCol, Type* pVector) const;	// ��ȡ������ָ���о���	

	int		GetRank() const;								// ��ȡ��������

	Matrix<Type>	inv()const;									// ��������
	Matrix<Type>	pinv()const;
	bool ExchangeRow(int nRow1, int nRow2);						// ��������
	bool PrimaryShiftRow(int nRow, Type dMultiple);				// dMultiple * nRow
	bool PrimaryShiftRow(int nRow1, Type dMultiple, int nRow2);	// nRow1 - dMultiple * nRow2
	bool ExchangeCol(int nCol1, int nCol2);						// ��������
	bool PrimaryShiftCol(int nCol, Type dMultiple);				// dMultiple * nRow
	bool PrimaryShiftCol(int nCol1, Type dMultiple, int nCol2);	// nRow1 - dMultiple * nRow2

	Matrix<Type>	 operator*(const Type& ratio);					// ����
	Matrix<Type>     operator=(const Matrix<Type>& other) ;			// ������ֵ
	Matrix<Type>	 operator+(const Matrix<Type>& other) const;	// ��������
	Matrix<Type>	 operator-(const Matrix<Type>& other) const;	// ��
	Matrix<Type>	 operator*(const Matrix<Type>& other) const;	// ��
	bool	         operator==(const Matrix<Type>& other) const;	// �ж�2�������Ƿ�����

	//�ϲ��������󣬰��еķ�ʽ�����磺 3*1��3*1���ϲ�Ϊ6*1,������������									
	static Matrix<Type>	CombineMatrixRows(Matrix<Type> Matrix1, Matrix<Type> Matrix2);
	
	//�ϲ��������󣬰��еķ�ʽ��Matrix1Ϊ���ۼӵ�Ŀ��������Matrix2Ϊ��������					
	static Matrix<Type>	CombineMatrixColumns(Matrix<Type> Matrix1, Matrix<Type> Matrix2);		
	
	//��ȡ�Ӿ���,���Ŵ�0��ʼ,�ӵ�irowstart�е���irowend�У��ӵ�icolumstart�е���icolumend��
	Matrix<Type> Getsubmatrix(int irowstart, int irowend, int icolumstart, int icolumend);	

	//�����Ӿ���,���Ŵ�0��ʼ,�ӵ�irowstart�е���irowend�У��ӵ�icolumstart�е���icolumend��
	bool Setsubmatrix(int irowstart, int irowend, int icolumstart, int icolumend,Matrix<Type> subMatrix);	
  
public:
	bool isValid;				// ��־λ�������Ƿ���Ч
protected:
	int	m_nNumColumns;			// ��������
	int	m_nNumRows;				// ��������
	Type*	m_pData;			// �������ݻ�����



	// ����һ��α��
	int _pinv(Type* pinv_a,Type* a,int m, int n) const;
	void sss(double fg[2],double cs[2])const;
	void damul(double a[],double b[],int m,int n,int k,double c[])const;
	void ppp(double a[],double e[],double s[],double v[],int m,int n)const;
	int dluav(double a[],int m,int n,double u[],double v[],double eps,int ka)const;
	double norm(double a[],int m,int n,double u[],double v[],double eps,int ka)const;


	//��������α��
	int Jpinv(double** J,int m,int n,double **Jinv) const;

	static int Singular_Value_Decomposition(double* A, int nrows, int ncols, double* U,
		double* singular_values, double* V, double* dummy_array);

	static void Householders_Reduction_to_Bidiagonal_Form(double* A, int nrows,
		int ncols, double* U, double* V, double* diagonal, double* superdiagonal);

	static int  Givens_Reduction_to_Diagonal_Form(int nrows, int ncols,
		double* U, double* V, double* diagonal, double* superdiagonal);

	static void Sort_by_Decreasing_Singular_Values(int nrows, int ncols,
		double* singular_values, double* U, double* V);

	static void Singular_Value_Decomposition_Solve(double* U, double* D, double* V,
		double tolerance, int nrows, int ncols, double *B, double* x);

	static	void Singular_Value_Decomposition_Inverse(double* U, double* D, double* V,
		double tolerance, int nrows, int ncols, double *Astar);
};

// ��������ģ��
template<typename Type, int nRows, int nCols> 
class VecMatrix:public Matrix<Type>
{
public:
	VecMatrix():Matrix<Type>(nRows,nCols){};
	~VecMatrix(){};
};

// ��̬����
typedef Matrix<double> CMatrixXd;

// ��������
typedef VecMatrix<double,3,3> CMatrix3d;
typedef VecMatrix<double,4,4> CMatrix4d;
typedef VecMatrix<double,6,6> CMatrix6d;

// ��������
typedef VecMatrix<double,3,1> CVector3x1d;
typedef VecMatrix<double,1,3> CVector1x3d;
typedef VecMatrix<double,4,1> CVector4x1d;
typedef VecMatrix<double,6,1> CVector6x1d;
typedef VecMatrix<double,16,1> CVector16x1d;

#endif // !defined(MATRIX_H_)
