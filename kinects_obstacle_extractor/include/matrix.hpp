
#include "matrix.h"

//////////////////////////////////////////////////////////////////////
// Ĭ�Ϲ��캯��
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>::Matrix()
{
	m_nNumRows = 1;
	m_nNumColumns = 1;
	m_pData = NULL;
	Init(m_nNumRows, m_nNumColumns);
}

//////////////////////////////////////////////////////////////////////
// ָ�����й��캯��
//
// ������
// 1. int nRows - ָ���ľ�������
// 2. int nCols - ָ���ľ�������
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>::Matrix(int nRows, int nCols)
{
	m_pData = NULL;
	m_nNumRows = nRows;
	m_nNumColumns = nCols;
	Init(m_nNumRows, m_nNumColumns);
}

//////////////////////////////////////////////////////////////////////
// �������캯��
// ������
//   const Matrix& other - Դ����
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>::Matrix(const Matrix& other)
{
	m_pData = NULL;
	m_nNumRows = other.GetNumRows();
	m_nNumColumns = other.GetNumColumns();
	bool bSuccess = Init(m_nNumRows, m_nNumColumns);
	memcpy(m_pData,other.m_pData,sizeof(Type)*m_nNumColumns*m_nNumRows);
}

//////////////////////////////////////////////////////////////////////
// ָ��ֵ���캯��
//
// ������
// 1. int nRows - ָ���ľ�������
// 2. int nCols - ָ���ľ�������
// 3. Type value[] - һά���飬����ΪnRows*nCols���洢������Ԫ�ص�ֵ
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>::
	Matrix(int nRows, int nCols, Type value[])
{
	m_pData = NULL;
	m_nNumRows = nRows;
	m_nNumColumns = nCols;
	Init(m_nNumRows, m_nNumColumns);
	SetData(value);
}

//////////////////////////////////////////////////////////////////////
// ��������
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>::~Matrix()
{
	if (m_pData)
	{
		delete[] m_pData;
		m_pData = NULL;
	}
}

//////////////////////////////////////////////////////////////////////
// ȫ������ʼ������
//
// ����ֵ��bool �ͣ���ʼ���Ƿ��ɹ�
//////////////////////////////////////////////////////////////////////
template<typename Type>bool Matrix<Type>::ZeroInitialize()
{
	for (int i = 0; i < m_nNumRows; i++)
	{
		for (int j = 0; j < m_nNumColumns; j++)
		{
			SetElement(i, j, 0);
		}
	}

	isValid = true;
	return true;
}

//////////////////////////////////////////////////////////////////////
// ��λ����ʼ������
//
// ����ֵ��bool �ͣ���ʼ���Ƿ��ɹ�
//////////////////////////////////////////////////////////////////////
template<typename Type>bool Matrix<Type>::IdentityInitialize()
{
	this->ZeroInitialize();
	int temp = MAX(m_nNumRows, m_nNumColumns);

	for (int i = 0; i < temp; i++)
	{
		SetElement(i, i, 1);
	}
	return true;
}

//////////////////////////////////////////////////////////////////////
// ��ʼ������
// ������
// 1. int nRows - ָ���ľ�������
// 2. int nCols - ָ���ľ�������
// ����ֵ��bool �ͣ���ʼ���Ƿ��ɹ�
//////////////////////////////////////////////////////////////////////
template<typename Type>bool Matrix<Type>::Init(int nRows, int nCols)
{
	this->isValid = false;
	if (m_pData)
	{
		delete[] m_pData;
		m_pData = NULL;
	}

	m_nNumRows = nRows;
	m_nNumColumns = nCols;
	int nSize = nCols*nRows;

	if (nSize < 0)
	{
		this->isValid = false;
		std::cout<<("Rows and Cols must be positive!\n");
		return false;
	}

	// �����ڴ�
	m_pData = new Type[nSize];
	if (m_pData == NULL)
	{
		this->isValid = false;
		std::cout<<("Rows and Cols must be positive!\n");
		return false;
	}

	// �ڴ�����ʧ��
	// if (IsBadReadPtr(m_pData, sizeof(Type) * nSize))
	// {
	// 	this->isValid = false;
	// 	std::cout<<("Rows and Cols must be positive!\n");
	// 	return false;
	// }

	// ����Ԫ��ֵ��0
	memset(m_pData, 0, sizeof(Type) * nSize);

	isValid = true;

	return true;
}

//////////////////////////////////////////////////////////////////////
// ������ֵ����
//
// ����ֵ��bool �ͣ���ʼ���Ƿ��ɹ�
//////////////////////////////////////////////////////////////////////
template<typename Type>bool  Matrix<Type>::SetData(Type value[])
{
	// empty the memory
	memset(m_pData, 0, sizeof(Type) * m_nNumColumns*m_nNumRows);
	// copy data
	memcpy(m_pData, value, sizeof(Type)*m_nNumColumns*m_nNumRows);
	return true;
}

//////////////////////////////////////////////////////////////////////
// ����ָ��Ԫ�ص�ֵ
//
// ������
// 1. int nRows - ָ���ľ�������
// 2. int nCols - ָ���ľ�������
// 3. Type value - ָ��Ԫ�ص�ֵ
//
// ����ֵ��bool �ͣ�˵�������Ƿ��ɹ�
//////////////////////////////////////////////////////////////////////
template<typename Type>bool Matrix<Type>::
	SetElement(int nRow, int nCol, Type value)
{
	if (nCol<0 || nCol>=m_nNumColumns || nRow<0 || nRow>=m_nNumRows)
	{
		std::cout<<("nRow or nCos is out of range!\n");
		return false;// array bounds error
	}

	if (m_pData == NULL)
	{
		std::cout<<("the matrix is NOT valid!\n");
		return false;// bad pointer error
	}

	m_pData[nCol + nRow * m_nNumColumns] = value;

	return true;
}

//////////////////////////////////////////////////////////////////////
// ��ӡ������ֵ
//////////////////////////////////////////////////////////////////////
template<typename Type>void Matrix<Type>::printf() const
{
	std::cout<<("\n");
	for (int i = 0; i < m_nNumRows; i++)
	{
		for (int j = 0; j < m_nNumColumns; j++)
		{
			std::cout<<("%10.4f ", GetElement(i, j));
		}
		std::cout<<("\n");
	}
	std::cout<<("\n");
}

//////////////////////////////////////////////////////////////////////
// ��ȡ����������
//
// ����ֵ��Type��ָ�룬ָ��������Ԫ�ص����ݻ�����
//////////////////////////////////////////////////////////////////////
template<typename Type>Type* Matrix<Type>::GetData() const
{
	return m_pData;
}

//////////////////////////////////////////////////////////////////////
// ��ȡ����������
//
// ����ֵ��int �ͣ�����������
//////////////////////////////////////////////////////////////////////
template<typename Type>int	Matrix<Type>::GetNumRows() const
{
	return m_nNumRows;
}

//////////////////////////////////////////////////////////////////////
// ��ȡ����������
//
// ����ֵ��int �ͣ�����������
//////////////////////////////////////////////////////////////////////
template<typename Type>int	Matrix<Type>::GetNumColumns() const
{
	return m_nNumColumns;
}

//////////////////////////////////////////////////////////////////////
// ����ָ��Ԫ�ص�ֵ
//
// ������
// 1. int nRows - ָ���ľ�������
// 2. int nCols - ָ���ľ�������
//
// ����ֵ��Type �ͣ�ָ��Ԫ�ص�ֵ
//////////////////////////////////////////////////////////////////////
template<typename Type>Type Matrix<Type>::
	GetElement(int nRow, int nCol) const
{	 
	if ((m_pData) &&   (nCol >= 0 && nCol < m_nNumColumns 
			&& nRow >= 0 && nRow < m_nNumRows))
	{
	      return m_pData[nCol + nRow * m_nNumColumns];
	}
	else
	{
		std::cout<<("the element does not exist!\n");
		return false;
	}
}

//////////////////////////////////////////////////////////////////////
// ��ȡָ���е�����
//
// ������
// 1. int nRows - ָ���ľ�������
// 2.  Type* pVector - ָ�������и�Ԫ�صĻ�����
//
// ����ֵ��int �ͣ�������Ԫ�صĸ�����������������
//////////////////////////////////////////////////////////////////////
template<typename Type>int Matrix<Type>::
	GetRowVector(int nRow, Type*  pVector) const
{
	for (int j = 0; j<m_nNumColumns; ++j)
	{
		pVector[j] = GetElement(nRow, j);
	}

	return m_nNumColumns;
}

//////////////////////////////////////////////////////////////////////
// ��ȡָ���е�����
//
// ������
// 1. int nCols - ָ���ľ�������
// 2.  Type=* pVector - ָ�������и�Ԫ�صĻ�����
//
// ����ֵ��int �ͣ�������Ԫ�صĸ�����������������
//////////////////////////////////////////////////////////////////////
template<typename Type>int Matrix<Type>::
	GetColVector(int nCol, Type* pVector) const
{
	for (int i = 0; i<m_nNumRows; ++i)
	{
		pVector[i] = GetElement(i, nCol);
	}

	return m_nNumRows;
}

//////////////////////////////////////////////////////////////////////
// ������������
//
// ����ֵ����������
//////////////////////////////////////////////////////////////////////
template<typename Type>int Matrix<Type>::GetRank()const
{
	int err = 0;
	int rank = 0;
	int m = this->m_nNumRows;
	int n = this->m_nNumColumns;
	double* A = NULL;
	double* U = NULL;
	double* D = NULL;
	double* V = NULL;
	double* dummy_array = NULL;

	if (m>=n)
	{
		A = new double[m*n];
		U = new double[m*n];
		D = new double[n];
		V = new double[n*n];
		dummy_array = new double[n]; 

		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				A[i*n + j] = GetElement(i,j);
			}
		}

		err = Singular_Value_Decomposition(A, m, n, U, D, V, dummy_array);//�����ֽ�
	}
	else
	{
		A = new double[m*n];
		U = new double[m*n];
		D = new double[m];
		V = new double[m*m];
		dummy_array = new double[m]; 

		for (int j = 0; j < n; j++)
		{
			for (int i = 0; i < m; i++)
			{
				A[j*m + i] = GetElement(i,j);//���ſɱȾ���ת��
			}
		}

		err = Singular_Value_Decomposition(A, n, m, U, D, V, dummy_array);//�����ֽ�
	}

	
	for (int i=0; i<m; i++)
	{
		if (D[i] > MIN_DOUBLE)
		{
			rank = rank + 1;
		}
	}


	//�ͷſռ�

	delete[] A;
	delete[] U;
	delete[] D;
	delete[] V;
	delete[] dummy_array;

	return rank;
}



//////////////////////////////////////////////////////////////////////
// �������ȷ�������
//
// ����ֵ��������
//////////////////////////////////////////////////////////////////////
template<typename Type>  Matrix<Type> Matrix<Type>::inv()const
{

	if (m_nNumColumns != m_nNumRows)
	{
		std::cout<<("For inv, the Rows and Cols must be equal!\n");
		Matrix<Type>	errorresult;
		errorresult.isValid = false;
		errorresult.SetElement(0, 0, 9999);
		return errorresult;
	}

	Matrix<Type> ret(m_nNumRows, m_nNumColumns);

	int i = 0;
	int j = 0;
	int k = 0;
	double dTemp = 0;
	double dMulti = 0;

	// 1. ������������mAug = (A | E) ����ʼ��
	Matrix<Type> mAug(m_nNumRows, 2 * m_nNumColumns); 

	for (i = 0; i < mAug.m_nNumRows; i++)
	{
		for (j = 0; j < mAug.m_nNumColumns; j++)
		{
			if (j <= m_nNumColumns - 1)
			{
				mAug.SetElement(i, j, GetElement(i, j));
			}
			else
			{
				if (i == j - m_nNumColumns)
				{
					mAug.SetElement(i, j, 1);
				}
				else
				{
					mAug.SetElement(i, j, 0);
				}
			}
		}
	}

	// 2. ��˹��Ԫ���ȱ任������
	for (k = 0; k < mAug.m_nNumRows - 1; k++)		
	{
		// 2.1 ��������Ϊk,k����Ϊ0,���б任
		if (fabs(mAug.GetElement(k, k)) < ACCURACY_FACTOR)
		{
			for (i = k + 1; i < mAug.m_nNumRows; i++)
			{
				if (fabs(mAug.GetElement(i, k)) > ACCURACY_FACTOR)
				{
					break;
				}
			}
			if (i >= mAug.m_nNumRows)	//�������Ⱦ��󣬷���-1
			{
				ret.isValid = false;
			}
			else
			{
				mAug.ExchangeRow(i, k);
			}
		}

		// 2.2 ��Ԫ
		for (i = k + 1; i < mAug.m_nNumRows; i++)
		{
			//���ñ���
			dMulti = mAug.GetElement(i, k) / mAug.GetElement(k, k);
			if (fabs(dMulti) > ACCURACY_FACTOR)
				mAug.PrimaryShiftRow(i, dMulti, k);	//�б任
		}
	}

	// 3. �任������
	for (k = mAug.m_nNumRows - 1; k > 0; k--)
	{
		if (mAug.GetElement(k, k) == 0)
		{
			ret.isValid = false;
		}
		// ��Ԫ
		for (i = k - 1; i >= 0; i--)
		{
			dMulti = mAug.GetElement(i, k) / mAug.GetElement(k, k);
			if (fabs(dMulti) > ACCURACY_FACTOR)
				mAug.PrimaryShiftRow(i, dMulti, k);	//�б任
		}
	}

	// 4. �����߷�����Ϊ��λ����
	for (i = 0; i < mAug.m_nNumRows; i++)
	{
		if (mAug.GetElement(i, i) != 1)
		{
			dMulti = 1 / mAug.GetElement(i, i);
			mAug.PrimaryShiftRow(i, dMulti);		 //�б任
		}
	}

	// 5. ����������
	ret.isValid = true;
	for (i = 0; i < m_nNumRows; i++)
	{
		for (j = 0; j < m_nNumRows; j++)
		{
			ret.SetElement(i,j,mAug.GetElement(i,j+mAug.m_nNumRows));
		}
	}
	return 	ret;
}


//////////////////////////////////////////////////////////////////////
// ��������������
//
// ������
// 1. int nRow1 - ָ���ľ�����
// 2. int nRos2 - ָ���ľ�����
//////////////////////////////////////////////////////////////////////
template<typename Type> bool Matrix<Type>::
	ExchangeRow(int nRow1, int nRow2)
{
	if (nRow1 > m_nNumRows || nRow2 > m_nNumRows)
	{
		std::cout<<("The rows are out of range!\n");
		return false;
	}

	if (nRow1 == nRow2)
	{
		return true;
	}

	double dTemp;
	for (int j = 0; j < m_nNumColumns; j++)
	{
		dTemp = GetElement(nRow1, j);
		SetElement(nRow1, j, GetElement(nRow2, j));
		SetElement(nRow2, j, dTemp);
	}
	return true;
}

//////////////////////////////////////////////////////////////////////
// �Ծ�����ָ���г���һ��ϵ��
//
// ������
// 1. int nRow1 - ָ���ľ�����
// 2. Type dMultipe - �˵�ϵ��
//////////////////////////////////////////////////////////////////////
template<typename Type> bool Matrix<Type>::
	PrimaryShiftRow(int nRow, Type dMultiple)
{
	if (nRow >= m_nNumRows)
	{
		std::cout<<("The rows are out of range!\n");
		return false;
	}

	for (int i = 0; i < m_nNumColumns; i++)
	{
		SetElement(nRow, i, GetElement(nRow, i) * dMultiple);
	}

	return true;
}

//////////////////////////////////////////////////////////////////////
// �Ծ�����ָ���� ��ȥ ��һ�г���һ��ϵ��
//
// ������
// 1. int nRow1 - ָ���ľ�����
// 2. Type dMultipe - �˵�ϵ��
// 3. int nRos2 - ���˵ľ�����
//////////////////////////////////////////////////////////////////////
template<typename Type> bool Matrix<Type>::
	PrimaryShiftRow(int nRow1, Type dMultiple, int nRow2)
{
	if (nRow1 >= m_nNumRows || nRow2 >= m_nNumRows)
		return false;
	double dValue;
	for (int i = 0; i < m_nNumColumns; i++)
	{
		dValue = GetElement(nRow1, i);
		dValue -= GetElement(nRow2, i)* dMultiple;
		SetElement(nRow1, i, dValue);
	}

	return true;
}

//////////////////////////////////////////////////////////////////////
// ��������������
//
// ������
// 1. int nCol1 - ָ���ľ�����
// 2. int nCol2 - ָ���ľ�����
//////////////////////////////////////////////////////////////////////
template<typename Type> bool Matrix<Type>::
	ExchangeCol(int nCol1, int nCol2)
{
	if (nCol1 > m_nNumColumns || nCol2 > m_nNumColumns)
		return false;
	if (nCol1 == nCol2)
		return true;

	double dTemp;
	for (int j = 0; j < m_nNumRows; j++)
	{
		dTemp = GetElement(j, nCol1);
		SetElement(j, nCol1, GetElement(j, nCol2));
		SetElement(j, nCol2, dTemp);
	}
	return true;
}

//////////////////////////////////////////////////////////////////////
// �Ծ�����ָ���г���һ��ϵ��
//
// ������
// 1. int nCol - ָ���ľ�����
// 2. Type dMultipe - �˵�ϵ��
//////////////////////////////////////////////////////////////////////
template<typename Type> bool Matrix<Type>::
	PrimaryShiftCol(int nCol, Type dMultiple)
{
	if (nCol >= m_nNumColumns)
		return false;

	for (int i = 0; i < m_nNumRows; i++)
	{
		SetElement(i, nCol, GetElement(i, nCol) * dMultiple);
	}
	return true;
}

//////////////////////////////////////////////////////////////////////
// �Ծ�����ָ���� ��ȥ ��һ�г���һ��ϵ��
//
// ������
// 1. int nCol1 - ָ���ľ�����
// 2. Type dMultipe - �˵�ϵ��
// 3. int nCol2 - ���˵ľ�����
//////////////////////////////////////////////////////////////////////
template<typename Type> bool Matrix<Type>::
	PrimaryShiftCol(int nCol1, Type dMultiple, int nCol2)
{
	if (nCol1 >= m_nNumColumns || nCol2 >= m_nNumColumns)
		return false;
	double dValue;
	for (int i = 0; i < m_nNumRows; i++)
	{
		dValue = GetElement(i, nCol1);
		dValue -= GetElement(i, nCol2)* dMultiple;
		SetElement(i, nCol1, dValue);
	}

	return true;
}


//////////////////////////////////////////////////////////////////////
// ����������=����������ֵ
//
// ������
// 1. const Matrix& other - ���ڸ�������ֵ��Դ����
//
// ����ֵ��Matrix
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type> 
	Matrix<Type>::operator=(const Matrix<Type>& other)
{
	if (&other != this)
	{
		// copy the pointer
		if (Init(other.GetNumRows(), other.GetNumColumns()))
		{
			memcpy(m_pData, other.m_pData, 
					sizeof(Type)*m_nNumColumns*m_nNumRows);
		}
		else
		{
			std::cout<<("operator '=' failed!\n");
			this->isValid = false;
		}
	}

	// finally return a reference to ourselves
	return *this;
}

//////////////////////////////////////////////////////////////////////
// ����������==���жϾ����Ƿ�����
//
// ������
// 1. const Matrix<Type>& other - ���ڱȽϵľ���
//
// ����ֵ��bool �ͣ���������������ΪTRUE������ΪFALSE
//////////////////////////////////////////////////////////////////////
template<typename Type>bool Matrix<Type>::
	operator==(const Matrix<Type>& other) const
{
	// ���ȼ����������Ƿ�����
	if (m_nNumColumns != other.GetNumColumns() 
		|| m_nNumRows != other.GetNumRows())
	{
		return false;
	}

	for (int i = 0; i<m_nNumRows; ++i)
	{
		for (int j = 0; j<m_nNumColumns; ++j)
		{
			if (ABS(GetElement(i, j) - other.GetElement(i, j)) 
					>ACCURACY_FACTOR)
			{
				return false;
			}
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////
// ����������+��ʵ�־����ļӷ�
//
// ������
// 1. const Matrix<Type>& other - ��ָ���������ӵľ���
//
// ����ֵ��Matrix<Type>�ͣ�ָ��������other����֮��
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type> Matrix<Type>::
	operator+(const Matrix<Type>& other) const
{
	// ���ȼ����������Ƿ�����
	if (m_nNumColumns == other.GetNumColumns() 
		&& m_nNumRows == other.GetNumRows())
	{
		// ������������
		Matrix<Type>	result(*this);		// ��������
		// �����ӷ�
		for (int i = 0; i < m_nNumRows; ++i)
		{
			for (int j = 0; j < m_nNumColumns; ++j)
				result.SetElement(i, j, result.GetElement(i, j) +
									other.GetElement(i, j));
		}
		return result;
	}
	else
	{
		Matrix<Type>	errorresult;
		errorresult.SetElement(0,0,9999);
		return errorresult;
	}
}

//////////////////////////////////////////////////////////////////////
// ����������-��ʵ�־����ļ���
// ������
// 1. const Matrix<Type>& other - ��ָ�����������ľ���
//
// ����ֵ��Matrix<Type>�ͣ�ָ��������other����֮��
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type> Matrix<Type>::
	operator-(const Matrix<Type>& other) const
{
	// ���ȼ����������Ƿ�����
	if (m_nNumColumns == other.GetNumColumns() 
			&& m_nNumRows == other.GetNumRows())
	{
		// ����Ŀ������
		Matrix<Type>	result(*this);
		// ���м�������
		for (int i = 0; i < m_nNumRows; ++i)
		{
			for (int j = 0; j < m_nNumColumns; ++j)
				result.SetElement(i, j, result.GetElement(i, j)
							- other.GetElement(i, j));
		}

		return result;
	}
	else
	{
		Matrix<Type>	errorresult;
		errorresult.isValid = false;
		errorresult.SetElement(0, 0, 9999);
		return errorresult;
	}
}

//////////////////////////////////////////////////////////////////////
// ����������*��ʵ�־����ĳ˷�
//
// ������
// 1. const Matrix<Type>& other - ��ָ���������˵ľ���
//
// ����ֵ��Matrix<Type>�ͣ�ָ��������other����֮��
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type> Matrix<Type>::
	operator*(const Matrix<Type>& other) const
{
	// ���ȼ����������Ƿ�����Ҫ��
	if (m_nNumColumns == other.GetNumRows())
	{
		// construct the object we are going to return
		Matrix<Type>	result(m_nNumRows, other.GetNumColumns());

		// �����˷�����
		//
		// [A][B][C]   [G][H]     [A*G + B*I + C*K][A*H + B*J + C*L]
		// [D][E][F] * [I][J] =   [D*G + E*I + F*K][D*H + E*J + F*L]
		//             [K][L]
		//
		Type	value;
		for (int i = 0; i < result.GetNumRows(); ++i)
		{
			for (int j = 0; j < other.GetNumColumns(); ++j)
			{
				value = 0.0;
				for (int k = 0; k < m_nNumColumns; ++k)
				{
					value += GetElement(i, k) * other.GetElement(k, j);
				}

				result.SetElement(i, j, value);
			}
		}

		return result;
	}
	else
	{
		std::cout<<("operator '*' require M1.Cols equals to M2.Rows!\n");
		Matrix<Type>	errorresult;
		errorresult.isValid = false;
		errorresult.SetElement(0, 0, 9999);
		return errorresult;
	}
}


//////////////////////////////////////////////////////////////////////
// ����������*������ÿ��Ԫ�س���ratio
//
// ������
// 1. const Type
//
// ����ֵ��Matrix�ͣ�ָ��������other����֮��
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type> Matrix<Type>::
	operator*(const Type& ratio)
{
	for (int i = 0; i < m_nNumRows; i++)
	{
		for (int j = 0; j < m_nNumColumns; j++)
		{
			SetElement(i, j, ratio*GetElement(i, j));
		}
	}
	return *this;
}


//////////////////////////////////////////////////////////////////////
// �ϲ��������󣬰��еķ�ʽ���磺 3*1��3*1���ϲ�Ϊ6*1,������������
//
// ������
// 1. Matrix1, ����1
// 2. Matrix2, ����2
//
// ����ֵ��Matrix�ͣ����кϲ����ľ���
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>	Matrix<Type>::CombineMatrixRows(
	Matrix<Type>Matrix1, Matrix<Type> Matrix2)
{
	if (1 == Matrix1.GetNumRows() && 1 == Matrix1.GetNumColumns())
	{
		return Matrix2;
	}
	int columnsnum = Matrix1.GetNumColumns()> Matrix2.GetNumColumns()
				?Matrix1.GetNumColumns() :  Matrix2.GetNumColumns();
	int rowsnum	= Matrix1.GetNumRows() +  Matrix2.GetNumRows();

	Matrix<Type> ret(rowsnum, columnsnum);
	ret.ZeroInitialize();
	for (int i = 0; i<Matrix1.GetNumRows(); i++)
	{
		for (int j = 0; j<Matrix1.GetNumColumns(); j++)
		{
			ret.SetElement(i, j,    Matrix1.GetElement(i, j)   );
		}
	}
	for (int i = 0; i<Matrix2.GetNumRows(); i++)
	{
		for (int j = 0; j<Matrix2.GetNumColumns(); j++)
		{
			ret.SetElement(i + Matrix1.GetNumRows(), j,
							Matrix2.GetElement(i, j)  );
		}
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////
// �ϲ��������󣬰���,Matrix1Ϊ���ۼӵ�Ŀ��������Matrix2Ϊ��������
//
// ������
// 1. Matrix1, ����1
// 2. Matrix2, ����2
//
// ����ֵ��Matrix�ͣ����кϲ����ľ���
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>	Matrix<Type>::CombineMatrixColumns(
	 Matrix<Type> Matrix1, Matrix<Type> Matrix2)
{
	if (1 == Matrix1.GetNumRows() && 1 == Matrix1.GetNumColumns())
	{
		return Matrix2;
	}
	int rowsnum = Matrix1.GetNumRows()  > Matrix2.GetNumRows() 
						? Matrix1.GetNumRows() : Matrix2.GetNumRows();
	int columnsnum = Matrix1.GetNumColumns() + Matrix2.GetNumColumns();

	Matrix<Type> ret(rowsnum, columnsnum);
	ret.ZeroInitialize();
	for (int i = 0; i<Matrix1.GetNumRows(); i++)
	{
		for (int j = 0; j<Matrix1.GetNumColumns(); j++)
		{
			ret.SetElement(i, j, Matrix1.GetElement(i, j) );
		}
	}
	for (int i = 0; i<Matrix2.GetNumRows(); i++)
	{
		for (int j = 0; j<Matrix2.GetNumColumns(); j++)
		{
			ret.SetElement(i , j+ Matrix1.GetNumColumns(), 
							Matrix2.GetElement(i, j)     );
		}
	}
	return ret;
} 

//////////////////////////////////////////////////////////////////////
//��ȡ�Ӿ���,���Ŵ�0��ʼ,�ӵ�irowstart�е���irowend�У�
//						�ӵ�icolumstart�е���icolumend��
// ������
// 1. int rowStart,		// ��ʼ�к�
// 2. int rosEnd,		// ��ֹ�к�
// 3. int colStart,		// ��ʼ�к�
// 4. int cosEnd,		// ��ֹ�к�
//
// ����ֵ��Matrix�ͣ�ָ�����е��Ӿ���
//////////////////////////////////////////////////////////////////////
template<typename Type>Matrix<Type>  Matrix<Type>::Getsubmatrix(
	int irowstart, int irowend, int icolumstart, int icolumend)
{
	//�����ж���
	int rownum= irowend - irowstart +1;
	int columnum = icolumend - icolumstart+1;
	Matrix<Type> retMatrix(rownum, columnum);
	int srcrowno = 0;
	for (int i = irowstart; i<=irowend; i++)
	{
		int srccolumno = 0;
		for (int j = icolumstart   ; j <= icolumend; j++ )
		{
			Type sss=GetElement(i  , j  );
			retMatrix.SetElement(srcrowno, srccolumno, sss);
			srccolumno++;
		}
		srcrowno++;
	}
	return retMatrix;
}

//////////////////////////////////////////////////////////////////////
//�����Ӿ���,���Ŵ�0��ʼ,�ӵ�irowstart�е���irowend�У�
//						�ӵ�icolumstart�е���icolumend��
// ������
// 1. int rowStart,		// ��ʼ�к�
// 2. int rosEnd,		// ��ֹ�к�
// 3. int colStart,		// ��ʼ�к�
// 4. int cosEnd,		// ��ֹ�к�
//
// ����ֵ��Matrix�ͣ������Ӿ��������¾���
//////////////////////////////////////////////////////////////////////
template<typename Type>bool  Matrix<Type>::Setsubmatrix(int irowstart,
	 int irowend, int icolumstart, int icolumend,Matrix<Type> subMatrix)
{
	//�����ж���
	int rownum= irowend - irowstart +1;
	int columnum = icolumend - icolumstart+1;

	for (int i = irowstart; i<=irowend; i++)
	{
		for (int j = icolumstart   ; j <= icolumend; j++ )
		{
			this->SetElement(i,j,subMatrix.GetElement(i,j));
		}
	}
	return true;
}












#pragma region pinv_method_1
//////////////////////////////////////////////////////////////////////
// ���㲻���Ⱦ�����α��
//
// ����ֵ��α������
//////////////////////////////////////////////////////////////////////
template<typename Type>	Matrix<Type> Matrix<Type>::pinv()const
{
	Matrix<Type> ret(m_nNumColumns, m_nNumRows);

	ret.ZeroInitialize();

	int err = 0;
	int method = 2;

	if (method == 1)
	{
		Type* pa = ret.GetData();
		Type* a = this->GetData();

		err = _pinv(pa,a,m_nNumRows,m_nNumColumns);
	}

	if (method == 2)
	{ 
		double** J,**Jinv;
		J = (double**)malloc(sizeof(double*)*m_nNumRows);
		for (int i = 0; i<m_nNumRows; i++)
		{
			J[i] = (double*)malloc(sizeof(double)*m_nNumColumns);
		}

		Jinv = (double**)malloc(sizeof(double*)*m_nNumColumns);
		for (int i=0; i<m_nNumColumns; i++)
		{
			Jinv[i] = (double*)malloc(sizeof(double)*m_nNumRows);
		}

		for (int i=0; i<m_nNumRows; i++)
		{
			for (int j=0; j<m_nNumColumns; j++)
			{
				J[i][j] = this->GetElement(i,j);
			}
		}

		err = Jpinv(J,m_nNumRows,m_nNumColumns,Jinv);

		for (int i=0;i<m_nNumColumns; i++)
		{
			for (int j=0; j<m_nNumRows; j++)
			{
				ret.SetElement(i,j,Jinv[i][j]);
			}
		}
	}


	if (err <= 0)
	{
		ret.isValid = false;
	}

	return ret;
}

template<typename Type>int	Matrix<Type>::_pinv(Type* pinv_a,Type* a,int m, int n) const
{
	int i,j,err;

	double* u = new double[m*m];
	double* v = new double[n*n];
	double* c = new double[m*n];
	double* d = new double[m*n];

	for(i=0;i<m*m;i++)
		u[i]=0;

	err = dluav(a,m,n,u,v,ACCURACY_FACTOR,m+1);

	

	// ת��
	double* pinvW = new double[n*m];
	double* vT = new double[n*n];
	double* uT = new double[m*m];

	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			if (fabs(a[j*n+i])>MIN_DOUBLE)
			{
				pinvW[i*m + j] = 1.0 / a[j*n + i];
			}
			else
			{
				pinvW[i*m + j] = 0;
			}
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			vT[i*n + j] = v[j*n + i];
		}
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			uT[i*m + j] = u[j*m + i];
		}
	}

	damul(vT,pinvW,n,n,m,c);
	damul(c,uT,n,m,m,pinv_a);

	delete[] u;
	delete[] v;
	delete[] c;
	delete[] d;
	delete[] pinvW;
	delete[] vT;
	delete[] uT;

	return err;
}

template<typename Type>void Matrix<Type>::sss(double fg[2],double cs[2])const
{
	double r,d;
	//if((fabs(fg[0])+fabs(fg[1]))==0.0)
	if((fabs(fg[0])+fabs(fg[1]))<MIN_DOUBLE)
	{
		cs[0]=1.0;cs[1]=0.0;d=0.0;
	}
	else
	{
		d=sqrt(fg[0]*fg[0]+fg[1]*fg[1]);
		if(fabs(fg[0])>fabs(fg[1]))
		{
			d=fabs(d);
			if(fg[0]<0.0)
				d=-d;
		}
		if(fabs(fg[1])>=fabs(fg[0]))
		{
			d=fabs(d);
			if(fg[1]<0.0)
				d=-d;
		}
		cs[0]=fg[0]/d;
		cs[1]=fg[1]/d;
	}
	r=1.0;
	if(fabs(fg[0])>fabs(fg[1]))
		r=cs[1];
	else
		//if(cs[0]!=0.0)
		if(fabs(cs[0])>MIN_DOUBLE)
			r=1.0/cs[0];
	fg[0]=d;
	fg[1]=r;
	return;
}

template<typename Type>void Matrix<Type>::damul(double a[],double b[],int m,int n,int k,double c[])const
{
	int i,j,l,u;
	for(i=0;i<=m-1;i++)
		for(j=0;j<=k-1;j++)
		{
			u=i*k+j;
			c[u]=0;
			for(l=0;l<=n-1;l++)
				c[u]=c[u]+a[i*n+l]*b[l*k+j];
		}
		return;
}

template<typename Type>void Matrix<Type>::ppp(double a[],double e[],double s[],double v[],int m,int n)const
{
	int i,j,p,q;
	double d;
	if(m>=n)
		i=n;
	else
		i=m;
	for(j=1;j<=i-1;j++)
	{
		a[(j-1)*n+j-1]=s[j-1];
		a[(j-1)*n+j]=e[j-1];
	}
	a[(i-1)*n+i-1]=s[i-1];
	if(m<n)
		a[(i-1)*n+i]=e[i-1];
	for(i=1;i<=n-1;i++)
		for(j=i+1;j<=n;j++)
		{
			p=(i-1)*n+j-1;
			q=(j-1)*n+i-1;
			d=v[p];v[p]=v[q];v[q]=d;
		}
		return;
}

template<typename Type>double Matrix<Type>::norm(double a[],int m,int n,double u[],double v[],double eps,int ka)const
{
	dluav(a,m,n,u,v,eps,ka);
	return a[0];
}

template<typename Type>int Matrix<Type>::dluav(double a[],int m,int n,double u[],double v[],double eps,int ka)const
{
/*********************************************************************
 * ����������ֵ�ֽ⣬�μ���c �����㷨���򼯡�������P169
 * ����˵����
 * a m*n��ʵ���󣬷���ʱ���Խ��߸�������ֵ���ǵ���˳�򣩣�����Ԫ��Ϊ0
 * m,n ����A������������
 * u m*m�ľ��󣬴�������������
 * v n*n�ľ��󣬴�������������
 * eps ˫����ʵ�ͱ�������������Ҫ��
 * ka ���α�������ֵΪmax(m,n)+1
 * ����ֵ���������ر�־С��0����˵�������˵���MAX_ITERA�λ�δ����ĳ��
 * ����ֵ����������ʱ����A�ķֽ�ʽΪUAV���������ر�־����0����˵��
 * ������������
 ********************************************************************/


	int i,j,k,l,it,ll,kk,ix,iy,mm,nn,iz,ml,ks;
	double d,dd,t,sm,sml,eml,sk,ek,b,c,shh,fg[2],cs[2];
	double *s,*e,*w;
	s = new double[ka];
	e = new double[ka];
	w = new double[ka];
	
	for(i=1;i<=m;i++)
	{
		ix=(i-1)*m+i-1;
		u[ix]=0;
	}
	for(i=1;i<=n;i++)
	{
		iy=(i-1)*n+i-1;
		v[iy]=0;
	}
	it=MAX_ITERA;k=n;
	if(m-1<n)
		k=m-1;
	l=m;
	if(n-2<m) l=n-2;
	if(l<0) l=0;
	ll=k;
	if(l>k) ll=l;
	if(ll>=1)
	{
		for(kk=1;kk<=ll;kk++)
		{
			if(kk<=k)
			{
				d=0.0;
				for(i=kk;i<=m;i++)
				{
					ix=(i-1)*n+kk-1;d=d+a[ix]*a[ix];
				}
				s[kk-1]=sqrt(d);
				//if(s[kk-1]!=0.0)
				if(fabs(s[kk-1])>MIN_DOUBLE)
				{
					ix=(kk-1)*n+kk-1;
					//if(a[ix]!=0.0)
					if(fabs(a[ix])>MIN_DOUBLE)
					{
						s[kk-1]=fabs(s[kk-1]);
						if(a[ix]<0.0) s[kk-1]=-s[kk-1];
					}
					for(i=kk;i<=m;i++)
					{
						iy=(i-1)*n+kk-1;
						a[iy]=a[iy]/s[kk-1];
					}
					a[ix]=1.0+a[ix];
				}
				s[kk-1]=-s[kk-1];
			}
			if(n>=kk+1)
			{
				for(j=kk+1;j<=n;j++)
				{
					//if((kk<=k)&&(s[kk-1]!=0.0))
					if((kk<=k)&&(fabs(s[kk-1])>MIN_DOUBLE))
					{
						d=0.0;
						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*n+kk-1;
							iy=(i-1)*n+j-1;
							d=d+a[ix]*a[iy];
						}
						d=-d/a[(kk-1)*n+kk-1];
						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*n+j-1;
							iy=(i-1)*n+kk-1;
							a[ix]=a[ix]+d*a[iy];
						}
					}
					e[j-1]=a[(kk-1)*n+j-1];
				}
			}
			if(kk<=k)
			{
				for(i=kk;i<=m;i++)
				{
					ix=(i-1)*m+kk-1;iy=(i-1)*n+kk-1;
					u[ix]=a[iy];
				}
			}
			if(kk<=l)
			{
				d=0.0;
				for(i=kk+1;i<=n;i++)
					d=d+e[i-1]*e[i-1];
				e[kk-1]=sqrt(d);
				//if(e[kk-1]!=0.0)
				if(fabs(e[kk-1])>MIN_DOUBLE)
				{
					//if(e[kk]!=0.0)
					if(fabs(e[kk])>MIN_DOUBLE)
					{
						e[kk-1]=fabs(e[kk-1]);
						if(e[kk]<0.0)
							e[kk-1]=-e[kk-1];
					}
					for(i=kk+1;i<=n;i++)
						e[i-1]=e[i-1]/e[kk-1];
					e[kk]=1.0+e[kk];
				}
				e[kk-1]=-e[kk-1];
				//if((kk+1<=m)&&(e[kk-1]!=0.0))
				if((kk+1<=m)&&(fabs(e[kk-1])>MIN_DOUBLE))
				{
					for(i=kk+1;i<=m;i++) w[i-1]=0.0;
					for(j=kk+1;j<=n;j++)
						for(i=kk+1;i<=m;i++)
							w[i-1]=w[i-1]+e[j-1]*a[(i-1)*n+j-1];
					for(j=kk+1;j<=n;j++)
						for(i=kk+1;i<=m;i++)
						{
							ix=(i-1)*n+j-1;
							a[ix]=a[ix]-w[i-1]*e[j-1]/e[kk];
						}
				}
				for(i=kk+1;i<=n;i++)
					v[(i-1)*n+kk-1]=e[i-1];
			}
		}
	}
	mm=n;
	if(m+1<n) mm=m+1;
	if(k<n) s[k]=a[k*n+k];
	if(m<mm) s[mm-1]=0.0;
	if(l+1<mm) e[l]=a[l*n+mm-1];
	e[mm-1]=0.0;
	nn=m;
	if(m>n) nn=n;
	if(nn>=k+1)
	{
		for(j=k+1;j<=nn;j++)
		{
			for(i=1;i<=m;i++)
				u[(i-1)*m+j-1]=0.0;
			u[(j-1)*m+j-1]=1.0;
		}
	}
	if(k>=1)/////////////////////////////////
	{
		for(ll=1;ll<=k;ll++)
		{
			kk=k-ll+1;iz=(kk-1)*m+kk-1;
			//if(s[kk-1]!=0.0)
			if(fabs(s[kk-1])>MIN_DOUBLE)
			{
				if(nn>=kk+1)
					for(j=kk+1;j<=nn;j++)
					{
						d=0.0;
						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*m+kk-1;
							iy=(i-1)*m+j-1;
							d=d+u[ix]*u[iy]/u[iz];
						}
						d=-d;
						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*m+j-1;
							iy=(i-1)*m+kk-1;
							u[ix]=u[ix]+d*u[iy];
						}
					}
					for(i=kk;i<=m;i++)
					{
						ix=(i-1)*m+kk-1;
						u[ix]=-u[ix];
					}
					u[iz]=1.0+u[iz];
					if(kk-1>=1)//////////////////////////////////////
						for(i=1;i<=kk-1;i++)
							u[(i-1)*m+kk-1]=0.0;
			}
			else
			{
				for(i=1;i<=m;i++)
					u[(i-1)*m+kk-1]=0.0;
				u[(kk-1)*m+kk-1]=1.0;
			}
		}
	}
	for(ll=1;ll<=n;ll++)
	{
		kk=n-ll+1;iz=kk*n+kk-1;
		//if((kk<=l)&&(e[kk-1]!=0.0))/////////////////////////////
		if((kk<=l)&&(fabs(e[kk-1])>MIN_DOUBLE))
		{
			for(j=kk+1;j<=n;j++)
			{
				d=0.0;
				for(i=kk+1;i<=n;i++)
				{
					ix=(i-1)*n+kk-1;iy=(i-1)*n+j-1;
					d=d+v[ix]*v[iy]/v[iz];
				}
				d=-d;
				for(i=kk+1;i<=n;i++)
				{
					ix=(i-1)*n+j-1;iy=(i-1)*n+kk-1;
					v[ix]=v[ix]+d*v[iy];
				}
			}
		}
		for(i=1;i<=n;i++)
			v[(i-1)*n+kk-1]=0.0;
		v[iz-n]=1.0;
	}
	for(i=1;i<=m;i++)
		for(j=1;j<=n;j++)
			a[(i-1)*n+j-1]=0.0;
	ml=mm;
	it=MAX_ITERA;
	while(1==1)//////////////////////////////////
	{
		if(mm==0)
		{
			ppp(a,e,s,v,m,n);
			// free(s);free(e);free(w);
			delete[] s; delete[] e; delete[] w;
			return l;
		}
		if(it==0)
		{
			ppp(a,e,s,v,m,n);
			// free(s);free(e);free(w);
			delete[] s; delete[] e; delete[] w;
			return -1;
		}
		kk=mm-1;
		//while((kk!=0)&&(fabs(e[kk-1])!=0.0))
		while((kk!=0)&&(fabs(e[kk-1])>MIN_DOUBLE))
		{
			d=fabs(s[kk-1])+fabs(s[kk]);
			dd=fabs(e[kk-1]);
			if(dd>eps*d)
				kk=kk-1;
			else
				e[kk-1]=0.0;
		}
		if(kk==mm-1)
		{
			kk=kk+1;
			if(s[kk-1]<0.0)
			{
				s[kk-1]=-s[kk-1];
				for(i=1;i<=n;i++)
				{
					ix=(i-1)*n+kk-1;
					v[ix]=-v[ix];
				}
			}
			while((kk!=ml)&&(s[kk-1]<s[kk]))
			{
				d=s[kk-1];s[kk-1]=s[kk];s[kk]=d;
				if(kk<n)
					for(i=1;i<=n;i++)
					{
						ix=(i-1)*n+kk-1;iy=(i-1)*n+kk;
						d=v[ix];v[ix]=v[iy];v[iy]=d;
					}
					if(kk<m)
						for(i=1;i<=m;i++)
						{
							ix=(i-1)*m+kk-1;
							iy=(i-1)*m+kk;
							d=u[ix];u[ix]=u[iy];u[iy]=d;
						}
						kk=kk+1;
			}
			it=MAX_ITERA;
			mm=mm-1;
		}
		else
		{
			ks=mm;
			//while((ks>kk)&&(fabs(s[ks-1])!=0.0))
			while((ks>kk)&&(fabs(s[ks-1])>MIN_DOUBLE))
			{
				d=0.0;
				if(ks!=mm)
					d=d+fabs(e[ks-1]);
				if(ks!=kk+1) d=d+fabs(e[ks-2]);
				dd=fabs(s[ks-1]);
				if(dd>eps*d)
					ks=ks-1;
				else
					s[ks-1]=0.0;
			}
			if(ks==kk)
			{
				kk=kk+1;
				d=fabs(s[mm-1]);
				t=fabs(s[mm-2]);
				if(t>d)
					d=t;
				t=fabs(e[mm-2]);
				if(t>d)
					d=t;
				t=fabs(s[kk-1]);
				if(t>d)
					d=t;
				t=fabs(e[kk-1]);
				if(t>d)
					d=t;
				sm=s[mm-1]/d;sml=s[mm-2]/d;
				eml=e[mm-2]/d;
				sk=s[kk-1]/d;ek=e[kk-1]/d;
				b=((sml+sm)*(sml-sm)+eml*eml)/2.0;
				c=sm*eml;c=c*c;shh=0.0;
				//if((b!=0.0)||(c!=0.0))
				if((fabs(b)>MIN_DOUBLE)||(fabs(c)>MIN_DOUBLE))
				{
					shh=sqrt(b*b+c);
					if(b<0.0)
						shh=-shh;
					shh=c/(b+shh);
				}
				fg[0]=(sk+sm)*(sk-sm)-shh;
				fg[1]=sk*ek;
				for(i=kk;i<=mm-1;i++)
				{
					sss(fg,cs);
					if(i!=kk)
						e[i-2]=fg[0];
					fg[0]=cs[0]*s[i-1]+cs[1]*e[i-1];
					e[i-1]=cs[0]*e[i-1]-cs[1]*s[i-1];
					fg[1]=cs[1]*s[i];
					s[i]=cs[0]*s[i];
					//if((cs[0]!=1.0)||(cs[1]!=0.0))
					if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))
						for(j=1;j<=n;j++)
						{
							ix=(j-1)*n+i-1;
							iy=(j-1)*n+i;
							d=cs[0]*v[ix]+cs[1]*v[iy];
							v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
							v[ix]=d;
						}
						sss(fg,cs);
						s[i-1]=fg[0];
						fg[0]=cs[0]*e[i-1]+cs[1]*s[i];
						s[i]=-cs[1]*e[i-1]+cs[0]*s[i];
						fg[1]=cs[1]*e[i];
						e[i]=cs[0]*e[i];
						if(i<m)
							//if((cs[0]!=1.0)||(cs[1]!=0.0))
							if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))
								for(j=1;j<=m;j++)
								{
									ix=(j-1)*m+i-1;
									iy=(j-1)*m+i;
									d=cs[0]*u[ix]+cs[1]*u[iy];
									u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
									u[ix]=d;
								}
				}
				e[mm-2]=fg[0];
				it=it-1;
			}
			else
			{
				if(ks==mm)
				{
					kk=kk+1;
					fg[1]=e[mm-2];e[mm-2]=0.0;
					for(ll=kk;ll<=mm-1;ll++)
					{
						i=mm+kk-ll-1;
						fg[0]=s[i-1];
						sss(fg,cs);
						s[i-1]=fg[0];
						if(i!=kk)
						{
							fg[1]=-cs[1]*e[i-2];
							e[i-2]=cs[0]*e[i-2];
						}
						//if((cs[0]!=1.0)||(cs[1]!=0.0))
						if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))
							for(j=1;j<=n;j++)
							{
								ix=(j-1)*n+i-1;
								iy=(j-1)*n+mm-1;
								d=cs[0]*v[ix]+cs[1]*v[iy];
								v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
								v[ix]=d;
							}
					}
				}
				else
				{
					kk=ks+1;
					fg[1]=e[kk-2];
					e[kk-2]=0.0;
					for(i=kk;i<=mm;i++)
					{
						fg[0]=s[i-1];
						sss(fg,cs);
						s[i-1]=fg[0];
						fg[1]=-cs[1]*e[i-1];
						e[i-1]=cs[0]*e[i-1];
						//if((cs[0]!=1.0)||(cs[1]!=0.0))
						if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))
							for(j=1;j<=m;j++)
							{
								ix=(j-1)*m+i-1;
								iy=(j-1)*m+kk-2;
								d=cs[0]*u[ix]+cs[1]*u[iy];								
								u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
								u[ix]=d;
							}
					}
				}
			}
		}
	}
	// free(s);free(e);free(w);
	delete[] s; delete[] e; delete[] w;
	return l;
}

#pragma endregion pinv_method_1







#pragma region pinv2


template<typename Type>int Matrix<Type>::Jpinv(double** J,int m,int n,double **Jinv)const
{
	int err = 0;

	int i = 0;
	int j = 0;
	double tolerance = 0.0000001;
	double* A = NULL;
	double* U = NULL;
	double* D = NULL;
	double* V = NULL;
	double* dummy_array = NULL;
	double* Astar = NULL;

	if (m>=n)
	{
		// A = (double*)malloc(sizeof(double)* m*n);
		// U = (double*)malloc(sizeof(double)* m*n);
		// D = (double*)malloc(sizeof(double)*n);
		// V = (double*)malloc(sizeof(double)* n*n);
		// dummy_array = (double*)malloc(sizeof(double)*n);
		// Astar = (double*)malloc(sizeof(double)* m*n);

		A = new double[m*n];
		U = new double[m*n];
		D = new double[n];
		V = new double[n*n];
		dummy_array = new double[n];
		Astar = new double[m*n];

		for (i = 0; i < m; i++)
		{
			for (j = 0; j < n; j++)
			{
				A[i*n + j] = J[i][j];
			}
		}

		err = Singular_Value_Decomposition(A, m, n, U, D, V, dummy_array);//�����ֽ�

		Singular_Value_Decomposition_Inverse(U, D, V, tolerance, m, n, Astar);//��������

		for (i = 0; i < n; i++)
		{
			for (j = 0; j < m; j++)
			{
				Jinv[i][j] = Astar[i*m + j];
			}
		}
	}
	else
	{
		// A = (double*)malloc(sizeof(double)* m*n);
		// U = (double*)malloc(sizeof(double)* m*n);
		// D = (double*)malloc(sizeof(double)*m);
		// V = (double*)malloc(sizeof(double)* m*m);
		// dummy_array = (double*)malloc(sizeof(double)*m);
		// Astar = (double*)malloc(sizeof(double)* m*n);

		A = new double[m*n];
		U = new double[m*n];
		D = new double[m];
		V = new double[m*m];
		dummy_array = new double[m];
		Astar = new double[m*n];

		for (j = 0; j < n; j++)
		{
			for (i = 0; i < m; i++)
			{
				A[j*m + i] = J[i][j];//���ſɱȾ���ת��
			}
		}

		err = Singular_Value_Decomposition(A, n, m, U, D, V, dummy_array);//�����ֽ�

		Singular_Value_Decomposition_Inverse(U, D, V, tolerance, n, m, Astar);//��������

		for (i = 0; i < m; i++)
		{
			for (j = 0; j < n; j++)
			{
				Jinv[j][i] = Astar[i*n + j];//�ſɱȾ�����α��ת��
			}
		}
	}

	//�ͷſռ�

	delete[] A;
	delete[] U;
	delete[] D;
	delete[] V;
	delete[] dummy_array;
	delete[] Astar;

	return err;
}


template<typename Type>int Matrix<Type>::Singular_Value_Decomposition
	(double* A, int nrows, int ncols, double* U, double* singular_values, double* V, double* dummy_array)
{
	////////////////////////////////////////////////////////////////////////////////
	//  int Singular_Value_Decomposition(double* A, int nrows, int ncols,         //
	//        double* U, double* singular_values, double* V, double* dummy_array) //
	//                                                                            //
	//  Description:                                                              //
	//     This routine decomposes an m x n matrix A, with m >= n, into a product //
	//     of the three matrices U, D, and V', i.e. A = UDV', where U is an m x n //
	//     matrix whose columns are orthogonal, D is a n x n diagonal matrix, and //
	//     V is an n x n orthogonal matrix.  V' denotes the transpose of V.  If   //
	//     m < n, then the procedure may be used for the matrix A'.  The singular //
	//     values of A are the diagonal elements of the diagonal matrix D and     //
	//     correspond to the positive square roots of the eigenvalues of the      //
	//     matrix A'A.                                                            //
	//                                                                            //
	//     This procedure programmed here is based on the method of Golub and     //
	//     Reinsch as given on pages 134 - 151 of the "Handbook for Automatic     //
	//     Computation vol II - Linear Algebra" edited by Wilkinson and Reinsch   //
	//     and published by Springer-Verlag, 1971.                                //
	//                                                                            //
	//     The Golub and Reinsch's method for decomposing the matrix A into the   //
	//     product U, D, and V' is performed in three stages:                     //
	//       Stage 1:  Decompose A into the product of three matrices U1, B, V1'  //
	//         A = U1 B V1' where B is a bidiagonal matrix, and U1, and V1 are a  //
	//         product of Householder transformations.                            //
	//       Stage 2:  Use Given' transformations to reduce the bidiagonal matrix //
	//         B into the product of the three matrices U2, D, V2'.  The singular //
	//         value decomposition is then UDV'where U = U2 U1 and V' = V1' V2'.  //
	//       Stage 3:  Sort the matrix D in decreasing order of the singular      //
	//         values and interchange the columns of both U and V to reflect any  //
	//         change in the order of the singular values.                        //
	//                                                                            //
	//     After performing the singular value decomposition for A, call          //
	//     Singular_Value_Decomposition to solve the equation Ax = B or call      //
	//     Singular_Value_Decomposition_Inverse to calculate the pseudo-inverse   //
	//     of A.                                                                  //
	//                                                                            //
	//  Arguments:                                                                //
	//     double* A                                                              //
	//        On input, the pointer to the first element of the matrix            //
	//        A[nrows][ncols].  The matrix A is unchanged.                        //
	//     int nrows                                                              //
	//        The number of rows of the matrix A.                                 //
	//     int ncols                                                              //
	//        The number of columns of the matrix A.                              //
	//     double* U                                                              //
	//        On input, a pointer to a matrix with the same number of rows and    //
	//        columns as the matrix A.  On output, the matrix with mutually       //
	//        orthogonal columns which is the left-most factor in the singular    //
	//        value decomposition of A.                                           //
	//     double* singular_values                                                //
	//        On input, a pointer to an array dimensioned to same as the number   //
	//        of columns of the matrix A, ncols.  On output, the singular values  //
	//        of the matrix A sorted in decreasing order.  This array corresponds //
	//        to the diagonal matrix in the singular value decomposition of A.    //
	//     double* V                                                              //
	//        On input, a pointer to a square matrix with the same number of rows //
	//        and columns as the columns of the matrix A, i.e. V[ncols][ncols].   //
	//        On output, the orthogonal matrix whose transpose is the right-most  //
	//        factor in the singular value decomposition of A.                    //
	//     double* dummy_array                                                    //
	//        On input, a pointer to an array dimensioned to same as the number   //
	//        of columns of the matrix A, ncols.  This array is used to store     //
	//        the super-diagonal elements resulting from the Householder reduction//
	//        of the matrix A to bidiagonal form.  And as an input to the Given's //
	//        procedure to reduce the bidiagonal form to diagonal form.           //
	//                                                                            //
	//  Return Values:                                                            //
	//     0  Success                                                             //
	//    -1  Failure - During the Given's reduction of the bidiagonal form to    //
	//                  diagonal form the procedure failed to terminate within    //
	//                  MAX_ITERA iterations.                           //
	//                                                                            //
	//  Example:                                                                  //
	//     #define M                                                              //
	//     #define N                                                              //
	//     double A[M][N];                                                        //
	//     double U[M][N];                                                        //
	//     double V[N][N];                                                        //
	//     double singular_values[N];                                             //
	//     double* dummy_array;                                                   //
	//                                                                            //
	//     (your code to initialize the matrix A)                                 //
	//     dummy_array = (double*) malloc(N * sizeof(double));                    //
	//     if (dummy_array == NULL) {printf(" No memory available\n"); exit(0); } //
	//                                                                            //
	//     err = Singular_Value_Decomposition((double*) A, M, N, (double*) U,     //
	//                              singular_values, (double*) V, dummy_array);   //
	//                                                                            //
	//     free(dummy_array);                                                     //
	//     if (err < 0) printf(" Failed to converge\n");                          //
	//     else { printf(" The singular value decomposition of A is \n");         //
	//           ...                                                              //
	////////////////////////////////////////////////////////////////////////////////
	//                                                                            //

	Householders_Reduction_to_Bidiagonal_Form(A, nrows, ncols, U, V,
		singular_values, dummy_array);

	if (Givens_Reduction_to_Diagonal_Form(nrows, ncols, U, V,
		singular_values, dummy_array) < 0) return -1;

	Sort_by_Decreasing_Singular_Values(nrows, ncols, singular_values, U, V);

	return 0;
}



template<typename Type>void Matrix<Type>::Householders_Reduction_to_Bidiagonal_Form
	(double* A, int nrows, int ncols, double* U, double* V, double* diagonal, double* superdiagonal)
{
	////////////////////////////////////////////////////////////////////////////////
	// static void Householders_Reduction_to_Bidiagonal_Form(double* A, int nrows,//
	//  int ncols, double* U, double* V, double* diagonal, double* superdiagonal )//
	//                                                                            //
	//  Description:                                                              //
	//     This routine decomposes an m x n matrix A, with m >= n, into a product //
	//     of the three matrices U, B, and V', i.e. A = UBV', where U is an m x n //
	//     matrix whose columns are orthogonal, B is a n x n bidiagonal matrix,   //
	//     and V is an n x n orthogonal matrix.  V' denotes the transpose of V.   //
	//     If m < n, then the procedure may be used for the matrix A'.  The       //
	//                                                                            //
	//     The matrix U is the product of Householder transformations which       //
	//     annihilate the sub diagonal components of A while the matrix V is       //
	//     the product of Householder transformations which annihilate the        //
	//     components of A to the right of the superdiagonal.                     //
	//                                                                            //
	//     The Householder transformation which leaves invariant the first k-1    //
	//     elements of the k-th column and annihilates the all the elements below //
	//     the diagonal element is P = I - (2/u'u)uu', u is an nrows-dimensional  //
	//     vector the first k-1 components of which are zero and the last         //
	//     components agree with the current transformed matrix below the diagonal//
	//     diagonal, the remaining k-th element is the diagonal element - s, where//
	//     s = (+/-)sqrt(sum of squares of the elements below the diagonal), the  //
	//     sign is chosen opposite that of the diagonal element.                  //
	//                                                                            //
	//  Arguments:                                                                //
	//     double* A                                                              //
	//        On input, the pointer to the first element of the matrix            //
	//        A[nrows][ncols].  The matrix A is unchanged.                        //
	//     int nrows                                                              //
	//        The number of rows of the matrix A.                                 //
	//     int ncols                                                              //
	//        The number of columns of the matrix A.                              //
	//     double* U                                                              //
	//        On input, a pointer to a matrix with the same number of rows and    //
	//        columns as the matrix A.  On output, the matrix with mutually       //
	//        orthogonal columns which is the left-most factor in the bidiagonal  //
	//        decomposition of A.                                                 //
	//     double* V                                                              //
	//        On input, a pointer to a square matrix with the same number of rows //
	//        and columns as the columns of the matrix A, i.e. V[ncols][ncols].   //
	//        On output, the orthogonal matrix whose transpose is the right-most  //
	//        factor in the bidiagonal decomposition of A.                        //
	//     double* diagonal                                                       //
	//        On input, a pointer to an array dimensioned to same as the number   //
	//        of columns of the matrix A, ncols.  On output, the diagonal of the  //
	//        bidiagonal matrix.                                                  //
	//     double* superdiagonal                                                  //
	//        On input, a pointer to an array dimensioned to same as the number   //
	//        of columns of the matrix A, ncols.  On output, the superdiagonal    //
	//        of the bidiagonal matrix.                                           //
	//                                                                            //
	//  Return Values:                                                            //
	//     The function is of type void and therefore does not return a value.    //
	//     The matrices U, V, and the diagonal and superdiagonal are calculated   //
	//     using the addresses passed in the argument list.                       //
	//                                                                            //
	//  Example:                                                                  //
	//     #define M                                                              //
	//     #define N                                                              //
	//     double A[M][N];                                                        //
	//     double U[M][N];                                                        //
	//     double V[N][N];                                                        //
	//     double diagonal[N];                                                    //
	//     double superdiagonal[N];                                               //
	//                                                                            //
	//     (your code to initialize the matrix A - Note this routine is not       //
	//     (accessible from outside i.e. it is declared static)                   //
	//                                                                            //
	//     Householders_Reduction_to_Bidiagonal_Form((double*) A, nrows, ncols,   //
	//                   (double*) U, (double*) V, diagonal, superdiagonal )      //
	//                                                                            //
	//     free(dummy_array);                                                     //
	//           ...                                                              //
	////////////////////////////////////////////////////////////////////////////////
	//                                                                            //


	int i, j, k, ip1;
	double s, s2, si, scale;
	double *pu, *pui, *pv, *pvi;
	double half_norm_squared;

	// Copy A to U

	memcpy(U, A, sizeof(double)* nrows * ncols);

	//

	diagonal[0] = 0.0;
	s = 0.0;
	scale = 0.0;
	for (i = 0, pui = U, ip1 = 1; i < ncols; pui += ncols, i++, ip1++) {
		superdiagonal[i] = scale * s;
		//       
		//                  Perform Householder transform on columns.
		//
		//       Calculate the normed squared of the i-th column vector starting at 
		//       row i.
		//
		for (j = i, pu = pui, scale = 0.0; j < nrows; j++, pu += ncols)
			scale += fabs(*(pu + i));

		if (scale > 0.0) {
			for (j = i, pu = pui, s2 = 0.0; j < nrows; j++, pu += ncols) {
				*(pu + i) /= scale;
				s2 += *(pu + i) * *(pu + i);
			}
			//
			//    
			//       Chose sign of s which maximizes the norm
			//  
			s = (*(pui + i) < 0.0) ? sqrt(s2) : -sqrt(s2);
			//
			//       Calculate -2/u'u
			//
			half_norm_squared = *(pui + i) * s - s2;
			//
			//       Transform remaining columns by the Householder transform.
			//
			*(pui + i) -= s;

			for (j = ip1; j < ncols; j++) {
				for (k = i, si = 0.0, pu = pui; k < nrows; k++, pu += ncols)
					si += *(pu + i) * *(pu + j);
				si /= half_norm_squared;
				for (k = i, pu = pui; k < nrows; k++, pu += ncols) {
					*(pu + j) += si * *(pu + i);
				}
			}
		}
		for (j = i, pu = pui; j < nrows; j++, pu += ncols) *(pu + i) *= scale;
		diagonal[i] = s * scale;
		//       
		//                  Perform Householder transform on rows.
		//
		//       Calculate the normed squared of the i-th row vector starting at 
		//       column i.
		//
		s = 0.0;
		scale = 0.0;
		if (i >= nrows || i == (ncols - 1)) continue;
		for (j = ip1; j < ncols; j++) scale += fabs(*(pui + j));
		if (scale > 0.0) {
			for (j = ip1, s2 = 0.0; j < ncols; j++) {
				*(pui + j) /= scale;
				s2 += *(pui + j) * *(pui + j);
			}
			s = (*(pui + ip1) < 0.0) ? sqrt(s2) : -sqrt(s2);
			//
			//       Calculate -2/u'u
			//
			half_norm_squared = *(pui + ip1) * s - s2;
			//
			//       Transform the rows by the Householder transform.
			//
			*(pui + ip1) -= s;
			for (k = ip1; k < ncols; k++)
				superdiagonal[k] = *(pui + k) / half_norm_squared;
			if (i < (nrows - 1)) {
				for (j = ip1, pu = pui + ncols; j < nrows; j++, pu += ncols) {
					for (k = ip1, si = 0.0; k < ncols; k++)
						si += *(pui + k) * *(pu + k);
					for (k = ip1; k < ncols; k++) {
						*(pu + k) += si * superdiagonal[k];
					}
				}
			}
			for (k = ip1; k < ncols; k++) *(pui + k) *= scale;
		}
	}

	// Update V
	pui = U + ncols * (ncols - 2);
	pvi = V + ncols * (ncols - 1);
	*(pvi + ncols - 1) = 1.0;
	s = superdiagonal[ncols - 1];
	pvi -= ncols;
	for (i = ncols - 2, ip1 = ncols - 1; i >= 0; i--, pui -= ncols,
		pvi -= ncols, ip1--) {
			if (s != 0.0) {
				pv = pvi + ncols;
				for (j = ip1; j < ncols; j++, pv += ncols)
					*(pv + i) = (*(pui + j) / *(pui + ip1)) / s;
				for (j = ip1; j < ncols; j++) {
					si = 0.0;
					for (k = ip1, pv = pvi + ncols; k < ncols; k++, pv += ncols)
						si += *(pui + k) * *(pv + j);
					for (k = ip1, pv = pvi + ncols; k < ncols; k++, pv += ncols)
						*(pv + j) += si * *(pv + i);
				}
			}
			pv = pvi + ncols;
			for (j = ip1; j < ncols; j++, pv += ncols) {
				*(pvi + j) = 0.0;
				*(pv + i) = 0.0;
			}
			*(pvi + i) = 1.0;
			s = superdiagonal[i];
	}

	// Update U

	pui = U + ncols * (ncols - 1);
	for (i = ncols - 1, ip1 = ncols; i >= 0; ip1 = i, i--, pui -= ncols) {
		s = diagonal[i];
		for (j = ip1; j < ncols; j++) *(pui + j) = 0.0;
		if (s != 0.0) {
			for (j = ip1; j < ncols; j++) {
				si = 0.0;
				pu = pui + ncols;
				for (k = ip1; k < nrows; k++, pu += ncols)
					si += *(pu + i) * *(pu + j);
				si = (si / *(pui + i)) / s;
				for (k = i, pu = pui; k < nrows; k++, pu += ncols)
					*(pu + j) += si * *(pu + i);
			}
			for (j = i, pu = pui; j < nrows; j++, pu += ncols){
				*(pu + i) /= s;
			}
		}
		else
			for (j = i, pu = pui; j < nrows; j++, pu += ncols) *(pu + i) = 0.0;
		*(pui + i) += 1.0;
	}

}



template<typename Type>int Matrix<Type>::Givens_Reduction_to_Diagonal_Form
	(int nrows, int ncols, double* U, double* V, double* diagonal, double* superdiagonal)
{

	////////////////////////////////////////////////////////////////////////////////
	// static int Givens_Reduction_to_Diagonal_Form( int nrows, int ncols,        //
	//         double* U, double* V, double* diagonal, double* superdiagonal )    //
	//                                                                            //
	//  Description:                                                              //
	//     This routine decomposes a bidiagonal matrix given by the arrays        //
	//     diagonal and superdiagonal into a product of three matrices U1, D and  //
	//     V1', the matrix U1 pre multiplies U and is returned in U, the matrix    //
	//     V1 pre multiplies V and is returned in V.  The matrix D is a diagonal   //
	//     matrix and replaces the array diagonal.                                //
	//                                                                            //
	//     The method used to annihilate the off diagonal elements is a variant    //
	//     of the QR transformation.  The method consists of applying Givens      //
	//     rotations to the right and the left of the current matrix until        //
	//     the new off-diagonal elements are chased out of the matrix.            //
	//                                                                            //
	//     The process is an iterative process which due to roundoff errors may   //
	//     not converge within a predefined number of iterations.  (This should   //
	//     be unusual.)                                                           //
	//                                                                            //
	//  Arguments:                                                                //
	//     int nrows                                                              //
	//        The number of rows of the matrix U.                                 //
	//     int ncols                                                              //
	//        The number of columns of the matrix U.                              //
	//     double* U                                                              //
	//        On input, a pointer to a matrix already initialized to a matrix     //
	//        with mutually orthogonal columns.   On output, the matrix with      //
	//        mutually orthogonal columns.                                        //
	//     double* V                                                              //
	//        On input, a pointer to a square matrix with the same number of rows //
	//        and columns as the columns of the matrix U, i.e. V[ncols][ncols].   //
	//        The matrix V is assumed to be initialized to an orthogonal matrix.  //
	//        On output, V is an orthogonal matrix.                               //
	//     double* diagonal                                                       //
	//        On input, a pointer to an array of dimension ncols which initially  //
	//        contains the diagonal of the bidiagonal matrix.  On output, the     //
	//        it contains the diagonal of the diagonal matrix.                    //
	//     double* superdiagonal                                                  //
	//        On input, a pointer to an array of dimension ncols which initially  //
	//        the first component is zero and the successive components form the  //
	//        superdiagonal of the bidiagonal matrix.                             //
	//                                                                            //
	//  Return Values:                                                            //
	//     0  Success                                                             //
	//    -1  Failure - The procedure failed to terminate within                  //
	//                  MAX_ITERA iterations.                           //
	//                                                                            //
	//  Example:                                                                  //
	//     #define M                                                              //
	//     #define N                                                              //
	//     double U[M][N];                                                        //
	//     double V[N][N];                                                        //
	//     double diagonal[N];                                                    //
	//     double superdiagonal[N];                                               //
	//     int err;                                                               //
	//                                                                            //
	//     (your code to initialize the matrices U, V, diagonal, and )            //
	//     ( superdiagonal.  - Note this routine is not accessible from outside)  //
	//     ( i.e. it is declared static.)                                         //
	//                                                                            //
	//     err = Givens_Reduction_to_Diagonal_Form( M,N,(double*)U,(double*)V,    //
	//                                                 diagonal, superdiagonal ); //
	//     if ( err < 0 ) printf("Failed to converge\n");                         //
	//     else { ... }                                                           //
	//           ...                                                              //
	////////////////////////////////////////////////////////////////////////////////
	//                                                                            //


	double epsilon;
	double c, s;
	double f, g, h;
	double x, y, z;
	double *pu, *pv;
	int i, j, k, m;
	int rotation_test;
	int iteration_count;

	for (i = 0, x = 0.0; i < ncols; i++) 
	{
		y = fabs(diagonal[i]) + fabs(superdiagonal[i]);
		if (x < y) x = y;
	}
	epsilon = x * DBL_EPSILON;
	for (k = ncols - 1; k >= 0; k--) {
		iteration_count = 0;
		while (1) {
			rotation_test = 1;
			for (m = k; m >= 0; m--) {
				if (fabs(superdiagonal[m]) <= epsilon) { rotation_test = 0; break; }
				if (fabs(diagonal[m - 1]) <= epsilon) break;
			}
			if (rotation_test) {
				c = 0.0;
				s = 1.0;
				for (i = m; i <= k; i++) {
					f = s * superdiagonal[i];
					superdiagonal[i] *= c;
					if (fabs(f) <= epsilon) break;
					g = diagonal[i];
					h = sqrt(f*f + g*g);
					diagonal[i] = h;
					c = g / h;
					s = -f / h;
					for (j = 0, pu = U; j < nrows; j++, pu += ncols) {
						y = *(pu + m - 1);
						z = *(pu + i);
						*(pu + m - 1) = y * c + z * s;
						*(pu + i) = -y * s + z * c;
					}
				}
			}
			z = diagonal[k];
			if (m == k) {
				if (z < 0.0) {
					diagonal[k] = -z;
					for (j = 0, pv = V; j < ncols; j++, pv += ncols)
						*(pv + k) = -*(pv + k);
				}
				break;
			}
			else {
				if (iteration_count >= MAX_ITERA) return -1;
				iteration_count++;
				x = diagonal[m];
				y = diagonal[k - 1];
				g = superdiagonal[k - 1];
				h = superdiagonal[k];
				f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
				g = sqrt(f * f + 1.0);
				if (f < 0.0) g = -g;
				f = ((x - z) * (x + z) + h * (y / (f + g) - h)) / x;
				// Next QR Transformtion
				c = 1.0;
				s = 1.0;
				for (i = m + 1; i <= k; i++) {
					g = superdiagonal[i];
					y = diagonal[i];
					h = s * g;
					g *= c;
					z = sqrt(f * f + h * h);
					superdiagonal[i - 1] = z;
					c = f / z;
					s = h / z;
					f = x * c + g * s;
					g = -x * s + g * c;
					h = y * s;
					y *= c;
					for (j = 0, pv = V; j < ncols; j++, pv += ncols) {
						x = *(pv + i - 1);
						z = *(pv + i);
						*(pv + i - 1) = x * c + z * s;
						*(pv + i) = -x * s + z * c;
					}
					z = sqrt(f * f + h * h);
					diagonal[i - 1] = z;
					if (z != 0.0) {
						c = f / z;
						s = h / z;
					}
					f = c * g + s * y;
					x = -s * g + c * y;
					for (j = 0, pu = U; j < nrows; j++, pu += ncols) {
						y = *(pu + i - 1);
						z = *(pu + i);
						*(pu + i - 1) = c * y + s * z;
						*(pu + i) = -s * y + c * z;
					}
				}
				superdiagonal[m] = 0.0;
				superdiagonal[k] = f;
				diagonal[k] = x;
			}
		}
	}
	return 0;
}



template<typename Type>void Matrix<Type>::Sort_by_Decreasing_Singular_Values
	(int nrows, int ncols, double* singular_values, double* U, double* V)
{
	////////////////////////////////////////////////////////////////////////////////
	// static void Sort_by_Decreasing_Singular_Values(int nrows, int ncols,       //
	//                            double* singular_values, double* U, double* V)  //
	//                                                                            //
	//  Description:                                                              //
	//     This routine sorts the singular values from largest to smallest        //
	//     singular value and interchanges the columns of U and the columns of V  //
	//     whenever a swap is made.  I.e. if the i-th singular value is swapped   //
	//     with the j-th singular value, then the i-th and j-th columns of U are  //
	//     interchanged and the i-th and j-th columns of V are interchanged.      //
	//                                                                            //
	//  Arguments:                                                                //
	//     int nrows                                                              //
	//        The number of rows of the matrix U.                                 //
	//     int ncols                                                              //
	//        The number of columns of the matrix U.                              //
	//     double* singular_values                                                //
	//        On input, a pointer to the array of singular values.  On output, the//
	//        sorted array of singular values.                                    //
	//     double* U                                                              //
	//        On input, a pointer to a matrix already initialized to a matrix     //
	//        with mutually orthogonal columns.  On output, the matrix with       //
	//        mutually orthogonal possibly permuted columns.                      //
	//     double* V                                                              //
	//        On input, a pointer to a square matrix with the same number of rows //
	//        and columns as the columns of the matrix U, i.e. V[ncols][ncols].   //
	//        The matrix V is assumed to be initialized to an orthogonal matrix.  //
	//        On output, V is an orthogonal matrix with possibly permuted columns.//
	//                                                                            //
	//  Return Values:                                                            //
	//        The function is of type void.                                       //
	//                                                                            //
	//  Example:                                                                  //
	//     #define M                                                              //
	//     #define N                                                              //
	//     double U[M][N];                                                        //
	//     double V[N][N];                                                        //
	//     double diagonal[N];                                                    //
	//                                                                            //
	//     (your code to initialize the matrices U, V, and diagonal. )            //
	//     ( - Note this routine is not accessible from outside)                  //
	//     ( i.e. it is declared static.)                                         //
	//                                                                            //
	//     Sort_by_Decreasing_Singular_Values(nrows, ncols, singular_values,      //
	//                                                 (double*) U, (double*) V); //
	//           ...                                                              //
	////////////////////////////////////////////////////////////////////////////////
	//                                                                            //


	int i, j, max_index;
	double temp;
	double *p1, *p2;

	for (i = 0; i < ncols - 1; i++) {
		max_index = i;
		for (j = i + 1; j < ncols; j++)
			if (singular_values[j] > singular_values[max_index])
				max_index = j;
		if (max_index == i) continue;
		temp = singular_values[i];
		singular_values[i] = singular_values[max_index];
		singular_values[max_index] = temp;
		p1 = U + max_index;
		p2 = U + i;
		for (j = 0; j < nrows; j++, p1 += ncols, p2 += ncols) {
			temp = *p1;
			*p1 = *p2;
			*p2 = temp;
		}
		p1 = V + max_index;
		p2 = V + i;
		for (j = 0; j < ncols; j++, p1 += ncols, p2 += ncols) {
			temp = *p1;
			*p1 = *p2;
			*p2 = temp;
		}
	}
}



template<typename Type>void Matrix<Type>::Singular_Value_Decomposition_Solve
	(double* U, double* D, double* V, double tolerance, int nrows, int ncols, double *B, double* x)
{

	////////////////////////////////////////////////////////////////////////////////
	//  void Singular_Value_Decomposition_Solve(double* U, double* D, double* V,  //
	//              double tolerance, int nrows, int ncols, double *B, double* x) //
	//                                                                            //
	//  Description:                                                              //
	//     This routine solves the system of linear equations Ax=B where A =UDV', //
	//     is the singular value decomposition of A.  Given UDV'x=B, then         //
	//     x = V(1/D)U'B, where 1/D is the pseudo-inverse of D, i.e. if D[i] > 0  //
	//     then (1/D)[i] = 1/D[i] and if D[i] = 0, then (1/D)[i] = 0.  Since      //
	//     the singular values are subject to round-off error.  A tolerance is    //
	//     given so that if D[i] < tolerance, D[i] is treated as if it is 0.      //
	//     The default tolerance is D[0] * DBL_EPSILON * ncols, if the user       //
	//     specified tolerance is less than the default tolerance, the default    //
	//     tolerance is used.                                                     //
	//                                                                            //
	//  Arguments:                                                                //
	//     double* U                                                              //
	//        A matrix with mutually orthonormal columns.                         //
	//     double* D                                                              //
	//        A diagonal matrix with decreasing non-negative diagonal elements.   //
	//        i.e. D[i] > D[j] if i < j and D[i] >= 0 for all i.                  //
	//     double* V                                                              //
	//        An orthogonal matrix.                                               //
	//     double tolerance                                                       //
	//        An lower bound for non-zero singular values (provided tolerance >   //
	//        ncols * DBL_EPSILON * D[0]).                                        //
	//     int nrows                                                              //
	//        The number of rows of the matrix U and B.                           //
	//     int ncols                                                              //
	//        The number of columns of the matrix U.  Also the number of rows and //
	//        columns of the matrices D and V.                                    //
	//     double* B                                                              //
	//        A pointer to a vector dimensioned as nrows which is the  right-hand //
	//        side of the equation Ax = B where A = UDV'.                         //
	//     double* x                                                              //
	//        A pointer to a vector dimensioned as ncols, which is the least      //
	//        squares solution of the equation Ax = B where A = UDV'.             //
	//                                                                            //
	//  Return Values:                                                            //
	//        The function is of type void.                                       //
	//                                                                            //
	//  Example:                                                                  //
	//     #define M                                                              //
	//     #define N                                                              //
	//     #define NB                                                             //
	//     double U[M][N];                                                        //
	//     double V[N][N];                                                        //
	//     double D[N];                                                           //
	//     double B[M];                                                           //
	//     double x[N];                                                           //
	//     double tolerance;                                                      //
	//                                                                            //
	//     (your code to initialize the matrices U,D,V,B)                         //
	//                                                                            //
	//     Singular_Value_Decomposition_Solve((double*) U, D, (double*) V,        //
	//                                              tolerance, M, N, B, x, bcols) //
	//                                                                            //
	//     printf(" The solution of Ax=B is \n");                                 //
	//           ...                                                              //
	////////////////////////////////////////////////////////////////////////////////
	//                                                                            //


	int i, j, k;
	double *pu, *pv;
	double dum;

	dum = DBL_EPSILON * D[0] * (double)ncols;
	if (tolerance < dum) tolerance = dum;

	for (i = 0, pv = V; i < ncols; i++, pv += ncols) {
		x[i] = 0.0;
		for (j = 0; j < ncols; j++)
			if (D[j] > tolerance) {
				for (k = 0, dum = 0.0, pu = U; k < nrows; k++, pu += ncols)
					dum += *(pu + j) * B[k];
				x[i] += dum * *(pv + j) / D[j];
			}
	}
}



template<typename Type>void Matrix<Type>::Singular_Value_Decomposition_Inverse
	(double* U, double* D, double* V, double tolerance, int nrows, int ncols, double *Astar)
{
	int i, j, k;
	double *pu, *pv, *pa;
	double dum;

	dum = DBL_EPSILON * D[0] * (double)ncols;
	if (tolerance < dum) tolerance = dum;
	for (i = 0, pv = V, pa = Astar; i < ncols; i++, pv += ncols)
		for (j = 0, pu = U; j < nrows; j++, pa++)
			for (k = 0, *pa = 0.0; k < ncols; k++, pu++)
				if (D[k] > tolerance) *pa += *(pv + k) * *pu / D[k];
}

#pragma endregion pinv2