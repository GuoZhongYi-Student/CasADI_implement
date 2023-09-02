#include"GPR_lib.h"
#include"Common_lib.h"


//��˹���̳�ʼ������
/*
	hyperparameters			��˹���̻ع�ģ�͵ĳ������ṹ��
	*feature_data			�������������
	*lable_data				����ı�ǩ���ݣ�һά��
	*size_column            ������������������
	*size_row               ������������������
	*GP_init                �����˹���̻ع�ģ�ͳ�ʼ��֮��ı���
*/
int GP_Init(struct GP_hyperparameters* hyperparameters, double* feature_data, double* lable_data, int size_column, int size_row,struct GP_init_data* GP_init)
{
	double* Covar_function;
	Covar_function=(double*)malloc(sizeof(double) * size_column * size_column);


	//��ʼ��Э�����
	Covar_function_init(hyperparameters, feature_data, size_column, size_row, Covar_function);

	//Chelesky����ֽ�
	Chelesky_decomposition(Covar_function, size_column, GP_init->L);

	//alfa���
	alfa_solver(GP_init, lable_data,size_column);

	free(Covar_function);
	return 0;
}

int destandardize(double* data, double* mean, double* std, int size_column, int size_row)
{

	return 0;
}

//���ݱ�׼��
/*
	*data				 ��Ҫ��׼�������ݾ���
	*mean				 ��������ݾ���ľ�ֵ
	*std				 ��������ݾ���ı�׼��
	*size_column         ��������ݾ��������
	*size_row            ��������ݾ��������
	*data_standardized   �����׼��֮������ݾ���
*/
int standardize(double *data,double *mean,double *std, int size_column, int size_row,double *data_standardized)
{
	for (int i = 0; i < size_row; i++)
	{
		for (int j = 0; j < size_column; j++)
		{
			*(data_standardized+ j * size_row + i) = (*(data + j * size_row + i) - *(mean + i))/ *(std + i);
		}
	}
	return 0;
}

//Э��������ʼ������
/*
	hyperparameters      Э������ĳ�����
	*feature_data        �������������
	*size_column         ������������������������ݸ�����
	*size_row            �������������������������ݵ�����������
	*Covar_function      �����Э�������
*/
int Covar_function_init(struct GP_hyperparameters* hyperparameters, double* feature_data, int size_column, int size_row,double* Covar_function)
{
	double sum_tmp = 0;

	for (int i = 0; i < size_column; i++)
	{
		for (int j = 0; j < size_column; j++)
		{
			for (int k = 0; k < size_row; k++)
			{
				sum_tmp = sum_tmp + (*(feature_data + j * size_row + k) - *(feature_data + i * size_row + k)) * (*(feature_data + j * size_row + k) - *(feature_data + i * size_row + k)) / (hyperparameters->ell[k] * hyperparameters->ell[k]);
			}
			sum_tmp = hyperparameters->sf2 * exp(-sum_tmp / 2);
			if (i == j)
			{
				sum_tmp = sum_tmp + hyperparameters->sn2;
			}
			*(Covar_function + j * size_column + i) = sum_tmp;

			sum_tmp = 0;
		}
	}

	return 0;
}


//chelesky�ֽ�
/*
	*data                �������������
	*matrix_dimension    �������������ά��
	*L                   chelesky�ֽ��õ������Ǿ���
*/
int Chelesky_decomposition(double* data, int matrix_dimension, double* L)
{
	for (int k = 0; k < matrix_dimension; k++)
	{
		double sum = 0;
		for (int i = 0; i < k; i++)
			sum += L[k* matrix_dimension+i] * L[k * matrix_dimension + i];
		sum = data[k * matrix_dimension + k] - sum;
		L[k * matrix_dimension + k] = sqrt(sum > 0 ? sum : 0);
		for (int i = k + 1; i < matrix_dimension; i++)
		{
			sum = 0;
			for (int j = 0; j < k; j++)
				sum += L[i * matrix_dimension + j] * L[k * matrix_dimension + j];
			L[i * matrix_dimension + k] = (data[i * matrix_dimension + k] - sum) / L[k * matrix_dimension + k];
		}
		for (int j = 0; j < k; j++)
			L[j * matrix_dimension + k] = 0;
	}
	return 0;
}

//chelesky�ֽ�
/*
	*data                �������������
	*matrix_dimension    �������������ά��
	*L                   chelesky�ֽ��õ������Ǿ���
*/
int alfa_solver(struct GP_init_data* GP_init, double* lable_data, int size_column)
{
	double* alfa_tmp;
	alfa_tmp = (double*)malloc(sizeof(double) * size_column);

	double* matrix_transpose_tmp;
	matrix_transpose_tmp = (double*)malloc(sizeof(double) * size_column * size_column);

	matrix_transpose(GP_init->L, size_column, size_column, matrix_transpose_tmp);
	lowertriangle_mldivide(GP_init->L, lable_data, size_column, alfa_tmp);
	uppertriangle_mldivide(matrix_transpose_tmp, alfa_tmp, size_column, GP_init->alfa);

	free(alfa_tmp);
	free(matrix_transpose_tmp);
	return 0;
}
