#include"GPR_lib.h"
#include"Common_lib.h"


//高斯过程初始化函数
/*
	hyperparameters			高斯过程回归模型的超参数结构体
	*feature_data			输入的特征数据
	*lable_data				输入的标签数据（一维）
	*size_column            输入的特征矩阵的行数
	*size_row               输入的特征矩阵的列数
	*GP_init                输出高斯过程回归模型初始化之后的变量
*/
int GP_Init(struct GP_hyperparameters* hyperparameters, double* feature_data, double* lable_data, int size_column, int size_row,struct GP_init_data* GP_init)
{
	double* Covar_function;
	Covar_function=(double*)malloc(sizeof(double) * size_column * size_column);


	//初始化协方差函数
	Covar_function_init(hyperparameters, feature_data, size_column, size_row, Covar_function);

	//Chelesky矩阵分解
	Chelesky_decomposition(Covar_function, size_column, GP_init->L);

	//alfa求解
	alfa_solver(GP_init, lable_data,size_column);

	free(Covar_function);
	return 0;
}

int destandardize(double* data, double* mean, double* std, int size_column, int size_row)
{

	return 0;
}

//数据标准化
/*
	*data				 需要标准化的数据矩阵
	*mean				 输入的数据矩阵的均值
	*std				 输入的数据矩阵的标准差
	*size_column         输入的数据矩阵的行数
	*size_row            输入的数据矩阵的列数
	*data_standardized   输出标准化之后的数据矩阵
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

//协方差矩阵初始化函数
/*
	hyperparameters      协方差函数的超参数
	*feature_data        输入的特征数据
	*size_column         输入的特征矩阵行数（即数据个数）
	*size_row            输入的特征矩阵的列数（即数据的特征个数）
	*Covar_function      输出的协方差矩阵
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


//chelesky分解
/*
	*data                输入的正定矩阵
	*matrix_dimension    输入的正定矩阵维度
	*L                   chelesky分解后得到的三角矩阵
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

//chelesky分解
/*
	*data                输入的正定矩阵
	*matrix_dimension    输入的正定矩阵维度
	*L                   chelesky分解后得到的三角矩阵
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
