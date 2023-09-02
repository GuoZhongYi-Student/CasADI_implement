#include"stdio.h"
#include"Common_lib.h"
#include <vector>



//均值函数 求二维矩阵的均值
/*
	*data            输入的二维矩阵转换为一维向量
	*mean            输出的均值向量
	size_column      输入二维矩阵的行数
	size_row         输入二维矩阵的列数
	mode             求均值的方法 （0 求每行的均值 1 求每列的均值） 
*/
int mean_function(double* data, double* mean, int size_column, int size_row, int mode)
{
	double sum_tmp=0;
	if (mode == 0)
	{
		for (int i = 0; i < size_column; i++)
		{
			for (int j = 0; j < size_row; j++)
			{
				sum_tmp= sum_tmp + *(data + i * size_column + j);
			}
			sum_tmp = sum_tmp / size_row;
			*(mean + i) = sum_tmp;
			sum_tmp = 0;
		}
		return 0;
	}
	else if (mode == 1)
	{
		for (int i = 0; i < size_row; i++)
		{
			for (int j = 0; j < size_column; j++)
			{
				sum_tmp = sum_tmp + *(data + j * size_row +i);
			}
			sum_tmp = sum_tmp / size_column;
			*(mean + i) = sum_tmp;
			sum_tmp = 0;
		}
		return 0;
	}
	else
	{
		printf("mode is not support!\n");
		return 0;
	}
}

//标准差函数 可求二维矩阵的标准差
/*
	*data            输入的二维矩阵转换为一维向量
	*mean            输入的均值向量
	size_column      输入二维矩阵的行数
	size_row         输入二维矩阵的列数
	mode             求均值的方法 （0 求每行的标准差 1 求每列的标准差）
	*std             输出的标准差
*/
int std_function(double* data, double* mean, int size_column, int size_row, int mode,double* std)
{
	double sum_tmp = 0;
	if (mode == 0)
	{
		for (int i = 0; i < size_column; i++)
		{
			for (int j = 0; j < size_row; j++)
			{
				sum_tmp = sum_tmp + (*(data + i * size_column + j) - *(mean+ i))* (*(data + i * size_column + j) - *(mean + i));
			}
			sum_tmp = sqrt(sum_tmp / (size_row - 1));
			*(std + i) = sum_tmp;
			sum_tmp = 0;
		}
		return 0;
	}
	else if (mode == 1)
	{
		for (int i = 0; i < size_row; i++)
		{
			for (int j = 0; j < size_column; j++)
			{
				sum_tmp = sum_tmp + (*(data + j * size_row + i)- *(mean + i))* (*(data + j * size_row + i) - *(mean + i));
			}
			sum_tmp = sqrt(sum_tmp / (size_column-1));
			*(std + i) = sum_tmp;
			sum_tmp = 0;
		}
		return 0;
	}
	else
	{
		printf("mode is not support!\n");
		return 0;
	}

}

//求解线性方程组（适用条件 data为下三角矩阵）
/*
	*data            输入的二维矩阵，为下三角矩阵
	*value           输入的向量
	size_column      输入二维矩阵的行数
	size_row         输入二维矩阵的列数
	*root            线性方程组的根
*/
int lowertriangle_mldivide(double* data, double* value, int size_column, double* root)
{
	double sum = 0;
	root[0] = value[0] / data[0];
	for (int i = 1; i < size_column; i++)
	{
		for (int j = 0; j < i; j++)
			sum += data[i * size_column + j] * root[j];
		root[i] = (value[i] - sum) / data[i * size_column + i];
		sum = 0;
	}
	return 0;
}

//求解线性方程组（适用条件 data为上三角矩阵）
/*
	*data            输入的二维矩阵，为上三角矩阵
	*value           输入的向量
	size_column      输入二维矩阵的行数
	size_row         输入二维矩阵的列数
	*root            线性方程组的根
*/
int uppertriangle_mldivide(double* data, double* value, int size_column, double* root)
{
	double sum = 0;
	root[size_column-1] = value[size_column-1] / data[size_column * size_column - 1];
	for (int i = size_column-1; i >= 0; i--)
	{
		for (int j = size_column-1; j > i; j--)
			sum += data[i * size_column + j] * root[j];
		root[i] = (value[i] - sum) / data[i * size_column + i];
		sum = 0;
	}
	return 0;
}


//矩阵求转置
/*
	*data            输入矩阵（向量形式）
	size_column      输入二维矩阵的行数
	size_row         输入二维矩阵的列数
	*data_transpose  输出转置后的矩阵
*/
int matrix_transpose(double* data, int size_column, int size_row, double* data_transpose)
{
	for (int i = 0; i < size_column; i++)
	{
		for (int j = 0; j < size_row; j++)
		{
			*(data_transpose+ j* size_column+i) = *(data + i * size_row + j);
		}
	}
	return 0;
}