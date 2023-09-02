#include"stdio.h"
#include"Common_lib.h"
#include <vector>



//��ֵ���� ���ά����ľ�ֵ
/*
	*data            ����Ķ�ά����ת��Ϊһά����
	*mean            ����ľ�ֵ����
	size_column      �����ά���������
	size_row         �����ά���������
	mode             ���ֵ�ķ��� ��0 ��ÿ�еľ�ֵ 1 ��ÿ�еľ�ֵ�� 
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

//��׼��� �����ά����ı�׼��
/*
	*data            ����Ķ�ά����ת��Ϊһά����
	*mean            ����ľ�ֵ����
	size_column      �����ά���������
	size_row         �����ά���������
	mode             ���ֵ�ķ��� ��0 ��ÿ�еı�׼�� 1 ��ÿ�еı�׼�
	*std             ����ı�׼��
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

//������Է����飨�������� dataΪ�����Ǿ���
/*
	*data            ����Ķ�ά����Ϊ�����Ǿ���
	*value           ���������
	size_column      �����ά���������
	size_row         �����ά���������
	*root            ���Է�����ĸ�
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

//������Է����飨�������� dataΪ�����Ǿ���
/*
	*data            ����Ķ�ά����Ϊ�����Ǿ���
	*value           ���������
	size_column      �����ά���������
	size_row         �����ά���������
	*root            ���Է�����ĸ�
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


//������ת��
/*
	*data            �������������ʽ��
	size_column      �����ά���������
	size_row         �����ά���������
	*data_transpose  ���ת�ú�ľ���
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
