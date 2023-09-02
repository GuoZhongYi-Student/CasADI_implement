#pragma once

#include "stdio.h"
#include <vector>
#include <stdlib.h>
#include "math.h"

//��ȡ�ļ�����
extern int Read_txt_file(char filename[], double* value);

//д���ļ�����
extern int Write_txt_file(char filename[], double* value);

//���ֵ
extern int mean_function(double *data, double *mean, int size_column,int size_row, int mode);

//���׼��
extern int std_function(double* data, double* mean, int size_column, int size_row, int mode, double* std);

//������Է�����
extern int mldivide(double* data,double* value, int size_column, int size_row,double* root);

//����ת��
extern int matrix_transpose(double* data, int size_column, int size_row, double* data_transpose);

//���������Է��������
extern int lowertriangle_mldivide(double* data, double* value, int size_column, double* root);

//���������Է��������
extern int uppertriangle_mldivide(double* data, double* value, int size_column, double* root);
