#pragma once

#include "stdio.h"
#include <vector>



//读取文件数据
extern int Read_txt_file(char filename[], double* value);

//写入文件数据
extern int Write_txt_file(char filename[], double* value);

//求均值
extern int mean_function(double *data, double *mean, int size_column,int size_row, int mode);

//求标准差
extern int std_function(double* data, double* mean, int size_column, int size_row, int mode, double* std);

//求解线性方程组
extern int mldivide(double* data,double* value, int size_column, int size_row,double* root);

//矩阵转置
extern int matrix_transpose(double* data, int size_column, int size_row, double* data_transpose);

//下三角线性方程组求解
extern int lowertriangle_mldivide(double* data, double* value, int size_column, double* root);

//上三角线性方程组求解
extern int uppertriangle_mldivide(double* data, double* value, int size_column, double* root);
