
#include "stdio.h"



//数据标准化
extern int standardize(double* data, double* mean, double* std, int size_column, int size_row, double* data_standardized);


//数据反标准化
extern int destandardize(double* data, double* mean, double* std, int size_column, int size_row);



struct GP_hyperparameters
{
	double ell[8];
	double sf2;
	double sn2;
};

struct GP_init_data
{
	double* L;
	double* alfa;
};

//高斯过程回归模型初始化
extern int GP_Init(struct GP_hyperparameters* hyperparameters, double* feature_data, double* lable_data, int size_column, int size_row, struct GP_init_data* GP_init);

extern int Covar_function_init(struct GP_hyperparameters* hyperparameters, double* feature_data, int size_column, int size_row,double* Covar_function);

extern int Chelesky_decomposition(double* data, int matrix_dimension, double* L);

extern int alfa_solver(struct GP_init_data* GP_init,double* lable_data,int size_column);


