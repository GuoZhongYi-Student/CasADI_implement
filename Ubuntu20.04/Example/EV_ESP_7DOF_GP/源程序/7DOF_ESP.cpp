#include <iostream>
#include <vector>
#include <iomanip>
#include <time.h>
#include "casadi/casadi.hpp"
#include "Common_lib.h"
#include "GPR_lib.h"




#define Predictive_Horizon 8
#define Control_Horizon 2
#define Control_dimension 5
#define state_dimension 7
#define N 300
#define Dataset_dimension 10
#define feature_dimension 8
#define lable_dimension 2
#define state_dimension_gp 3 // ���ڸ�˹�ع��״̬����ά��

#define averagebyrow 0
#define averagebycolumn 1



using namespace casadi;
int timescale = 0;
double refer_steerangle[N+20]; //�ο�ֵӦ�ñ�ʵ��ֵ�������Ԥ��ʱ���

int reference_generation(double Vx, double* yawrate_r)
{
	double Lf = 1.05;
	double Lr = 1.61;
	double Kf_FAI = 43000;
	double Kr_FAI = 28662;
	double L = Lf + Lr;
	double M = 1572;
	double miu = 0.8;
	double Kv = (Lr * M) / (Kf_FAI * L) - (Lf * M) / (Kr_FAI * L); //ת�����ݶ�
	

	for (int i = 0; i < Predictive_Horizon; i++)
	{
		yawrate_r[i] = Vx * refer_steerangle[timescale + i] / (L + Vx * Vx * Kv);

		if (yawrate_r[i] > (0.85 * miu * 9.8 / Vx))
		{
			yawrate_r[i] = 0.85 * miu * 9.8 / Vx;
		}
		if(yawrate_r[i] < (-0.85 * miu * 9.8 / Vx))
		{
			yawrate_r[i] = -0.85 * miu * 9.8 / Vx;
		}	
		//std::cout << yawrate_r[i] << std::endl;
	}

	return 0;
	}

int Read_txt_file(char filename[],double* value)
{
	FILE* fpRead_file = fopen(filename, "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &value[i]);
	}
	fclose(fpRead_file);
}

int Write_txt_file(char filename[], double* value)
{
	FILE* fpWrite_file = fopen(filename, "w");
	if (fpWrite_file == NULL)
	{
		printf("Write file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fprintf(fpWrite_file, "%lf\n", value[i]);
	}
	fclose(fpWrite_file);
}


int main() {

	clock_t start, end;
	double	Re = 0.325;					// ��̥�뾶(��λ:��)
	double	M = 1572;					// ����������(��λ:ǧ��)
	double	Lf = 1.05;					// �������ĵ�ǰ��ľ���(��λ:��)
	double	Lr = 1.61;					// �������ĵ�����ľ���(��λ:��)
	double	Iz = 2059.2;				// �����Ƴ�������ϵz���ת������(��λ:)
	double	Cf = 43000;					// ����ǰ��Ĳ���Ǹն�(��λ:)
	double	Cr = 28662;					// ��������Ĳ���Ǹն�(��λ:)
	double	dr = 1.55;					// �����־�
	double	Iw = 22.5;					// ��̥ת������
	double	Tt = 3.6;					// ����������
	// Controller Parameter
	double	Ts = 0.03;					// ����ʱ��(��λ:��)
	double	Tire_number = 4;			// ��̥����

	// Burckhardt Tire Model Parameter Miu = 0.8
	double	C1 = 1.23;
	double  C2 = 24;
	double  C3 = 2.8;
	double  ks = 0.85;
	
	double dataset[N*Dataset_dimension] = { 0 };
	double feature_data[N][feature_dimension] = { 0 };
	double lable_data[N][lable_dimension] = { 0 };

	FILE* fpRead_file = fopen("300data.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N * Dataset_dimension; i++)
	{
		fscanf(fpRead_file, "%lf", &dataset[i]);
	}
	fclose(fpRead_file);

//�����ݼ���Ϊ�������ݺͱ�ǩ����
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < Dataset_dimension; j++)
		{
			if (j < feature_dimension)
			{
				feature_data[i][j] = dataset[i * Dataset_dimension + j];
			}
			else
			{
				lable_data[i][j- feature_dimension] = dataset[i * Dataset_dimension + j];
			}
		}
	}
	double feature_data_1D[feature_dimension * N] = { 0 };
	double lable_data_1D[lable_dimension * N] = { 0 };
	double feature_data_1D_standardized[feature_dimension * N] = { 0 };
	double lable_data_1D_standardized[lable_dimension * N] = { 0 };
//	printf("%lf", feature_data[0][0]);
//	printf("%lf", lable_data[0][0]);
// 
//��ά����ת��Ϊһά
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < feature_dimension; j++)
		{
			feature_data_1D[i * feature_dimension + j] = feature_data[i][j];
		}
	}
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < lable_dimension; j++)
		{
			lable_data_1D[i * lable_dimension + j] = lable_data[i][j];
		}
	}

	double mean_data_feature[feature_dimension] = {0};
	double mean_data_lable[lable_dimension] = { 0 };
	double std_data_feature[feature_dimension] = {0};
	double std_data_lable[lable_dimension] = { 0 };
	//mean_data = (double*)malloc(sizeof(double)* N);

	double Vx0[N], beta0[N], gamma0[N], delta0[N], Tfl[N], Tfr[N], Trl[N], Trr[N], wfl[N],  wfr[N], wrl[N], wrr[N];
	double Fzfl[N], Fzfr[N], Fzrl[N], Fzrr[N];
	
	//���������ļ�
	char vx_filename[]          = "Vx.txt";
	char yawrate_filename[]     = "gamma.txt";
	char slipangle_filename[]   = "beta.txt";
	char wfl_filename[]		    = "wfl.txt";
	char wfr_filename[]			= "wfr.txt";
	char wrl_filename[]			= "wrl.txt";
	char wrr_filename[]			= "wrr.txt";
	char Fzfl_filename[]        = "Fzfl.txt";
	char Fzfr_filename[]		= "Fzfr.txt";
	char Fzrl_filename[]	    = "Fzrl.txt";
	char Fzrr_filename[]		= "Fzrr.txt";
	char reference_filename[]   = "reference.txt";

	//��������ļ�
	char delta_filename[] = "delta.txt";
	char Tfl_filename[] = "Tfl.txt";
	char Tfr_filename[] = "Tfr.txt";
	char Trl_filename[] = "Trl.txt";
	char Trr_filename[] = "Trr.txt";


	fpRead_file = fopen("Vx.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &Vx0[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("gamma.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &gamma0[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("beta.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &beta0[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("wfl.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &wfl[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("wfr.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &wfr[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("wrl.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &wrl[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("wrr.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &wrr[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("Fzfl.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &Fzfl[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("Fzfr.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &Fzfr[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("Fzrl.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &Fzrl[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("Fzrr.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &Fzrr[i]);
	}
	fclose(fpRead_file);

	fpRead_file = fopen("reference.txt", "r");
	if (fpRead_file == NULL)
	{
		printf("read file failed\n");
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_file, "%lf", &refer_steerangle[i]);
	}
	fclose(fpRead_file);


	//�����м����
	/*double alphaf, alphar;									//�����ֲ�ƫ��
	double speed_L1, speed_R1, speed_L2, speed_R2;			//�����ٶ�
	double Kappa_L1, Kappa_R1, Kappa_L2, Kappa_R2; 
	double Kappa_r_L1, Kappa_r_R1, Kappa_r_L2, Kappa_r_R2;	//�ۺϻ�����
	double miu_com_L1, miu_com_R1, miu_com_L2, miu_com_R2;	//�ۺϸ�����
	double Fy_L1, Fy_R1, Fy_L2, Fy_R2;						//������
	double Fx_L1, Fx_R1, Fx_L2, Fx_R2;						//������
	double dxdt[state_dimension] = { 0 };					//״̬�仯��*/


	// define CasADI varible
	SX  CV_1D = SX::sym("CV_1D", Control_Horizon * Control_dimension);
	SX  SV_1D = SX::sym("SV_1D", Predictive_Horizon * state_dimension);
	SX  x_tmp = SX::sym("SV_tmp", state_dimension);
	SX  u_tmp = SX::sym("CV_tmp", Control_dimension);
	SX  dxdt = SX::sym("dxdt", state_dimension);
	SX  Fz = SX::sym("Fz", Tire_number);
	SX  CV = SX::sym("control variable", Control_Horizon, Control_dimension);
	SX  SV = SX::sym("state variable", Predictive_Horizon, state_dimension);
	SX  ISV = SX::sym("SV_initial", state_dimension);
	SX  ICV = SX::sym("CV_initial", Control_dimension);
	SX  Setpoint = SX::sym("Setpoint", Predictive_Horizon);
	SX  con_eq = SX::sym("constraints_eq", Predictive_Horizon, state_dimension);
	SX  Tt_eq = SX::sym("Tt_eq", Control_Horizon, 1);
	SX  sum_objective = 0, sum_delta_u = 0, cost = 0;

	// Multiple Shooting initial value
	std::vector<double>	u0_initial = { 0, 0.9, 0.9, 0.9, 0.9 };
	std::vector<double>	u0;
	for (int i = 0; i < Control_Horizon; i++)
	{
		u0.insert(u0.end(), u0_initial.begin(), u0_initial.end());
	}

	// x0 = [0.000000000000001, 0.000000000000001, 57, 57, 57, 57, 16.66]; ��ʼ�ٶ�Ϊ60km/h
	std::vector<double> x0_initial = { 0.000000000000001, 0.000000000000001, 75, 75, 75, 75, 25};
	std::vector<double> x0;
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		x0.insert(x0.end(), x0_initial.begin(), x0_initial.end());
	}

	std::vector<double> IDV;     
	IDV.insert(IDV.end(), u0.begin(), u0.end());
	IDV.insert(IDV.end(), x0.begin(), x0.end());
	

	// ��ƫ�Ǽ���
	SX alphaf = x_tmp(0) + (Lf / x_tmp(6)) * x_tmp(1) - u_tmp(0) + 0.0000001;
	SX alphar = x_tmp(0) - (Lr / x_tmp(6)) * x_tmp(1) + 0.0000001;
	// �����ʼ���
	SX speed_L1 = (x_tmp(6) - dr / 2 * x_tmp(1)) * cos(u_tmp(0)) + (x_tmp(0) * x_tmp(6) + Lf * x_tmp(1)) * sin(u_tmp(0));
	SX speed_R1 = (x_tmp(6) + dr / 2 * x_tmp(1)) * cos(u_tmp(0)) + (x_tmp(0) * x_tmp(6) + Lf * x_tmp(1)) * sin(u_tmp(0));
	SX speed_L2 = (x_tmp(6) - dr / 2 * x_tmp(1));
	SX speed_R2 = (x_tmp(6) + dr / 2 * x_tmp(1));

	SX Kappa_L1 = (x_tmp(2) * Re - speed_L1) / speed_L1;
	SX Kappa_R1 = (x_tmp(3) * Re - speed_R1) / speed_R1;
	SX Kappa_L2 = (x_tmp(4) * Re - speed_L2) / speed_L2;
	SX Kappa_R2 = (x_tmp(5) * Re - speed_R2) / speed_R2;
	// �ۺϻ����ʼ���
	SX Kappa_r_L1 = sqrt(Kappa_L1 * Kappa_L1 + alphaf * alphaf);
	SX Kappa_r_R1 = sqrt(Kappa_R1 * Kappa_R1 + alphaf * alphaf);
	SX Kappa_r_L2 = sqrt(Kappa_L2 * Kappa_L2 + alphar * alphar);
	SX Kappa_r_R2 = sqrt(Kappa_R2 * Kappa_R2 + alphar * alphar);

	SX miu_com_L1 = C1 * (1 - exp(-C2 * Kappa_r_L1)) - C3 * Kappa_r_L1;
	SX miu_com_R1 = C1 * (1 - exp(-C2 * Kappa_r_R1)) - C3 * Kappa_r_R1;
	SX miu_com_L2 = C1 * (1 - exp(-C2 * Kappa_r_L2)) - C3 * Kappa_r_L2;
	SX miu_com_R2 = C1 * (1 - exp(-C2 * Kappa_r_R2)) - C3 * Kappa_r_R2;
	// ����������
	SX Fy_L1 = -ks * alphaf * Fz(0) * miu_com_L1 / Kappa_r_L1;
	SX Fy_R1 = -ks * alphaf * Fz(1) * miu_com_R1 / Kappa_r_R1;
	SX Fy_L2 = -ks * alphar * Fz(2) * miu_com_L2 / Kappa_r_L2;
	SX Fy_R2 = -ks * alphar * Fz(3) * miu_com_R2 / Kappa_r_R2;
	// ����������
	SX Fx_L1 = Kappa_L1 * Fz(0) * miu_com_L1 / Kappa_r_L1;
	SX Fx_R1 = Kappa_R1 * Fz(1) * miu_com_R1 / Kappa_r_R1;
	SX Fx_L2 = Kappa_L2 * Fz(2) * miu_com_L2 / Kappa_r_L2;
	SX Fx_R2 = Kappa_R2 * Fz(3) * miu_com_R2 / Kappa_r_R2;
	// ״̬�ռ䷽�̺�������
	dxdt(0) = ((Fx_R1 + Fx_L1) * sin(u_tmp(0)) + (Fy_R1 + Fy_L1) * cos(u_tmp(0)) + (Fy_R2 + Fy_L2)) / (M * x_tmp(6)) - x_tmp(1);
	dxdt(1) = (Lf * ((Fy_R1 + Fy_L1) * cos(u_tmp(0)) + (Fx_R1 + Fx_L1) * sin(u_tmp(0))) - Lr * (Fy_R2 + Fy_L2) + dr * (Fx_R1 * cos(u_tmp(0)) + Fx_R2 - Fy_R1 * sin(u_tmp(0)) + Fy_L1 * sin(u_tmp(0)) - Fx_L1 * cos(u_tmp(0)) - Fx_L2) / 2) / Iz;
	dxdt(2) = (u_tmp(1) * 100 - Re * Fx_R1) / Iw;
	dxdt(3) = (u_tmp(2) * 100 - Re * Fx_L1) / Iw;
	dxdt(4) = (u_tmp(3) * 100 - Re * Fx_R2) / Iw;
	dxdt(5) = (u_tmp(4) * 100 - Re * Fx_L2) / Iw;
	dxdt(6) = ((Fx_R1 + Fx_L1) * cos(u_tmp(0)) - (Fy_R1 + Fy_L1) * sin(u_tmp(0)) + Fx_R2 + Fx_L2) / M + x_tmp(0) * x_tmp(1) * x_tmp(6);

	SX inputs = SX::vertcat({ x_tmp(0) ,x_tmp(1),x_tmp(2),x_tmp(3),x_tmp(4),x_tmp(5),x_tmp(6),u_tmp(0),u_tmp(1),u_tmp(2),u_tmp(3),u_tmp(4) });
	SX outputs = SX::vertcat({ dxdt(0) ,dxdt(1),dxdt(2),dxdt(3),dxdt(4),dxdt(5),dxdt(6) });

	Function  F_dxdt("dxdt", { inputs }, { outputs });


//��ڽ��ٶȸ�˹���̻ع�ģ�ͳ�ʼ��

	int lable_count = 0;
	GP_init_data yawrate_GP_init;
	yawrate_GP_init.alfa = (double*)malloc(sizeof(double) * N * N);
	yawrate_GP_init.L = (double*)malloc(sizeof(double) * N * N);

	GP_init_data slipangle_GP_init;
	slipangle_GP_init.alfa = (double*)malloc(sizeof(double) * N * N);
	slipangle_GP_init.L = (double*)malloc(sizeof(double) * N * N);


	GP_hyperparameters yawrate_GP_hyper;
	yawrate_GP_hyper = { { 1.0, 500, 500, 500, 500, 100, 1.2, 100 } ,{60.6 * 60.6},{0.6 * 0.6} };

	GP_hyperparameters slipangle_GP_hyper;
	slipangle_GP_hyper= { { 1.0, 500, 500, 500, 500, 100, 1.2, 100 } ,{10.6 * 10.6},{0.6 * 0.6} };

	double slipangle_lable_1D_standardized[N] = { 0 };
	double yawrate_lable_1D_standardized[N] = { 0 };
	//printf("%lf", GP_hyper.ell[2]);

	mean_function(feature_data_1D,mean_data_feature, N, feature_dimension, averagebycolumn);
	mean_function(lable_data_1D, mean_data_lable, N, lable_dimension, averagebycolumn);
	std_function(feature_data_1D, mean_data_feature, N, feature_dimension, averagebycolumn, std_data_feature);
	std_function(lable_data_1D, mean_data_lable, N, lable_dimension, averagebycolumn, std_data_lable);

	standardize(feature_data_1D, mean_data_feature, std_data_feature, N, feature_dimension, feature_data_1D_standardized);
	standardize(lable_data_1D, mean_data_lable, std_data_lable, N, lable_dimension, lable_data_1D_standardized);

	for (int i = 0; i < N; i++)
	{
		yawrate_lable_1D_standardized[i] = lable_data_1D_standardized[lable_count];
		lable_count++;
		slipangle_lable_1D_standardized[i] = lable_data_1D_standardized[lable_count];
		lable_count++;
	}
	
	GP_Init(&yawrate_GP_hyper, feature_data_1D_standardized, yawrate_lable_1D_standardized, N, feature_dimension, &yawrate_GP_init);

	SX  z_s_yawrate     = SX::sym("z_s_yawrate", feature_dimension);
	SX  ks_s_yawrate    = SX::sym("ks_s_yawrate", N);
	SX  alpha_s_yawrate = SX::sym("alpha_s_yawrate", N);
	SX  kss_s_yawrate   = SX::sym("kss_s_yawrate");
	SX  ell_s_yawrate   = SX::sym("ell_s_yawrate", feature_dimension);
	SX  sf2_s_yawrate   = SX::sym("sf2_s_yawrate");
	SX  x_s_yawrate     = SX::sym("x_s_yawrate", feature_dimension);
	SX  X_s_yawrate     = SX::sym("X_s_yawrate", N*feature_dimension);

	SX  X_a_yawrate     = SX::zeros(N * feature_dimension);
	SX  alfa_a_yawrate  = SX::zeros(N);
	SX  ell_a_yawrate   = SX::zeros(feature_dimension);
	SX  sf2_a_yawrate   = yawrate_GP_hyper.sf2;
	SX  sn2_a_yawrate   = yawrate_GP_hyper.sn2;
	SX  L_a_yawrate     = SX::zeros(N * N);


	for (int i = 0; i < N * feature_dimension; i++)
	{
		X_a_yawrate(i) = feature_data_1D_standardized[i];
	}
	for (int i = 0; i < N; i++)
	{
		alfa_a_yawrate(i) = yawrate_GP_init.alfa[i];
	}
	for (int i = 0; i < feature_dimension; i++)
	{
		ell_a_yawrate(i) = yawrate_GP_hyper.ell[i];
	}
	for (int i = 0; i < N * N; i++)
	{
		L_a_yawrate(i) = yawrate_GP_init.L[i];
	}

	SX sum_tmp_yawrate=0;
	SX kernel_yawrate = 0;
	for (int i = 0; i < feature_dimension; i++)
	{
			sum_tmp_yawrate = sum_tmp_yawrate + (x_s_yawrate(i)- z_s_yawrate(i))* (x_s_yawrate(i) - z_s_yawrate(i)) / (ell_s_yawrate(i)* ell_s_yawrate(i));
	}
	kernel_yawrate = sf2_s_yawrate * exp(-sum_tmp_yawrate / 2);

	SX kernel_func_yawrate_inputs = SX::vertcat( { x_s_yawrate,z_s_yawrate,ell_s_yawrate,sf2_s_yawrate });
	SX kernel_func_yawrate_outputs = SX::vertcat({ kernel_yawrate });
	Function kernel_func_yawrate("kernel_func_yawrate", { kernel_func_yawrate_inputs }, { kernel_func_yawrate_outputs });

	SX kernel_func_inputs_yawrate_data;
	SXVector kernel_func_outputs_yawrate_data;
	SX X_s_yawrate_input= SX::zeros(feature_dimension);
	SX V_yawrate;

	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < feature_dimension; j++)
		{
			X_s_yawrate_input(j) = X_s_yawrate(i* feature_dimension+j);
		}
		kernel_func_inputs_yawrate_data = SX::vertcat({ X_s_yawrate_input,z_s_yawrate,ell_s_yawrate,sf2_s_yawrate });
		kernel_func_outputs_yawrate_data = kernel_func_yawrate(kernel_func_inputs_yawrate_data);
		ks_s_yawrate(i) = kernel_func_outputs_yawrate_data[0](0);
	}
	

	SX L_a_yawrate_2D = SX::zeros(N, N);
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			L_a_yawrate_2D(i, j) = L_a_yawrate(i * N + j);
		}
	}

	V_yawrate = solve(L_a_yawrate_2D, ks_s_yawrate); //L_a_2D������ת�ã�������Կ��������￴��

	

	SX ks_s_yawrate_T=ks_s_yawrate.T();

	SX yawrate_mean_inputs = SX::vertcat({ X_s_yawrate,z_s_yawrate,ell_s_yawrate,sf2_s_yawrate,alpha_s_yawrate });
	SX yawrate_mean_outputs = SX::vertcat({ mtimes(ks_s_yawrate_T,alpha_s_yawrate)});
	Function yawrate_mean("yawrate_mean", { yawrate_mean_inputs }, { yawrate_mean_outputs });


	SX yawrate_mean_input_a= SX::vertcat({ X_s_yawrate,z_s_yawrate,ell_a_yawrate,sf2_a_yawrate,alpha_s_yawrate });
	SX yawrate_mean_tmp_inputs = SX::vertcat({ X_s_yawrate,z_s_yawrate,alpha_s_yawrate });
	SX yawrate_mean_tmp_outputs = SX::vertcat({ yawrate_mean(yawrate_mean_input_a) });
	Function yawrate_mean_tmp("yawrate_mean_tmp", { yawrate_mean_tmp_inputs }, { yawrate_mean_tmp_outputs });
	//std::cout << yawrate_mean_tmp_outputs << std::endl;

	SX yawrate_mean_tmp_input_a = SX::vertcat({ X_a_yawrate,z_s_yawrate,alfa_a_yawrate });
	SX yawrate_mean_function_inputs = SX::vertcat({ z_s_yawrate });
	SX yawrate_mean_function_outputs = SX::vertcat({ yawrate_mean_tmp(yawrate_mean_tmp_input_a) });
	Function yawrate_mean_function("yawrate_mean_function", { yawrate_mean_function_inputs }, { yawrate_mean_function_outputs });
	//std::cout << yawrate_mean_function_outputs << std::endl;

	SX yawrate_var_inputs = SX::vertcat({ kss_s_yawrate,X_s_yawrate,z_s_yawrate,ell_s_yawrate,sf2_s_yawrate });
	SX yawrate_var_outputs = SX::vertcat({ kss_s_yawrate - mtimes(V_yawrate.T(),V_yawrate) });
	Function yawrate_var("yawrate_var", { yawrate_var_inputs }, { yawrate_var_outputs });

	SX yawrate_var_input_a = SX::vertcat({ kss_s_yawrate,X_s_yawrate,z_s_yawrate,ell_a_yawrate,sf2_a_yawrate });
	SX yawrate_var_tmp_inputs = SX::vertcat({ kss_s_yawrate,X_s_yawrate,z_s_yawrate });
	SX yawrate_var_tmp_outputs = SX::vertcat({ yawrate_var(yawrate_var_input_a) });
	Function yawrate_var_tmp("yawrate_var_tmp", { yawrate_var_tmp_inputs }, { yawrate_var_tmp_outputs });

	SX yawrate_var_tmp_input_a = SX::vertcat({ sf2_a_yawrate,X_a_yawrate,z_s_yawrate });
	SX yawrate_var_function_inputs = SX::vertcat({ z_s_yawrate });
	SX yawrate_var_function_outputs = SX::vertcat({ yawrate_var_tmp(yawrate_var_tmp_input_a) });
	Function yawrate_var_function("yawrate_var_function", { yawrate_var_function_inputs }, { yawrate_var_function_outputs });

	
//���Ĳ�ƫ�Ǹ�˹���̻ع�ģ�ͳ�ʼ��

	GP_Init(&slipangle_GP_hyper, feature_data_1D_standardized, slipangle_lable_1D_standardized, N, feature_dimension, &slipangle_GP_init);

	SX  z_s_slipangle     = SX::sym("z_s_slipangle", feature_dimension);
	SX  ks_s_slipangle    = SX::sym("ks_s_slipangle", N);
	SX  alpha_s_slipangle = SX::sym("alpha_s_slipangle", N);
	SX  kss_s_slipangle   = SX::sym("kss_s_slipangle");
	SX  ell_s_slipangle   = SX::sym("ell_s_slipangle", feature_dimension);
	SX  sf2_s_slipangle   = SX::sym("sf2_s_slipangle");
	SX  x_s_slipangle     = SX::sym("x_s_slipangle", feature_dimension);
	SX  X_s_slipangle     = SX::sym("X_s_slipangle", N * feature_dimension);

	SX  X_a_slipangle     = SX::zeros(N * feature_dimension);
	SX  alfa_a_slipangle  = SX::zeros(N);
	SX  ell_a_slipangle   = SX::zeros(feature_dimension);
	SX  sf2_a_slipangle   = slipangle_GP_hyper.sf2;
	SX  sn2_a_slipangle   = slipangle_GP_hyper.sn2;
	SX  L_a_slipangle     = SX::zeros(N * N);


	for (int i = 0; i < N * feature_dimension; i++)
	{
		X_a_slipangle(i) = feature_data_1D_standardized[i];
	}
	for (int i = 0; i < N; i++)
	{
		alfa_a_slipangle(i) = slipangle_GP_init.alfa[i];
	}
	for (int i = 0; i < feature_dimension; i++)
	{
		ell_a_slipangle(i) = slipangle_GP_hyper.ell[i];
	}
	for (int i = 0; i < N * N; i++)
	{
		L_a_slipangle(i) = slipangle_GP_init.L[i];
	}

	SX sum_tmp_slipangle=0;
	SX kernel_slipangle=0;
	for (int i = 0; i < feature_dimension; i++)
	{
		sum_tmp_slipangle = sum_tmp_slipangle + (x_s_slipangle(i) - z_s_slipangle(i)) * (x_s_slipangle(i) - z_s_slipangle(i)) / (ell_s_slipangle(i) * ell_s_slipangle(i));
	}
	kernel_slipangle = sf2_s_slipangle * exp(-sum_tmp_slipangle / 2);

	SX kernel_func_slipangle_inputs = SX::vertcat({ x_s_slipangle,z_s_slipangle,ell_s_slipangle,sf2_s_slipangle });
	SX kernel_func_slipangle_outputs = SX::vertcat({ kernel_slipangle });
	Function kernel_func_slipangle("kernel_func_slipangle", { kernel_func_slipangle_inputs }, { kernel_func_slipangle_outputs });

	SX kernel_func_inputs_slipangle_data;
	SXVector kernel_func_outputs_slipangle_data;
	SX X_s_slipangle_input = SX::zeros(feature_dimension);
	SX V_slipangle;

	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < feature_dimension; j++)
		{
			X_s_slipangle_input(j) = X_s_slipangle(i * feature_dimension + j);
		}
		kernel_func_inputs_slipangle_data = SX::vertcat({ X_s_slipangle_input,z_s_slipangle,ell_s_slipangle,sf2_s_slipangle });
		kernel_func_outputs_slipangle_data = kernel_func_slipangle(kernel_func_inputs_slipangle_data);
		ks_s_slipangle(i) = kernel_func_outputs_slipangle_data[0](0);
	}

	SX L_a_slipangle_2D = SX::zeros(N, N);
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			L_a_slipangle_2D(i, j) = L_a_slipangle(i * N + j);
		}
	}


	V_slipangle = solve(L_a_slipangle_2D, ks_s_slipangle); //L_a_2D������ת�ã�������Կ��������￴��
	
	SX ks_s_slipangle_T = ks_s_slipangle.T();

	SX slipangle_mean_inputs = SX::vertcat({ X_s_slipangle,z_s_slipangle,ell_s_slipangle,sf2_s_slipangle,alpha_s_slipangle });
	SX slipangle_mean_outputs = SX::vertcat({ mtimes(ks_s_slipangle_T,alpha_s_slipangle) });
	Function slipangle_mean("slipangle_mean", { slipangle_mean_inputs }, { slipangle_mean_outputs });


	SX slipangle_mean_input_a = SX::vertcat({ X_s_slipangle,z_s_slipangle,ell_a_slipangle,sf2_a_slipangle,alpha_s_slipangle });
	SX slipangle_mean_tmp_inputs = SX::vertcat({ X_s_slipangle,z_s_slipangle,alpha_s_slipangle });
	SX slipangle_mean_tmp_outputs = SX::vertcat({ slipangle_mean(slipangle_mean_input_a) });
	Function slipangle_mean_tmp("slipangle_mean_tmp", { slipangle_mean_tmp_inputs }, { slipangle_mean_tmp_outputs });


	SX slipangle_mean_tmp_input_a = SX::vertcat({ X_a_slipangle,z_s_slipangle,alfa_a_slipangle });
	SX slipangle_mean_function_inputs = SX::vertcat({ z_s_slipangle });
	SX slipangle_mean_function_outputs = SX::vertcat({ slipangle_mean_tmp(slipangle_mean_tmp_input_a) });
	Function slipangle_mean_function("slipangle_mean_function", { slipangle_mean_function_inputs }, { slipangle_mean_function_outputs });


	SX slipangle_var_inputs = SX::vertcat({ kss_s_slipangle,X_s_slipangle,z_s_slipangle,ell_s_slipangle,sf2_s_slipangle });
	SX slipangle_var_outputs = SX::vertcat({ kss_s_slipangle - mtimes(V_slipangle.T(),V_slipangle) });
	Function slipangle_var("slipangle_var", { slipangle_var_inputs }, { slipangle_var_outputs });

	SX slipangle_var_input_a = SX::vertcat({ kss_s_slipangle,X_s_slipangle,z_s_slipangle,ell_a_slipangle,sf2_a_slipangle });
	SX slipangle_var_tmp_inputs = SX::vertcat({ kss_s_slipangle,X_s_slipangle,z_s_slipangle });
	SX slipangle_var_tmp_outputs = SX::vertcat({ slipangle_var(slipangle_var_input_a) });
	Function slipangle_var_tmp("slipangle_var_tmp", { slipangle_var_tmp_inputs }, { slipangle_var_tmp_outputs });

	SX slipangle_var_tmp_input_a = SX::vertcat({ sf2_a_slipangle,X_a_slipangle,z_s_slipangle });
	SX slipangle_var_function_inputs = SX::vertcat({ z_s_slipangle });
	SX slipangle_var_function_outputs = SX::vertcat({ slipangle_var_tmp(slipangle_var_tmp_input_a) });
	Function slipangle_var_function("slipangle_var_function", { slipangle_var_function_inputs }, { slipangle_var_function_outputs });


//���������״̬�ռ䷽��
	SX inputValues;
	SX GP_input;
	SXVector yawrate_error_mean_model, yawrate_error_std_model, slipangle_error_mean_model, slipangle_error_std_model;
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		if (i == 0)
		{
			inputValues = SX::vertcat({ ISV(0), ISV(1),ISV(2),ISV(3),ISV(4),ISV(5),ISV(6), CV(0,0), CV(0,1), CV(0,2), CV(0,3), CV(0,4) });

			SXVector outputValues = F_dxdt(inputValues);
			//std::cout << outputValues[0](1).size() << std::endl;
			GP_input= SX::vertcat({ CV(0,0), CV(0,1)*100, CV(0,2) * 100, CV(0,3) * 100, CV(0,4) * 100, ISV(0), ISV(1),ISV(6)});
			for (int j = 0; j < feature_dimension; j++)
			{
				GP_input(j) = (GP_input(j) - mean_data_feature[j]) / std_data_feature[j];
			}
			yawrate_error_mean_model=yawrate_mean_function(GP_input);
			yawrate_error_std_model= yawrate_var_function(GP_input);

			slipangle_error_mean_model= slipangle_mean_function(GP_input);
			slipangle_error_std_model= slipangle_var_function(GP_input);

			yawrate_error_mean_model[0](0) = yawrate_error_mean_model[0](0) * std_data_lable[0] + mean_data_lable[0];
			yawrate_error_std_model[0](0) = sqrt(yawrate_error_std_model[0](0)) * std_data_lable[0];

			slipangle_error_mean_model[0](0) = slipangle_error_mean_model[0](0)* std_data_lable[1] + mean_data_lable[1];
			slipangle_error_std_model[0](0)= sqrt(slipangle_error_std_model[0](0))* std_data_lable[1];

			con_eq(i, 0) = SV(i, 0) - ISV(0) - outputValues[0](0) * Ts- slipangle_error_mean_model[0](0);
			con_eq(i, 1) = SV(i, 1) - ISV(1) - outputValues[0](1) * Ts- yawrate_error_mean_model[0](0);
			con_eq(i, 2) = SV(i, 2) - ISV(2) - outputValues[0](2) * Ts;
			con_eq(i, 3) = SV(i, 3) - ISV(3) - outputValues[0](3) * Ts;
			con_eq(i, 4) = SV(i, 4) - ISV(4) - outputValues[0](4) * Ts;
			con_eq(i, 5) = SV(i, 5) - ISV(5) - outputValues[0](5) * Ts;
			con_eq(i, 6) = SV(i, 6) - ISV(6) - outputValues[0](6) * Ts;

			Tt_eq(i, 0) = CV(i, 1) + CV(i, 2) + CV(i, 3) + CV(i, 4) - Tt;
		}
		else
		{
			if (i >= Control_Horizon)
			{
				inputValues = SX::vertcat({ SV(i - 1,0), SV(i - 1,1),SV(i - 1,2),SV(i - 1,3),SV(i - 1,4),SV(i - 1,5),SV(i - 1,6), CV(Control_Horizon-1,0), CV(Control_Horizon - 1,1), CV(Control_Horizon - 1,2), CV(Control_Horizon - 1,3), CV(Control_Horizon - 1,4) });
				SXVector outputValues = F_dxdt(inputValues);

				GP_input = SX::vertcat({ CV(Control_Horizon - 1,0), CV(Control_Horizon - 1,1)*100, CV(Control_Horizon - 1,2) * 100, CV(Control_Horizon - 1,3) * 100, CV(Control_Horizon - 1,4) * 100, SV(i-1,0), SV(i - 1,1),SV(i - 1,6) });
				for (int j = 0; j < feature_dimension; j++)
				{
					GP_input(j) = (GP_input(j) - mean_data_feature[j]) / std_data_feature[j];
				}
				yawrate_error_mean_model = yawrate_mean_function(GP_input);
				yawrate_error_std_model = yawrate_var_function(GP_input);

				slipangle_error_mean_model = slipangle_mean_function(GP_input);
				slipangle_error_std_model = slipangle_var_function(GP_input);



				yawrate_error_mean_model[0](0) = yawrate_error_mean_model[0](0) * std_data_lable[0] + mean_data_lable[0];
				yawrate_error_std_model[0](0) = sqrt(yawrate_error_std_model[0](0)) * std_data_lable[0];

				slipangle_error_mean_model[0](0) = slipangle_error_mean_model[0](0) * std_data_lable[1] + mean_data_lable[1];
				slipangle_error_std_model[0](0) = sqrt(slipangle_error_std_model[0](0)) * std_data_lable[1];

				con_eq(i, 0) = SV(i, 0) - SV(i - 1, 0) - outputValues[0](0) * Ts - slipangle_error_mean_model[0](0);
				con_eq(i, 1) = SV(i, 1) - SV(i - 1, 1) - outputValues[0](1) * Ts - yawrate_error_mean_model[0](0);
				con_eq(i, 2) = SV(i, 2) - SV(i - 1, 2) - outputValues[0](2) * Ts;
				con_eq(i, 3) = SV(i, 3) - SV(i - 1, 3) - outputValues[0](3) * Ts;
				con_eq(i, 4) = SV(i, 4) - SV(i - 1, 4) - outputValues[0](4) * Ts;
				con_eq(i, 5) = SV(i, 5) - SV(i - 1, 5) - outputValues[0](5) * Ts;
				con_eq(i, 6) = SV(i, 6) - SV(i - 1, 6) - outputValues[0](6) * Ts;
			}
			else
			{
				inputValues = SX::vertcat({ SV(i - 1,0), SV(i - 1,1),SV(i - 1,2),SV(i - 1,3),SV(i - 1,4),SV(i - 1,5),SV(i - 1,6), CV(i,0), CV(i,1), CV(i,2), CV(i,3), CV(i,4) });
				SXVector outputValues = F_dxdt(inputValues);

				GP_input = SX::vertcat({ CV(i,0), CV(i,1) * 100, CV(i,2) * 100, CV(i,3) * 100, CV(i,4) * 100, SV(i - 1,0), SV(i - 1,1),SV(i - 1,6) });
				for (int j = 0; j < feature_dimension; j++)
				{
					GP_input(j) = (GP_input(j) - mean_data_feature[j]) / std_data_feature[j];
				}
				yawrate_error_mean_model = yawrate_mean_function(GP_input);
				yawrate_error_std_model = yawrate_var_function(GP_input);

				slipangle_error_mean_model = slipangle_mean_function(GP_input);
				slipangle_error_std_model = slipangle_var_function(GP_input);


				yawrate_error_mean_model[0](0) = yawrate_error_mean_model[0](0) * std_data_lable[0] + mean_data_lable[0];
				yawrate_error_std_model[0](0) = sqrt(yawrate_error_std_model[0](0)) * std_data_lable[0];

				slipangle_error_mean_model[0](0) = slipangle_error_mean_model[0](0) * std_data_lable[1] + mean_data_lable[1];
				slipangle_error_std_model[0](0) = sqrt(slipangle_error_std_model[0](0)) * std_data_lable[1];

				con_eq(i, 0) = SV(i, 0) - SV(i - 1, 0) - outputValues[0](0) * Ts - slipangle_error_mean_model[0](0);
				con_eq(i, 1) = SV(i, 1) - SV(i - 1, 1) - outputValues[0](1) * Ts - yawrate_error_mean_model[0](0);
				con_eq(i, 2) = SV(i, 2) - SV(i - 1, 2) - outputValues[0](2) * Ts;
				con_eq(i, 3) = SV(i, 3) - SV(i - 1, 3) - outputValues[0](3) * Ts;
				con_eq(i, 4) = SV(i, 4) - SV(i - 1, 4) - outputValues[0](4) * Ts;
				con_eq(i, 5) = SV(i, 5) - SV(i - 1, 5) - outputValues[0](5) * Ts;
				con_eq(i, 6) = SV(i, 6) - SV(i - 1, 6) - outputValues[0](6) * Ts;
				Tt_eq(i, 0) = CV(i, 1) + CV(i, 2) + CV(i, 3) + CV(i, 4) - Tt;
			}
		}
	}


	for (int i = 0; i < Predictive_Horizon; i++)
	{
		sum_objective = sum_objective + 1 * SV(i, 0) * SV(i, 0) + 2 * (SV(i, 1) - Setpoint(i)) * (SV(i, 1) - Setpoint(i));

	}
	for (int i = 0; i < Control_Horizon; i++)
	{
		if (i == 0)
		{
			sum_delta_u = sum_delta_u + 10 *(CV(i, 0) - ICV(0))* (CV(i, 0) - ICV(0))+0.01*((CV(i, 1) - ICV(1)) * (CV(i, 1) - ICV(1))+ (CV(i, 2) - ICV(2)) * (CV(i, 2) - ICV(2))+(CV(i, 3) - ICV(3)) * (CV(i, 3) - ICV(3))+(CV(i, 4) - ICV(4)) * (CV(i, 4) - ICV(4)));
		}
		else
		{
			sum_delta_u = sum_delta_u + 10 * (CV(i, 0) - CV(i-1,0)) * (CV(i, 0) - CV(i-1,0)) + 0.01 * ((CV(i, 1) - CV(i - 1,1)) * (CV(i, 1) - CV(i - 1, 1)) + (CV(i, 2) - CV(i - 1, 2)) * (CV(i, 2) - CV(i - 1, 2)) + (CV(i, 3) - CV(i - 1, 3)) * (CV(i, 3) - CV(i - 1, 3)) + (CV(i, 4) - CV(i - 1, 4)) * (CV(i, 4) - CV(i - 1, 4)));
		}
	}
	cost = 1 * sum_objective + 2 * sum_delta_u;

	int h = 0;
	for(int i=0;i< Control_Horizon;i++)
		for (int j = 0; j < Control_dimension; j++)
		{
			CV_1D(h) = CV(i, j);
			h = h + 1;
		}
	h = 0;
	for (int i = 0; i < Predictive_Horizon; i++)
		for (int j = 0; j < state_dimension; j++)
		{
			SV_1D(h) = SV(i, j);
			h = h + 1;
		}
	
	SX con_eq_1D = SX::reshape(con_eq,-1,1);
	SX Tt_eq_1D = SX::reshape(Tt_eq, -1, 1);
	SX cons_all = SX::vertcat({ con_eq_1D ,Tt_eq_1D });
	SX PV = SX::vertcat({ISV,ICV,Setpoint,Fz});
	SX DV = SX::vertcat({ CV_1D ,SV_1D });
	//std::cout << PV.size() << std::endl;
	SX CC = cons_all;
	SX CF = cost;
	SXDict nlp = { { "x", DV }, { "p", PV }, { "f",CF },{ "g",CC } };
	
	//std::cout << CC(0) << std::endl;
	Dict ipoptoptions = { {"print_level",0},{"print_timing_statistics","no"},{"max_iter",100},{"mu_init",0.01},{"tol",1e-8},{"mu_strategy","adaptive"},{"warm_start_init_point","yes"} };
	Dict options = { {"expand",1},{"print_time",0},{"ipopt",ipoptoptions}};
	Function solver = nlpsol("solver", "ipopt", nlp, options);
	// ������������


	std::map<std::string, DM> arg, res;

	std::vector<double> lbu_initial = { -0.1,0,0,0,0};
	std::vector<double> ubu_initial = { 0.1,2,2,2,2};
	std::vector<double> lbx_initial = { -100,-100,-100,-100,-100,-100,-100};
	std::vector<double> ubx_initial = { 100,100,100,100,100,100,100};
	std::vector<double> lbg_initial = { 0 };
	std::vector<double> ubg_initial = { 0 };

	std::vector<double> lbu;
	std::vector<double> ubu;
	std::vector<double> lbx;
	std::vector<double> ubx;
	std::vector<double> lbg;
	std::vector<double> ubg;

	for (int i = 0; i < Control_Horizon; i++)
	{
		lbu.insert(lbu.end(), lbu_initial.begin(), lbu_initial.end());
		ubu.insert(ubu.end(), ubu_initial.begin(), ubu_initial.end());
	}
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		lbx.insert(lbx.end(), lbx_initial.begin(), lbx_initial.end());
		ubx.insert(ubx.end(), ubx_initial.begin(), ubx_initial.end());
	}
	for (int i = 0; i < state_dimension * Predictive_Horizon + Control_Horizon; i++)
	{
		lbg.insert(lbg.end(), lbg_initial.begin(), lbg_initial.end());
		ubg.insert(ubg.end(), ubg_initial.begin(), ubg_initial.end());
	}

	std::vector<double> lbux;
	std::vector<double> ubux;
	lbux.insert(lbux.end(), lbu.begin(), lbu.end());
	lbux.insert(lbux.end(), lbx.begin(), lbx.end());
	ubux.insert(ubux.end(), ubu.begin(), ubu.end());
	ubux.insert(ubux.end(), ubx.begin(), ubx.end());

	double* Setpoint1 = (double*)malloc(Predictive_Horizon * sizeof(double));


	std::vector<double> p0;

	std::vector<double> Solver_ISV;
	
	std::vector<double> Solver_ICV = u0_initial;

	std::vector<double> Solver_Setpoint(Predictive_Horizon);
	std::vector<double> Solver_output, Solver_state;
	std::vector<double> x_initial = IDV;
	std::vector<double> Solver_Fz;

	for(int i=0;i<N;i++)
	{
		Solver_ISV = { beta0[timescale],gamma0[timescale],wfl[timescale],wfr[timescale], wrl[timescale], wrr[timescale], Vx0[timescale]};
		Solver_Fz = {Fzfl[timescale],Fzfr[timescale],Fzrl[timescale],Fzrr[timescale] };

		reference_generation(Vx0[timescale], Setpoint1);

		for (int i = 0; i < Predictive_Horizon; i++)
		{
			Solver_Setpoint[i] = Setpoint1[i];
		}

		p0.insert(p0.end(), Solver_ISV.begin(), Solver_ISV.end());
		p0.insert(p0.end(), Solver_ICV.begin(), Solver_ICV.end());
		p0.insert(p0.end(), Solver_Setpoint.begin(), Solver_Setpoint.end());
		p0.insert(p0.end(), Solver_Fz.begin(),Solver_Fz.end());
		

		arg["lbx"] = lbux;
		arg["ubx"] = ubux;
		arg["lbg"] = lbg;
		arg["ubg"] = ubg;
		arg["x0"] = x_initial;
		arg["p"] = p0;
		start = clock();
		res=solver(arg);
		end = clock();
		
		std::cout << "����ʱ��" << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
		//std::cout << "Optimal solution for p = " << arg.at("p") << ":" << std::endl;
		//std::cout << std::setw(30) << "Primal solution: " << res.at("x") << std::endl;
		Solver_state = (std::vector<double>)res.at("x");
		delta0[timescale] = Solver_state[0];
		Tfl[timescale] = Solver_state[1];
		Tfr[timescale] = Solver_state[2];
		Trl[timescale] = Solver_state[3];
		Trr[timescale] = Solver_state[4];
		Solver_ICV = { Solver_state[0] ,Solver_state[1] ,Solver_state[2] ,Solver_state[3] ,Solver_state[4] };
		x_initial = Solver_state;
		std::cout << std::setw(30) << "output: " << Solver_state[0] << Solver_state[1] << Solver_state[2]  << Solver_state[3] << Solver_state[4] << std::endl;

		p0.clear();
		timescale++;
	}

	Write_txt_file(delta_filename,delta0);
	Write_txt_file(Tfl_filename, Tfl);
	Write_txt_file(Tfr_filename, Tfr);
	Write_txt_file(Trl_filename, Trl);
	Write_txt_file(Trr_filename, Trr);

	return 0;
}
