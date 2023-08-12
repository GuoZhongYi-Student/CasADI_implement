#include <iostream>
#include <vector>
#include <iomanip>
#include <time.h>
#include "casadi/casadi.hpp"

#define Predictive_Horizon 10
#define Control_Horizon 10
#define Control_dimension 5
#define state_dimension 2
#define N 451

using namespace casadi;
int timescale = 0;
int reference_generation(double Vx, double* yawrate_r)
{
	double Lf = 1.05;
	double Lr = 1.61;
	double Kf_FAI = 43000;
	double Kr_FAI = 28662;
	double L = Lf + Lr;
	double M = 1572;
	double miu = 0.8;
	double Kv = (Lr * M) / (Kf_FAI * L) - (Lf * M) / (Kr_FAI * L); //转向不足梯度
	std::vector<double> refer_steerangle = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00200000000000000, 0.00400000000000000, 0.00600000000000001, 0.00800000000000001, 0.0100000000000000, 0.0120000000000000, 0.0140000000000000, 0.0160000000000000, 0.0180000000000000, 0.0200000000000000, 0.0220000000000000, 0.0240000000000000, 0.0260000000000000, 0.0280000000000000, 0.0300000000000000, 0.0320000000000000, 0.0340000000000000, 0.0360000000000000, 0.0380000000000000, 0.0400000000000000, 0.0420000000000000, 0.0440000000000000, 0.0460000000000000, 0.0480000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0486666666666667, 0.0473333333333333, 0.0460000000000000, 0.0446666666666667, 0.0433333333333333, 0.0420000000000000, 0.0406666666666667, 0.0393333333333333, 0.0380000000000000, 0.0366666666666667, 0.0353333333333333, 0.0340000000000000, 0.0326666666666667, 0.0313333333333333, 0.0300000000000000, 0.0286666666666667, 0.0273333333333334, 0.0260000000000000, 0.0246666666666667, 0.0233333333333333, 0.0220000000000000, 0.0206666666666667, 0.0193333333333333, 0.0180000000000000, 0.0166666666666667, 0.0153333333333333, 0.0140000000000000, 0.0126666666666667, 0.0113333333333333, 0.0100000000000000, 0.00866666666666666, 0.00733333333333332, 0.00599999999999999, 0.00466666666666665, 0.00333333333333332, 0.00199999999999999, 0.000666666666666649, -0.000666666666666683, -0.00200000000000002, -0.00333333333333334, -0.00466666666666666, -0.00599999999999999, -0.00733333333333332, -0.00866666666666667, -0.00999999999999999, -0.0113333333333333, -0.0126666666666667, -0.0140000000000000, -0.0153333333333333, -0.0166666666666667, -0.0180000000000000, -0.0193333333333333, -0.0206666666666667, -0.0220000000000000, -0.0233333333333333, -0.0246666666666667, -0.0260000000000000, -0.0273333333333334, -0.0286666666666667, -0.0300000000000000, -0.0313333333333334, -0.0326666666666667, -0.0340000000000000, -0.0353333333333334, -0.0366666666666667, -0.0380000000000000, -0.0393333333333333, -0.0406666666666667, -0.0420000000000000, -0.0433333333333333, -0.0446666666666667, -0.0460000000000000, -0.0473333333333333, -0.0486666666666667, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0480000000000000, -0.0460000000000000, -0.0440000000000000, -0.0420000000000000, -0.0400000000000000, -0.0380000000000000, -0.0360000000000000, -0.0340000000000000, -0.0320000000000000, -0.0300000000000000, -0.0280000000000000, -0.0260000000000000, -0.0240000000000000, -0.0220000000000000, -0.0200000000000000, -0.0180000000000000, -0.0160000000000000, -0.0140000000000000, -0.0120000000000000, -0.00999999999999997, -0.00800000000000001, -0.00599999999999996, -0.00400000000000000, -0.00199999999999996, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.00200000000000005, -0.00400000000000000, -0.00600000000000005, -0.00800000000000001, -0.0100000000000001, -0.0120000000000000, -0.0140000000000000, -0.0160000000000000, -0.0180000000000000, -0.0200000000000000, -0.0220000000000000, -0.0240000000000000, -0.0260000000000000, -0.0280000000000000, -0.0300000000000000, -0.0320000000000000, -0.0340000000000000, -0.0360000000000000, -0.0380000000000000, -0.0400000000000000, -0.0420000000000000, -0.0440000000000000, -0.0460000000000000, -0.0480000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0500000000000000, -0.0486666666666666, -0.0473333333333333, -0.0460000000000000, -0.0446666666666667, -0.0433333333333333, -0.0420000000000000, -0.0406666666666666, -0.0393333333333333, -0.0380000000000000, -0.0366666666666667, -0.0353333333333334, -0.0340000000000000, -0.0326666666666667, -0.0313333333333333, -0.0300000000000000, -0.0286666666666667, -0.0273333333333334, -0.0260000000000000, -0.0246666666666667, -0.0233333333333333, -0.0220000000000000, -0.0206666666666666, -0.0193333333333333, -0.0180000000000000, -0.0166666666666667, -0.0153333333333333, -0.0140000000000000, -0.0126666666666666, -0.0113333333333333, -0.00999999999999997, -0.00866666666666666, -0.00733333333333329, -0.00599999999999999, -0.00466666666666669, -0.00333333333333332, -0.00200000000000002, -0.000666666666666649, 0.000666666666666659, 0.00200000000000002, 0.00333333333333332, 0.00466666666666669, 0.00599999999999999, 0.00733333333333335, 0.00866666666666667, 0.0100000000000000, 0.0113333333333333, 0.0126666666666667, 0.0140000000000000, 0.0153333333333334, 0.0166666666666667, 0.0180000000000000, 0.0193333333333333, 0.0206666666666667, 0.0220000000000000, 0.0233333333333334, 0.0246666666666667, 0.0260000000000000, 0.0273333333333334, 0.0286666666666667, 0.0300000000000000, 0.0313333333333333, 0.0326666666666667, 0.0340000000000000, 0.0353333333333334, 0.0366666666666667, 0.0380000000000000, 0.0393333333333333, 0.0406666666666667, 0.0420000000000000, 0.0433333333333334, 0.0446666666666667, 0.0460000000000000, 0.0473333333333333, 0.0486666666666667, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0500000000000000, 0.0480000000000000, 0.0459999999999999, 0.0440000000000000, 0.0420000000000000, 0.0400000000000000, 0.0379999999999999, 0.0359999999999999, 0.0340000000000000, 0.0320000000000000, 0.0300000000000001, 0.0279999999999999, 0.0260000000000000, 0.0240000000000000, 0.0220000000000001, 0.0199999999999999, 0.0180000000000000, 0.0160000000000000, 0.0140000000000001, 0.0119999999999999, 0.00999999999999997, 0.00800000000000001, 0.00600000000000005, 0.00399999999999992, 0.00199999999999996, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	//std::cout << refer_steerangle.size() << std::endl;

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


int main() {

	clock_t start, end;
	double Re = 0.325;
	double Lf = 1.05;
	double Lr = 1.61;
	double Iz = 2059.2;
	double Cf = 52000;
	double Cr = 43662;
	double dr = 1.55;
	double Iw = 22.5;
	double Tt = 3.6;
	double Ts = 0.02;
	double M = 1572;
	double Vx0[N], beta0[N], gamma0[N],delta0[N],Tfl[N], Tfr[N], Trl[N], Trr[N];

	FILE* fpRead_vx = fopen("vx.txt", "r");
	if (fpRead_vx == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_vx, "%lf", &Vx0[i]);
	}
	fclose(fpRead_vx);

	FILE* fpRead_gama0 = fopen("yawrate.txt", "r");
	if (fpRead_gama0 == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_gama0, "%lf", &gamma0[i]);
	}
	fclose(fpRead_gama0);

	FILE* fpRead_beta0 = fopen("slipangle.txt", "r");
	if (fpRead_beta0 == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fscanf(fpRead_beta0, "%lf", &beta0[i]);
	}
	fclose(fpRead_beta0);


	SX CV_1D = SX::sym("CV_1D", Control_Horizon * Control_dimension);
	SX SV_1D = SX::sym("SV_1D", Predictive_Horizon * state_dimension);
	SX x_tmp = SX::sym("SV_tmp", state_dimension);
	SX u_tmp = SX::sym("CV_tmp", Control_dimension);
	SX dxdt = SX::sym("dxdt", state_dimension);
	SX Vx = SX::sym("Vx");
	SX CV = SX::sym("control variable", Control_Horizon, Control_dimension);
	SX SV = SX::sym("state variable", Predictive_Horizon, state_dimension);
	SX ISV = SX::sym("SV_initial", state_dimension);
	SX ICV = SX::sym("control variable", Control_dimension);
	SX Setpoint = SX::sym("Setpoint", Predictive_Horizon);
	SX con_eq = SX::sym("constraints_eq", Predictive_Horizon, state_dimension);
	SX Tt_eq = SX::sym("Tt_eq", Predictive_Horizon, 1);
	SX sum_objective = 0 , sum_delta_u=0, cost=0;
	double u0[5][Control_Horizon],x0[2][Predictive_Horizon];


	SX alphaf = x_tmp(0) + (Lf / Vx) * x_tmp(1) - u_tmp(0) + 0.0000001;
	SX alphar = x_tmp(0) - (Lr / Vx) * x_tmp(1) + 0.0000001;

	SX Fy_F = -Cf * alphaf;
	SX Fy_R = -Cr * alphar;



	dxdt(0) = (2 * Fy_F + 2 * Fy_R) / (M * Vx) - x_tmp(1);
	dxdt(1) = (2 * Lf * Fy_F - 2 * Lr * Fy_R + dr * (-u_tmp(1) - u_tmp(3) + u_tmp(2) + u_tmp(4)) * 100 / 2 / Re) / Iz;

	SX inputs = SX::vertcat({ x_tmp(0) ,x_tmp(1),u_tmp(0),u_tmp(1),u_tmp(2),u_tmp(3),u_tmp(4) });
	SX outputs = SX::vertcat({ dxdt(0) ,dxdt(1) });

	Function F_dxdt("Function", { inputs }, { outputs });




	SX inputValues;
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		if (i == 0)
		{
			inputValues = SX::vertcat({ ISV(0), ISV(1), CV(0,0), CV(0,1), CV(0,2), CV(0,3), CV(0,4) });

			SXVector outputValues =F_dxdt(inputValues);
			//std::cout << outputValues[0](1).size() << std::endl;
			con_eq(i, 0) = SV(i, 0) - ISV(0) - outputValues[0](0) * Ts;
			con_eq(i, 1) = SV(i, 1) - ISV(1) - outputValues[0](1) * Ts;
			Tt_eq(i, 0) = CV(i, 1) + CV(i, 2) + CV(i, 3) + CV(i, 4) - Tt;
		}
		else
		{
			inputValues = SX::vertcat({ SV(i - 1,0), SV(i - 1,1), CV(i,0), CV(i,1), CV(i,2), CV(i,3), CV(i,4) });
			SXVector outputValues = F_dxdt(inputValues);
			con_eq(i, 0) = SV(i, 0) - SV(i-1,0) - outputValues[0](0) * Ts;
			con_eq(i, 1) = SV(i, 1) - SV(i-1,1) - outputValues[0](1) * Ts;
			Tt_eq(i, 0) = CV(i, 1) + CV(i, 2) + CV(i, 3) + CV(i, 4) - Tt;
		}
	}

	for (int i = 0; i < Predictive_Horizon; i++)
	{
		sum_objective = sum_objective + 1 * SV(i, 0) * SV(i, 0) + 2 * (SV(i, 1) - Setpoint(i)) * (SV(i, 1) - Setpoint(i));

	}
	for (int i = 0; i < Predictive_Horizon; i++)
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
	cost = 10 * sum_objective + 10 * sum_delta_u;
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
	SX PV = SX::vertcat({ISV,ICV,Setpoint,Vx});
	SX DV = SX::vertcat({ CV_1D ,SV_1D });
	//std::cout << PV.size() << std::endl;
	SX CC = cons_all;
	SX CF = cost;
	SXDict nlp = { { "x", DV }, { "p", PV }, { "f",CF },{ "g",CC } };
	
	
	Dict ipoptoptions = { {"print_level",0},{"print_timing_statistics","no"},{"max_iter",100},{"mu_init",0.01},{"tol",1e-8},{"warm_start_init_point","yes"},{"mu_strategy","adaptive"} };
	Dict options = { {"expand",1},{"print_time",0},{"ipopt",ipoptoptions}};
	Function solver = nlpsol("solver", "ipopt", nlp, options);
	std::map<std::string, DM> arg, res;

	std::vector<double> lbu = { -0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0,-0.1,0,0,0,0 };
	std::vector<double> ubu = { 0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2,0.1,2,2,2,2 };
	std::vector<double> lbx = { -100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100 };
	std::vector<double> ubx = { 100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100 };
	std::vector<double> lbg = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	std::vector<double> ubg = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	std::vector<double> lbux;
	std::vector<double> ubux;
	lbux.insert(lbux.end(), lbu.begin(), lbu.end());
	lbux.insert(lbux.end(), lbx.begin(), lbx.end());
	ubux.insert(ubux.end(), ubu.begin(), ubu.end());
	ubux.insert(ubux.end(), ubx.begin(), ubx.end());
	//std::cout << lbux.size() << std::endl;
	//std::cout << ubux.size() << std::endl;
	double* Setpoint1 = (double*)malloc(Predictive_Horizon * sizeof(double));


	std::vector<double> p0;

	std::vector<double> Solver_ISV;
	
	std::vector<double> Solver_ICV = { 0,0.9,0.9,0.9,0.9 };

	std::vector<double> Solver_Setpoint(Predictive_Horizon);
	std::vector<double> Solver_output, Solver_state;
	double Solver_Vx;
	std::vector<double> x_initial = { 0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0,0.9,0.9,0.9,0.9,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001,0.000000000000001 };

	for(int i=0;i<N;i++)
	{

		Solver_Vx = Vx0[timescale];
		Solver_ISV = { beta0[timescale],gamma0[timescale]};

		reference_generation(Solver_Vx, Setpoint1);

		for (int i = 0; i < Predictive_Horizon; i++)
		{
			Solver_Setpoint[i] = Setpoint1[i];
		}

		p0.insert(p0.end(), Solver_ISV.begin(), Solver_ISV.end());
		p0.insert(p0.end(), Solver_ICV.begin(), Solver_ICV.end());
		p0.insert(p0.end(), Solver_Setpoint.begin(), Solver_Setpoint.end());
		p0.insert(p0.end(), Solver_Vx);
		

		

		arg["lbx"] = lbux;
		arg["ubx"] = ubux;
		arg["lbg"] = lbg;
		arg["ubg"] = ubg;
		arg["x0"] = x_initial;
		arg["p"] = p0;
		start = clock();
		res=solver(arg);
		end = clock();
		
		std::cout << "运行时间" << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
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
	FILE* fpWrite_delta = fopen("delta.txt", "w");
	if (fpWrite_delta == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fprintf(fpWrite_delta, "%f\n", delta0[i]);
	}
	fclose(fpWrite_delta);

	FILE* fpWrite_Tfl = fopen("Tfl.txt", "w");
	if (fpWrite_Tfl == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fprintf(fpWrite_Tfl, "%f\n", Tfl[i]);
	}
	fclose(fpWrite_Tfl);

	FILE* fpWrite_Tfr = fopen("Tfr.txt", "w");
	if (fpWrite_Tfr == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fprintf(fpWrite_Tfr, "%f\n", Tfr[i]);
	}
	fclose(fpWrite_Tfr);

	FILE* fpWrite_Trl = fopen("Trl.txt", "w");
	if (fpWrite_Trl == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fprintf(fpWrite_Trl, "%f\n", Trl[i]);
	}
	fclose(fpWrite_Trl);

	FILE* fpWrite_Trr = fopen("Trr.txt", "w");
	if (fpWrite_Trr == NULL)
	{
		return 0;
	}
	for (int i = 0; i < N; i++)
	{
		fprintf(fpWrite_Trr, "%f\n", Trr[i]);
	}
	fclose(fpWrite_Trr);


	return 0;
}