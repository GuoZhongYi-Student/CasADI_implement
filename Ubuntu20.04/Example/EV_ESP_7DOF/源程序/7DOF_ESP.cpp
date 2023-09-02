#include <iostream>
#include <vector>
#include <iomanip>
#include <time.h>
#include "casadi/casadi.hpp"

#define Predictive_Horizon 10
#define Control_Horizon 10
#define Control_dimension 5
#define state_dimension 7
#define N 451


using namespace casadi;
int timescale = 0;
double refer_steerangle[N+20]; //参考值应该比实际值多出至少预测时域个

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
	double	Re = 0.325;					// 轮胎半径(单位:米)
	double	M = 1572;					// 汽车总质量(单位:千克)
	double	Lf = 1.05;					// 汽车质心到前轴的距离(单位:米)
	double	Lr = 1.61;					// 汽车质心到后轴的距离(单位:米)
	double	Iz = 2059.2;				// 整车绕车辆坐标系z轴的转动惯量(单位:)
	double	Cf = 43000;					// 汽车前轴的侧倾角刚度(单位:)
	double	Cr = 28662;					// 汽车后轴的侧倾角刚度(单位:)
	double	dr = 1.55;					// 左右轮距
	double	Iw = 22.5;					// 轮胎转动惯量
	double	Tt = 3.6;					// 总驱动力矩
	// Controller Parameter
	double	Ts = 0.02;					// 采样时间(单位:秒)
	double	Tire_number = 4;			// 轮胎数量

	// Burckhardt Tire Model Parameter Miu = 0.8
	double	C1 = 1.23;
	double  C2 = 24;
	double  C3 = 2.8;
	double  ks = 0.85;


	double Vx0[N], beta0[N], gamma0[N], delta0[N], Tfl[N], Tfr[N], Trl[N], Trr[N], wfl[N],  wfr[N], wrl[N], wrr[N];
	double Fzfl[N], Fzfr[N], Fzrl[N], Fzrr[N];
	
	//输入数据文件
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

	//输出数据文件
	char delta_filename[] = "delta.txt";
	char Tfl_filename[] = "Tfl.txt";
	char Tfr_filename[] = "Tfr.txt";
	char Trl_filename[] = "Trl.txt";
	char Trr_filename[] = "Trr.txt";


	Read_txt_file(vx_filename, Vx0);
	Read_txt_file(yawrate_filename, gamma0);
	Read_txt_file(slipangle_filename, beta0);
	Read_txt_file(wfl_filename, wfl);
	Read_txt_file(wfr_filename, wfr);
	Read_txt_file(wrl_filename, wrl);
	Read_txt_file(wrr_filename, wrr);
	Read_txt_file(Fzfl_filename, Fzfl);
	Read_txt_file(Fzfr_filename, Fzfr);
	Read_txt_file(Fzrl_filename, Fzrl);
	Read_txt_file(Fzrr_filename, Fzrr);
	Read_txt_file(reference_filename, refer_steerangle);


	//定义中间变量
	/*double alphaf, alphar;									//左右轮侧偏角
	double speed_L1, speed_R1, speed_L2, speed_R2;			//轮心速度
	double Kappa_L1, Kappa_R1, Kappa_L2, Kappa_R2; 
	double Kappa_r_L1, Kappa_r_R1, Kappa_r_L2, Kappa_r_R2;	//综合滑移率
	double miu_com_L1, miu_com_R1, miu_com_L2, miu_com_R2;	//综合附着率
	double Fy_L1, Fy_R1, Fy_L2, Fy_R2;						//侧向力
	double Fx_L1, Fx_R1, Fx_L2, Fx_R2;						//纵向力
	double dxdt[state_dimension] = { 0 };					//状态变化率*/


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
	SX  Tt_eq = SX::sym("Tt_eq", Predictive_Horizon, 1);
	SX  sum_objective = 0, sum_delta_u = 0, cost = 0;

	// Multiple Shooting initial value
	std::vector<double>	u0_initial = { 0, 0.9, 0.9, 0.9, 0.9 };
	std::vector<double>	u0;
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		u0.insert(u0.end(), u0_initial.begin(), u0_initial.end());
	}

	// x0 = [0.000000000000001, 0.000000000000001, 57, 57, 57, 57, 16.66]; 初始速度为60km/h
	std::vector<double> x0_initial = { 0.000000000000001, 0.000000000000001, 75, 75, 75, 75, 25};
	std::vector<double> x0;
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		x0.insert(x0.end(), x0_initial.begin(), x0_initial.end());
	}

	std::vector<double> IDV;     
	IDV.insert(IDV.end(), u0.begin(), u0.end());
	IDV.insert(IDV.end(), x0.begin(), x0.end());
	

	// 侧偏角计算
	SX alphaf = x_tmp(0) + (Lf / x_tmp(6)) * x_tmp(1) - u_tmp(0) + 0.0000001;
	SX alphar = x_tmp(0) - (Lr / x_tmp(6)) * x_tmp(1) + 0.0000001;
	// 滑移率计算
	SX speed_L1 = (x_tmp(6) - dr / 2 * x_tmp(1)) * cos(u_tmp(0)) + (x_tmp(0) * x_tmp(6) + Lf * x_tmp(1)) * sin(u_tmp(0));
	SX speed_R1 = (x_tmp(6) + dr / 2 * x_tmp(1)) * cos(u_tmp(0)) + (x_tmp(0) * x_tmp(6) + Lf * x_tmp(1)) * sin(u_tmp(0));
	SX speed_L2 = (x_tmp(6) - dr / 2 * x_tmp(1));
	SX speed_R2 = (x_tmp(6) + dr / 2 * x_tmp(1));

	SX Kappa_L1 = (x_tmp(2) * Re - speed_L1) / speed_L1;
	SX Kappa_R1 = (x_tmp(3) * Re - speed_R1) / speed_R1;
	SX Kappa_L2 = (x_tmp(4) * Re - speed_L2) / speed_L2;
	SX Kappa_R2 = (x_tmp(5) * Re - speed_R2) / speed_R2;
	// 综合滑移率计算
	SX Kappa_r_L1 = sqrt(Kappa_L1 * Kappa_L1 + alphaf * alphaf);
	SX Kappa_r_R1 = sqrt(Kappa_R1 * Kappa_R1 + alphaf * alphaf);
	SX Kappa_r_L2 = sqrt(Kappa_L2 * Kappa_L2 + alphar * alphar);
	SX Kappa_r_R2 = sqrt(Kappa_R2 * Kappa_R2 + alphar * alphar);

	SX miu_com_L1 = C1 * (1 - exp(-C2 * Kappa_r_L1)) - C3 * Kappa_r_L1;
	SX miu_com_R1 = C1 * (1 - exp(-C2 * Kappa_r_R1)) - C3 * Kappa_r_R1;
	SX miu_com_L2 = C1 * (1 - exp(-C2 * Kappa_r_L2)) - C3 * Kappa_r_L2;
	SX miu_com_R2 = C1 * (1 - exp(-C2 * Kappa_r_R2)) - C3 * Kappa_r_R2;
	// 侧向力计算
	SX Fy_L1 = -ks * alphaf * Fz(0) * miu_com_L1 / Kappa_r_L1;
	SX Fy_R1 = -ks * alphaf * Fz(1) * miu_com_R1 / Kappa_r_R1;
	SX Fy_L2 = -ks * alphar * Fz(2) * miu_com_L2 / Kappa_r_L2;
	SX Fy_R2 = -ks * alphar * Fz(3) * miu_com_R2 / Kappa_r_R2;
	// 纵向力计算
	SX Fx_L1 = Kappa_L1 * Fz(0) * miu_com_L1 / Kappa_r_L1;
	SX Fx_R1 = Kappa_R1 * Fz(1) * miu_com_R1 / Kappa_r_R1;
	SX Fx_L2 = Kappa_L2 * Fz(2) * miu_com_L2 / Kappa_r_L2;
	SX Fx_R2 = Kappa_R2 * Fz(3) * miu_com_R2 / Kappa_r_R2;
	// 状态空间方程函数建立
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


	SX inputValues;
	for (int i = 0; i < Predictive_Horizon; i++)
	{
		if (i == 0)
		{
			inputValues = SX::vertcat({ ISV(0), ISV(1),ISV(2),ISV(3),ISV(4),ISV(5),ISV(6), CV(0,0), CV(0,1), CV(0,2), CV(0,3), CV(0,4) });

			SXVector outputValues = F_dxdt(inputValues);
			//std::cout << outputValues[0](1).size() << std::endl;

			con_eq(i, 0) = SV(i, 0) - ISV(0) - outputValues[0](0) * Ts;
			con_eq(i, 1) = SV(i, 1) - ISV(1) - outputValues[0](1) * Ts;
			con_eq(i, 2) = SV(i, 2) - ISV(2) - outputValues[0](2) * Ts;
			con_eq(i, 3) = SV(i, 3) - ISV(3) - outputValues[0](3) * Ts;
			con_eq(i, 4) = SV(i, 4) - ISV(4) - outputValues[0](4) * Ts;
			con_eq(i, 5) = SV(i, 5) - ISV(5) - outputValues[0](5) * Ts;
			con_eq(i, 6) = SV(i, 6) - ISV(6) - outputValues[0](6) * Ts;

			Tt_eq(i, 0) = CV(i, 1) + CV(i, 2) + CV(i, 3) + CV(i, 4) - Tt;
		}
		else
		{
			inputValues = SX::vertcat({ SV(i - 1,0), SV(i - 1,1),SV(i - 1,2),SV(i - 1,3),SV(i - 1,4),SV(i - 1,5),SV(i - 1,6), CV(i,0), CV(i,1), CV(i,2), CV(i,3), CV(i,4) });
			SXVector outputValues = F_dxdt(inputValues);
			con_eq(i, 0) = SV(i, 0) - SV(i - 1, 0) - outputValues[0](0) * Ts;
			con_eq(i, 1) = SV(i, 1) - SV(i - 1, 1) - outputValues[0](1) * Ts;
			con_eq(i, 2) = SV(i, 2) - SV(i - 1, 2) - outputValues[0](2) * Ts;
			con_eq(i, 3) = SV(i, 3) - SV(i - 1, 3) - outputValues[0](3) * Ts;
			con_eq(i, 4) = SV(i, 4) - SV(i - 1, 4) - outputValues[0](4) * Ts;
			con_eq(i, 5) = SV(i, 5) - SV(i - 1, 5) - outputValues[0](5) * Ts;
			con_eq(i, 6) = SV(i, 6) - SV(i - 1, 6) - outputValues[0](6) * Ts;

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
	
	
	Dict ipoptoptions = { {"print_level",0},{"print_timing_statistics","no"},{"max_iter",100},{"mu_init",0.01},{"tol",1e-8},{"mu_strategy","adaptive"} };
	Dict options = { {"expand",1},{"print_time",0},{"ipopt",ipoptoptions}};
	Function solver = nlpsol("solver", "ipopt", nlp, options);
	// 求解器定义完成


	std::map<std::string, DM> arg, res;

	std::vector<double> lbu_initial = { -0.1,0,0,0,0};
	std::vector<double> ubu_initial = { 0.1,2,2,2,2};
	std::vector<double> lbx_initial = { -100,-100,-100,-100,-100,-100,-100};
	std::vector<double> ubx_initial = { 100,100,100,100,100,100,100};
	std::vector<double> lbg_initial = { 0,0,0,0,0,0,0,0};
	std::vector<double> ubg_initial = { 0,0,0,0,0,0,0,0};

	std::vector<double> lbu;
	std::vector<double> ubu;
	std::vector<double> lbx;
	std::vector<double> ubx;
	std::vector<double> lbg;
	std::vector<double> ubg;

	for (int i = 0; i < Predictive_Horizon; i++)
	{
		lbu.insert(lbu.end(), lbu_initial.begin(), lbu_initial.end());
		ubu.insert(ubu.end(), ubu_initial.begin(), ubu_initial.end());
		lbx.insert(lbx.end(), lbx_initial.begin(), lbx_initial.end());
		ubx.insert(ubx.end(), ubx_initial.begin(), ubx_initial.end());
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

	Write_txt_file(delta_filename,delta0);
	Write_txt_file(Tfl_filename, Tfl);
	Write_txt_file(Tfr_filename, Tfr);
	Write_txt_file(Trl_filename, Trl);
	Write_txt_file(Trr_filename, Trr);

	return 0;
}