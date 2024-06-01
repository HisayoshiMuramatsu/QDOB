/**
	QDOB - Quasi-periodic Disturbance Observer
	@author: Hisayoshi Muramatsu
	@date: 2024.06.01
*/

#include <iostream>
#include <fstream>

#include "QDOB.cpp"

double Motor(const double& u, const double& v, const double& M, const double& T); // Model of a plant: 1/(Ms^2)
double PDctrl(const double& cmd, const double& y, const double& T); // Proportional-derivative (PD) controller

int main(){
	// Data output file
	std::ofstream ofs("DATA.dat");

	// Fundamental parameters
	const double tEND = 10;          // [s]; Simulation time
	const double T    = 0.0001;      // [s]; Sampling time
	const double w0   = 10;          // [rad/s]; Fundamental frequency of a periodic disturbance [rad/s]
	const double L    = 2.0*M_PI/w0; // [s]; Period of the disturbance
	double t = 0;                    // [s]; Time

	// Motor
	const double M=1; // [kg]; Mass
	double u=0;       // [N]; Motor force
	double v=0;       // [N]; Exogenous force
	double y=0;       // [m]; Position

	// Evaluation
	double mse=0, rmse=0; // for root mean square error (RMSE)

	// Proportional-derivative (PD) position control
	double cmd = 0; // [m]; Position command
	double r   = 0; // [N]; Reference

	// QDOB
	const int mu = 1;       // 0: estimation only, 1: compensation
	const int l = 3;        // The number of stages of the linear-phase LPF
	const int Nmax = 256;   // The maximum order of the LPF at each stage
	const double wa = 100;  // [rad/s] << wb;  Cutoff frequency for harmonics suppression.
	const double wb = 1000; // [rad/s] >> wa;  Cutoff frequency for stabilization.
	const double wc = w0/4; // [rad/s] < w0/2; Cutoff frequency for suppression bandwidth surrounding harmonics.
	double hd = 0;          // [N]; Estimate of a quasi-periodic disturbance.
	QDOB q(mu, l, Nmax, wa, wb, wc, L, M, T);

	std::cout << "[Start]" << std::endl;

	do{ // Position control simulation

		// Control
		cmd = 0;
		r = PDctrl(cmd, y, T); // Position controller
		u = q.Ctrl(hd, r, y);  // QDOB

		// Plant
		v = 0;
		for(int i(1); i<8; i++) v += sin(i*w0*t); // Disturbance
		y = Motor(u, v, M, T); // Motor model

		// Computation of RMSE
		mse += (cmd-y)*(cmd-y)*(T/tEND);
		rmse = sqrt(mse);

		// Display
		if( (int)round(t/T)%(int)round(1/T) == 0 ){
			std::cout << "Time : "      << t     << " s" << std::endl;
			std::cout << "Error : "     << cmd-y << " m" << std::endl;
			std::cout << "Est. error: " << v-hd  << " N" << std::endl;
			std::cout << "RMSE : "      << rmse  << " m" << std::endl;
		}

		ofs << t << " " << cmd << " " << y << " " << v << " " << hd << std::endl; // Data output

		t += T; // Update of time

	}while(t<=tEND);

	std::cout << "[Finish]" << std::endl;

	ofs.close();

	return 0;
}

double Motor(const double& u, const double& v, const double& M, const double& T){
	static double f[3]={}, y[3]={};

	for(int i(0); i<2; i++){
		f[2-i]=f[1-i];
		y[2-i]=y[1-i];
	}
	f[0] = u + v;
	y[0] = 2*y[1] - y[2] + (T*T/4/M)*(f[0] + 2*f[1] + f[2]);

	return y[0];
}

double PDctrl(const double& cmd, const double& y, const double& T){
	const double Kp = 2500; // Proportional gain
	const double Kd = 100;  // Derivative gain
	const double g  = 100;  // Cutoff frequency for the pseudo differentiation with a LPF
	static double e[2]={}, de[2]={};

	de[1] = de[0];
	e[1]  = e[0];

	e[0] = cmd - y; // Error
	de[0] = ((2-g*T)/(2+g*T))*de[1] + (2*g/(2+g*T))*(e[0]-e[1]); // Derivative of error

	return Kp*e[0] + Kd*de[0];

}
