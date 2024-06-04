/**
	QDOB - Quasi-periodic Disturbance Observer
	@author: Hisayoshi Muramatsu
	@date: 2024.06.04
*/

#ifndef DEF_QDOB
#define DEF_QDOB

class QDOB{

public:
	QDOB(
		int mu_int, // Switch for estimation only or compensation
		int l,      // The number of stages
		int Nmax,   // Maximum order
		double wa,  // Cutoff frequency
		double wb,  // Cutoff frequency
		double wc,  // Cutoff frequency
		double L,   // Period
		double M,   // Mass / Moment of inertia
		double T    // Sampling time
	);
	~QDOB();
	QDOB(const QDOB&)=delete;            // copy ctor
	QDOB& operator=(const QDOB&)=delete; // copy assignment

	// Controller based on the QDOB
	double Ctrl(double& hd, const double& r, const double& y);

private:
	const double mu;
	const int l;
	int N;
	const double wb, L, M, T, p;
	int eta;
	double y[3], lam[3], xi[2];

	std::vector<double> U, bU, theta, gamma, w;
	std::vector<Delay*> ThDelay;
	Delay* LamDelay;

	double fun_P(const double& lam_k1);
	double sub_h(const int& n, const double& wi, const double& Ui);
	double sub_w(const int& n, const int& N);

};

#endif
