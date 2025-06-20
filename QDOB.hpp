/**
	QDOB - Quasiperiodic Disturbance Observer
	@author: Hisayoshi Muramatsu
	@date: 2025.06.20

	The QDOB has been published in the following paper.
	---
	Hisayoshi Muramatsu,
	“Quasiperiodic Disturbance Observer for Wideband Harmonic Suppression,”
	IEEE Transactions on Control Systems Technology, Early Access, 2025.
	DOI:10.1109/TCST.2025.3566560
	(https://ieeexplore.ieee.org/document/11006295)
	---

	If you intend to modify the QDOB algorithm, it is highly recommended to read the above paper.
	Note that modifying the controller in the discrete-time domain can be challenging.
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
