/**
	QDOB - Quasi-periodic Disturbance Observer
	@author: Hisayoshi Muramatsu
	@date: 2024.06.01
*/

#include <vector>
#include <math.h>

#include "Delay.cpp"
#include "QDOB.hpp"

QDOB::QDOB(
		int mu_int,
		int l,
		int Nmax,
		double wa,
		double wb,
		double wc,
		double L,
		double M,
		double T
	):mu((double)mu_int), l(l), N(Nmax), wb(wb), L(L), M(M), T(T), p((2/L)*tan(L*wc/2)), eta(0), y{}, lam{}, xi{}, U(l), bU(l), theta(l), gamma(l), w(l), ThDelay(l), LamDelay(nullptr)
{
	const int bL = (int)round(L/T);
	const double c = 0.5*pow( (T*wa/M_PI) , 1.0/(double)l );

	for(int i(0);i<l;i++){
		if(i==0){
			U[i] = T;
		}else{
			U[i] = M_PI/w[i-1];
		}
		bU[i] = (int)round(U[i]/T);
		w[i]  = 2*M_PI*c/U[i];
	}

	int bUsum(0);
		for(int i(0);i<l;i++) bUsum += bU[i];
	N = std::min( (int)floor((double)(bL-1)/(double)bUsum) , Nmax );

	eta = bL - N*bUsum;

	for(int i(0);i<l;i++) ThDelay[i] = new Delay(2*bL);
	LamDelay = new Delay(eta);

}

QDOB::~QDOB(){
	for(int i(0);i<l;i++) delete ThDelay[i];
	delete LamDelay;
}

double QDOB::Ctrl(double& hd, const double& r, const double& y_){
	for(int i(0); i<2; i++){
		y[2-i] = y[1-i];
		lam[2-i] = lam[1-i];
	}
	y[0] = y_;
	xi[1]=xi[0];

	xi[0]  = ( 1 / (T*(1+wb*T)) ) * ( T*xi[1] + M*wb*(y[0]-2*y[1]+y[2]) );
	hd     = ( p*L / ((1-mu)*p*L+2) ) * (xi[0]-r) + fun_P(lam[1]);
	lam[0] = ( p*L / ((1-mu)*p*L+2) ) * (xi[0]-r) - ( ((1-mu)*p*L-2) / ((1-mu)*p*L+2) ) * hd;

	return r - mu*hd;
}

double QDOB::fun_P(const double& lam_k1){
	const double theta0 = LamDelay->Memory(lam_k1,1-eta); // lam_(k+eta)
		LamDelay->Update(lam_k1);
	for(int i(0);i<l;i++){
		theta[i]=0;
		gamma[i]=0;
		for(int n(-N); n<=N; n++){
			if(i==0){
				theta[i] += sub_w(n,N)*sub_h(n,w[i],U[i])*ThDelay[i]->Memory(theta0,(n-N)*bU[i]);
			}else{
				theta[i] += sub_w(n,N)*sub_h(n,w[i],U[i])*ThDelay[i]->Memory(theta[i-1],(n-N)*bU[i]);
			}
			gamma[i] += sub_w(n,N)*sub_h(n,w[i],U[i]);
		}
		if(i==0){
			ThDelay[i]->Update(theta0);
		}else{
			ThDelay[i]->Update(theta[i-1]);
		}
		theta[i]=theta[i]/gamma[i];
	}
	return theta[l-1];
}

double QDOB::sub_h(const int& n, const double& wi, const double& Ui){
	if(n==0){
		return Ui*wi/M_PI;
	}else{
		return sin((double)n*Ui*wi)/((double)n*M_PI);
	}
}

double QDOB::sub_w(const int& n, const int& N){
	return 0.42 + 0.5*cos((double)n*M_PI/(double)N) + 0.08*cos(2*(double)n*M_PI/(double)N);
}
