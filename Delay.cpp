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

#include "Delay.hpp"

Delay::Delay(
		int Lmax
	):Lmax(Lmax), count(0), Buff(Lmax){}

double Delay::Memory(const double& x, const int& L){
	int num = count+L;
	if(num<0) num += Lmax;
	if(L==0){
		return x;
	}else{
		return Buff[num];
	}
}

void Delay::Update(const double& x){
	Buff[count] = x;
	count++;
	if(count==Lmax) count=0;
}
