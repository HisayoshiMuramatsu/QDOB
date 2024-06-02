/**
	QDOB - Quasi-periodic Disturbance Observer
	@author: Hisayoshi Muramatsu
	@date: 2024.06.02
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
