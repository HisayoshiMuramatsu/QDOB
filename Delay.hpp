/**
	QDOB - Quasi-periodic Disturbance Observer
	@author: Hisayoshi Muramatsu
	@date: 2024.06.04
*/

#ifndef DEF_DELAY
#define DEF_DELAY

#include <vector>
#include <math.h>

class Delay{
public:
	Delay(
		int Lmax
	);
	double Memory(const double& x, const int& L);
	void Update(const double& x);

private:
	int Lmax, count;
	std::vector<double> Buff;
};

#endif
