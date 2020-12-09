#pragma once
#include "cdisccontroller.h"
#include "CParams.h"
#include "CDiscCollisionObject.h"
#include <cmath>
#include <ctime>
#include <cstdlib>
//#include <boost/multi_array.hpp>
//#include <cassert>

typedef unsigned int uint;
class CQLearningController :
	public CDiscController
{
private:
	uint _grid_size_x;
	uint _grid_size_y;
public:
   double ** Rewards;
   double **** Q;
	CQLearningController(HWND hwndMain);
	virtual void InitializeLearningAlgorithm(void);
	double R(uint x, uint y, uint sweeper_no);
	virtual bool Update(void);
	virtual ~CQLearningController(void);
};

