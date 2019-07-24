#pragma once

#include "Basics.h"
#include "lpvDS.h"

class Navigation
{
public:

	Navigation();
	~Navigation() {};
	Vector3d linear_DS(VectorXd target);
	Vector3d nonlinear_DS(VectorXd target);
};
