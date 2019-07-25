#pragma once

#include "Model.h"

Vector4d getQ(VectorXd Q);

VectorXd setQ(VectorXd Q, Vector4d a);

class Joints
{
public:
	
	VectorXd ref_pos;
	VectorXd sens_pos;
	VectorXd init_pos;
	VectorXd Ik_pos0;
	VectorXd com_pos;
	VectorXd ref_tau;
	VectorXd sens_vel;
	VectorXd sens_acc;
	VectorXd sens_tau;
	VectorXd freeze;
	VectorXd mode;

	Joints();
	~Joints() {};
	void plot(Model &model);
};
