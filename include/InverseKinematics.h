#pragma once
#include "Contacts.h"
#include "Joints.h"

class InverseKinematics
{
public:

	double damp;
	int num_iter;
	VectorXd Xr;
	VectorXd X0;
	VectorXd slack;
	MatrixXd J;
	VectorXd damping;

	InverseKinematics();
	~InverseKinematics() {};
	void update_model(Contact_Manager &points, VectorXd q0);
	VectorXd solve_QP(VectorXd &qref, VectorXd &qlow, VectorXd &qup);
	double return_hand_error();
	double solve(Contact_Manager &points, VectorXd& ref_pos, VectorXd freeze);
};
