#pragma once

#include "Contacts.h"
#include "Object.h"
#include "Walking.h"

class Manipulation
{
public:

	VectorXd bias_lh, bias_rh;
	VectorXd avg_lh, avg_rh;
	VectorXd force_lh, force_rh;
	VectorXd reading_lh, reading_rh;
	VectorXd grav_compns;

	double gamma;
	double dgamma;
	double tau;
	double dtau;
	double orientation_factor;
	MatrixXd A_V;
	MatrixXd A;
	VectorXd xd;
	VectorXd xR;
	VectorXd xV;
	VectorXd qd;
	VectorXd qR;
	VectorXd qV;
	double obj_width_dyn;
	double force_amp;

	Manipulation();
	~Manipulation() {};
	void set_start_point(VectorXd pos);
	void set_ideal_point(VectorXd pos);
	void read_forces(Contact_Manager &points);
	void initial_filtering(Contact_Manager &points, double dt);
	void secondary_filtering(Contact_Manager &points, double dt);
	void compliance_control(Contact_Manager &points);
	void jacobian_transpose(Contact_Manager &points, Joints &joints);
	void update(bool reachable, double dt, Object &object, bool grasp);
	bool grasped(double eps_obj);
	bool released(double eps_obj);
	VectorXd get_hand(Object &object, bool left);
};