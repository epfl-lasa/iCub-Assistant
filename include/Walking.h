#pragma once

#include "Contacts.h"
#include "Joints.h"
#include "MedianFilter.h"

class Walking
{
public:

	MedianFilter state_filt;
	VectorXd lp3_state, lp3_dstate;
	double shift;
	double fblx, fbrx, fbly, fbry;
	double Tstep;
	double tphase;
	double phase;
	double ref_vx, ref_vy, ref_w;
	double force_lf, force_rf;
	double al, ar;
	double hip_gain_K;
	double min_dt;
	double start;
	bool hardware;

	void initialize(double minDT);
	void update(double time, double dt, Contact_Manager &points);
	void calculate_footstep_adjustments(double time, double dt, Contact_Manager &points, Joints &joints);
	void apply_speed_limits();
	void demand_speeds(Vector3d speeds);
	void cartesian_tasks(double time, Contact_Manager &points);
	void joint_tasks(double time, double dt, Contact_Manager &points, Joints &joints);
	bool early_phase(double time, double dt);
};
