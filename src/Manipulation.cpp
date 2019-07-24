#include "Manipulation.h"



Manipulation::Manipulation()
{
	bias_lh = VectorXd::Zero(6);
	bias_rh = VectorXd::Zero(6);
	avg_lh = VectorXd::Zero(6);
	avg_rh = VectorXd::Zero(6);	
	force_lh = VectorXd::Zero(6);
	force_rh = VectorXd::Zero(6);
	reading_lh = VectorXd::Zero(6);
	reading_rh = VectorXd::Zero(6);
	
	#ifdef HARDWARE
		// pure bias values read from the hardware when stretching the arms and subtracting lower arm weights
		bias_lh << -11.8466,  -14.6179,  -36.7255,  0.177665, -0.748966,   0.18572;
		bias_rh <<  32.6632,  -5.57603,  -55.8315, -0.748966, -0.511756,  0.313862;
	#endif

	// multiarm ds variables
	gamma = 1;
	dgamma = 0;
	tau = 0;
	dtau = 0;
	orientation_factor = 3.0;
	A_V = MatrixXd::Zero(3,3);
	A_V(0,0) = -5;
	A_V(1,1) = -5;
	A_V(2,2) = -5;
	A = MatrixXd::Zero(3,3);
	A(0,0) = -0.3;
	A(1,1) = -0.3;
	A(2,2) = -0.3;

	// grasping properties
	obj_width_dyn = 1;
	force_amp = 0; 

	// ideal pos
	xd = zero_v3;
	qd = zero_quat;
}

void Manipulation::set_start_point(VectorXd pos)
{
	// multiarm position ds initiations
	xR = pos.segment(0,3);
	xV = xR;
	
	// multiarm orientation ds initiations
	qR = pos.segment(3,4);
	qV = qR;
}

void Manipulation::set_ideal_point(VectorXd pos)
{
	// multiarm position ds initiations
	xd = pos.segment(0,3);
	
	// multiarm orientation ds initiations
	qd = pos.segment(3,4);
}

void Manipulation::read_forces(Contact_Manager &points)
{
	// arm force estimation, this compensates lower-arm weights
	MatrixXd Forces(AIR_N_BODY,3);
	MatrixXd Torques(AIR_N_BODY,3);
	points.model.get_reac(Forces, Torques);

	MatrixXd RR = points.model.get_orient(26);
	reading_lh = 	vectorbig(	RR*(-points[CN_LH].T.F_sens-bias_lh.segment(0,3)) + RR*Forces.block(26,0,1,3).transpose(), 
								RR*(-points[CN_LH].R.F_sens-bias_lh.segment(3,3)) + RR*Torques.block(26,0,1,3).transpose());
	
	RR = points.model.get_orient(34);
	reading_rh = 	vectorbig(	RR*(-points[CN_RH].T.F_sens-bias_rh.segment(0,3)) + RR*Forces.block(34,0,1,3).transpose(),
								RR*(-points[CN_RH].R.F_sens-bias_rh.segment(3,3)) + RR*Torques.block(34,0,1,3).transpose());
}

void Manipulation::initial_filtering(Contact_Manager &points, double dt)
{
	read_forces(points);
	
	// orientation dependent bias if any
	avg_lh += (reading_lh - avg_lh) * 1.0 * dt;
	avg_rh += (reading_rh - avg_rh) * 1.0 * dt;
}

void Manipulation::secondary_filtering(Contact_Manager &points, double dt)
{
	read_forces(points);
	
	// online differences
	force_lh += (reading_lh - avg_lh - force_lh) * 5.0 * dt;
	force_rh += (reading_rh - avg_rh - force_rh) * 5.0 * dt;
}

void Manipulation::compliance_control(Contact_Manager &points, double force_amp)
{
	// calculate force directions
	Vector3d mid = points[CN_RH].ref_p.pos*0.5 + points[CN_LH].ref_p.pos*0.5;
	Vector3d hold_force_l = (mid - points[CN_LH].ref_p.pos).normalized() * force_amp;
	Vector3d hold_force_r = (mid - points[CN_RH].ref_p.pos).normalized() * force_amp;

	// This function should be called once and only after inverse kinematics	
	points[CN_LH].ref_p.pos += truncate_command(((force_lh).segment(0,3)+hold_force_l) * 0.1/30.0, 0.2, -0.2);
	points[CN_RH].ref_p.pos += truncate_command(((force_rh).segment(0,3)+hold_force_r) * 0.1/30.0, 0.2, -0.2);
}

void Manipulation::update(bool reachable, double dt, Object &object, bool grasp)
{
	tau = reachable;
	dtau = 0;

	// orientation control gains
	MatrixXd A_V_Q = (A_V.array() * orientation_factor).matrix();
	MatrixXd A_Q = (A.array() * orientation_factor).matrix();

	// multiarm position DS
	Vector3d xO = object.sens_pos.segment(0,3);
	Vector3d dxO = object.sens_vel.segment(0,3);
	Vector3d X_I_C = xO;
	Vector3d dxV = gamma * dxO + dgamma * (xO - X_I_C) + A_V * ((xV - X_I_C) - gamma * (xO - X_I_C));
	xV += dxV * dt;
	Vector3d ATX = A * (xR - xd - tau * (xV - xd)) + dtau * (xV - xd);
	Vector3d dxR  = tau * dxV * 0 + ATX;
	xR += dxR * dt;

	// multiarm orientation DS
	VectorXd qO = object.sens_pos.segment(3,4);
	VectorXd dqO = object.sens_vel.segment(3,3);
	VectorXd Q_I_C = qO;
	Vector3d dqV = gamma * dqO + dgamma * quat_diff(qO, Q_I_C) + A_V_Q * (quat_diff(qV, Q_I_C) - gamma * quat_diff(qO, Q_I_C));
	qV = quat_mul(quat_exp(0.5 * quat_deriv(dqV * dt)), qV);
	Vector3d ATQ = A_Q * (quat_diff(qR, qd) - tau * quat_diff(qV, qd)) + dtau * quat_diff(qV, qd);
	Vector3d dqR  = tau * dqV * 0 + ATQ;
	qR = quat_mul(quat_exp(0.5 * quat_deriv(dqR * dt)), qR);

	// grasping behavior
	obj_width_dyn += ((grasp ? 0 : 1) - obj_width_dyn) * 1 * dt;
	force_amp = obj_width_dyn * 20;
	force_amp *= grasp? 1 : 0;
}

bool Manipulation::grasped(double eps_obj)
{
	return obj_width_dyn < eps_obj;
}

bool Manipulation::released(double eps_obj)
{
	return obj_width_dyn > 1 - eps_obj;
}

VectorXd Manipulation::get_hand(Object &object, bool left)
{
	return get_hand_general(object, vectorbig(xR,qR), obj_width_dyn, left);
}

