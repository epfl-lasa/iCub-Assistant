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
	grav_compns = VectorXd::Zero(AIR_N_U);
	
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
	xd[0] = zero_v3; qd[0] = zero_quat;
	xd[1] = zero_v3; qd[1] = zero_quat;
	xO[0] = zero_v3; qO[0] = zero_quat;
	xO[1] = zero_v3; qO[1] = zero_quat;
	xR[0] = zero_v3; qR[0] = zero_quat;
	xR[1] = zero_v3; qR[1] = zero_quat;
	xV[0] = zero_v3; qV[0] = zero_quat;
	xV[1] = zero_v3; qV[1] = zero_quat;
}

void Manipulation::set_target_point(Object *object)
{
	VectorXd lh = object->get_hand(true);
	VectorXd rh = object->get_hand(false);

	// multiarm position ds initiations
	xO[0] = lh.segment(0,3);
	xO[1] = rh.segment(0,3);

	// grasping adjustment
	Vector3d left2right = (xO[1]-xO[0]).normalized();
	xO[0] += left2right * object->expansion * (1-obj_width_dyn);
	xO[1] -= left2right * object->expansion * (1-obj_width_dyn);
	
	// multiarm orientation ds initiations
	qO[0] = lh.segment(3,4);
	qO[1] = rh.segment(3,4);

	// grasping force
	force_amp = object->max_force;
}

void Manipulation::set_start_point(Object *object)
{
	VectorXd lh = object->get_hand(true);
	VectorXd rh = object->get_hand(false);

	// multiarm position ds initiations
	xR[0] = lh.segment(0,3);
	xV[0] = xR[0];
	xR[1] = rh.segment(0,3);
	xV[1] = xR[1];
	
	// multiarm orientation ds initiations
	qR[0] = lh.segment(3,4);
	qV[0] = qR[0];
	qR[1] = rh.segment(3,4);
	qV[1] = qR[1];
}

void Manipulation::set_ideal_point(Object *object)
{
	VectorXd lh = object->get_hand(true);
	VectorXd rh = object->get_hand(false);

	// multiarm position ds initiations
	xd[0] = lh.segment(0,3);
	xd[1] = rh.segment(0,3);
	
	// multiarm orientation ds initiations
	qd[0] = lh.segment(3,4);
	qd[1] = rh.segment(3,4);
}

void Manipulation::read_forces(Contact_Manager &points)
{
	MatrixXd Forces(AIR_N_BODY,3);
	MatrixXd Torques(AIR_N_BODY,3);
	points.model.get_reac(Forces, Torques);
	grav_compns = points.model.get_frcmat() * (-1);

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
	force_lh += (reading_lh - avg_lh - force_lh) * 1.0 * dt;
	force_rh += (reading_rh - avg_rh - force_rh) * 1.0 * dt;
}

void Manipulation::compliance_control(Contact_Manager &points)
{
	double force = (1.0 - obj_width_dyn) * force_amp;
	// calculate force directions
	Vector3d mid = points[CN_RH].ref_p.pos.segment(0,3)*0.5 + points[CN_LH].ref_p.pos.segment(0,3)*0.5;
	Vector3d hold_force_l = (mid - points[CN_LH].ref_p.pos.segment(0,3)).normalized() * force;
	Vector3d hold_force_r = (mid - points[CN_RH].ref_p.pos.segment(0,3)).normalized() * force;

	// This function should be called once and only after inverse kinematics	
	points[CN_LH].ref_p.pos.segment(0,3) += truncate_command(((force_lh).segment(0,3)+hold_force_l) * 0.1/30.0, 0.2, -0.2);
	points[CN_RH].ref_p.pos.segment(0,3) += truncate_command(((force_rh).segment(0,3)+hold_force_r) * 0.1/30.0, 0.2, -0.2);
}

void Manipulation::jacobian_transpose(Contact_Manager &points, Joints &joints)
{
	double force = (1.0 - obj_width_dyn) * force_amp;
	points[CN_LH].T.K = -Vector3d(30, 1, 1);
	points[CN_LH].R.K = -Vector3d(10, 0.3, 1);
	points[CN_RH].T.K = -Vector3d(30, 1, 1);
	points[CN_RH].R.K = -Vector3d(10, 0.3, 1);
	points.control(CN_LH, points[CN_LH].ref_p);
	points.control(CN_RH, points[CN_RH].ref_p);
	points[CN_LH].T.F_ref += - (points[CN_RH].p.pos - points[CN_LH].p.pos).segment(0,3).normalized() * force;
	points[CN_RH].T.F_ref += - (points[CN_LH].p.pos - points[CN_RH].p.pos).segment(0,3).normalized() * force;
	VectorXd h = grav_compns - points.get_virtual_mult();
	joints.ref_tau.segment(24,14) = h.segment(24,14);
}

void Manipulation::update(bool reachable, double dt, bool grasp)
{
	tau = reachable;
	dtau = 0;

	// orientation control gains
	MatrixXd A_V_Q = (A_V.array() * orientation_factor).matrix();
	MatrixXd A_Q = (A.array() * orientation_factor).matrix();

	// dynamical systems
	for(int i=0; i<2; i++)
	{
		// multiarm position DS
		Vector3d dxO = zero_v3;
		Vector3d X_I_C = xO[i];
		Vector3d dxV = gamma * dxO + dgamma * (xO[i] - X_I_C) + A_V * ((xV[i] - X_I_C) - gamma * (xO[i] - X_I_C));
		xV[i] += dxV * dt;
		Vector3d ATX = A * (xR[i] - xd[i] - tau * (xV[i] - xd[i])) + dtau * (xV[i] - xd[i]);
		Vector3d dxR  = tau * dxV * 0 + ATX;
		xR[i] += dxR * dt;

		// multiarm orientation DS
		VectorXd dqO = zero_v3;
		VectorXd Q_I_C = qO[i];
		Vector3d dqV = gamma * dqO + dgamma * quat_diff(qO[i], Q_I_C) + A_V_Q * (quat_diff(qV[i], Q_I_C) - gamma * quat_diff(qO[i], Q_I_C));
		qV[i] = quat_mul(quat_exp(0.5 * quat_deriv(dqV * dt)), qV[i]);
		Vector3d ATQ = A_Q * (quat_diff(qR[i], qd[i]) - tau * quat_diff(qV[i], qd[i])) + dtau * quat_diff(qV[i], qd[i]);
		Vector3d dqR  = tau * dqV * 0 + ATQ;
		qR[i] = quat_mul(quat_exp(0.5 * quat_deriv(dqR * dt)), qR[i]);
	}

	// grasping behavior
	obj_width_dyn += ((grasp ? 0 : 1) - obj_width_dyn) * 1 * dt;
}

VectorXd Manipulation::get_hand(bool left)
{
	int i = left ? 0 : 1;
	return vectorbig(xR[i],qR[i]);
}

bool Manipulation::grasped(double eps_obj)
{
	return obj_width_dyn < eps_obj;
}

bool Manipulation::released(double eps_obj)
{
	return obj_width_dyn > 1 - eps_obj;
}