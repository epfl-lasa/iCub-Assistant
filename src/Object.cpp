#include "Object.h"

VectorXd get_hand_general(Object &object, VectorXd center, double shrink, bool left)
{
	VectorXd out = VectorXd::Zero(7);
	Vector3d direction = Vector3d(0,0,0);
	direction[object.grasp_axis] = (object.dim[object.grasp_axis]/2.0 + shrink * object.expansion) * (left ? 1 : -1);
	out.segment(0,3) = center.segment(0,3) + quat2dc(center.segment(3,4)) * direction;
	out.segment(3,4) = quat_mul(center.segment(3,4), ang2quat(Vector3d(0,-M_PI/2,0)));
	return out;
}

Object::Object()
{
	sens_pos = VectorXd::Zero(7);
	sens_vel = VectorXd::Zero(6);
	dim = VectorXd::Zero(3);
	expansion = 0;
	grasp_axis = 0;
	top_marker = false;
}

VectorXd Object::get_hand(bool left)
{
	return get_hand_general(*this, sens_pos, 1, left);
}

VectorXd trans7_add(VectorXd a, VectorXd b)
{
	VectorXd c(7);
	c.segment(0,3) = a.segment(0,3) + b.segment(0,3);
	c.segment(3,4) = quat_mul(b.segment(3,4), a.segment(3,4));
	return c;
}

VectorXd trans7_invert(VectorXd a)
{
	VectorXd c = a;
	c.segment(0,3) *= -1.0;
	c.segment(3,4) = quat_inverse(c.segment(3,4));
	return c;
}

VectorXd trans7_sub(VectorXd a, VectorXd b)
{
	VectorXd c(7);
	b = trans7_invert(b);
	c.segment(0,3) = quat2dc(b.segment(3,4)) * (a.segment(0,3) + b.segment(0,3));
	c.segment(3,4) = quat_mul(b.segment(3,4), a.segment(3,4));
	return c;
}

VectorXd trans7_scale(VectorXd a, double b)
{
	VectorXd c = a;
	c.segment(0,3) *= b;
	c.segment(3,4) =  quat_scale(c.segment(3,4), b);
	return c;
}

bool Object::update_position(VectorXd P_B, VectorXd P_lf, VectorXd P_rf, VectorXd P_R, VectorXd P_obj)
{
	if(P_R.isZero(0) || P_obj.isZero(0)) // some mocap port is not working ...
		return false;

	// This function brings the Object to a frame attached to the mid-foot point, heading forward
	// model's base frame: P_B
	// model's feet frame, P_lf, P_rf

	// model's mid-foot frame
	VectorXd P_F = trans7_add(trans7_scale(P_lf,0.5), trans7_scale(P_rf,0.5));

	// model's mid-foot frame in model's base frame
	VectorXd P_F_B = trans7_sub(P_F, P_B);

	// reality's mid-foot frame in reality's base frame
	VectorXd P_Fp_R = P_F_B;

	// reality's object frame in reality's base frame
	VectorXd P_obj_R = trans7_sub(P_obj, P_R);

	// reality's object frame in reality's mid-foot frame
	VectorXd P_obj_Fp = trans7_sub(P_obj_R, P_Fp_R);

	apply_mocap_thresholds(P_obj_Fp);

	sens_pos = P_obj_Fp;

	if(top_marker)
		sens_pos.segment(0,3) -= quat2dc(sens_pos.segment(3,4)) * Vector3d(0,0,dim[2]/2.0);

	return true;
}

void apply_mocap_thresholds(VectorXd &pos)
{
	// intuitive thresholds on feasible positioins
	pos[0] = truncate_command(pos[0], 0.5, 0.0);
	pos[1] = truncate_command(pos[1], 0.5, -0.5);
	pos[2] = truncate_command(pos[2], 1.5, 0.0);

	// intuitive thresholds on feasible orientations
	double theta_max = M_PI/4; // in any direction
	if(pos.segment(3,4).adjoint()*zero_quat < cos(theta_max/2.0))
	{
		double alpha = ( cos(theta_max/2.0) - 1.0) / ( pos.segment(3,4).adjoint()*zero_quat - 1.0);
		pos.segment(3,4) = alpha * (pos.segment(3,4)-zero_quat)*alpha + zero_quat;
		pos.segment(3,4) = pos.segment(3,4).normalized();
	}
}
