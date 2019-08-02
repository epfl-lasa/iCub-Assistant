#include "Object.h"

Object::Object()
{
	sens_pos = VectorXd::Zero(7);
	sens_vel = VectorXd::Zero(6);
	dim = VectorXd::Zero(3);
	expansion = 0;
	grasp_axis = Vector3d(0, 1, 0);
	grasp_rot = zero_quat;
	top_marker = false;
	max_force = 0;
	enable_optimal_grasp();
}

VectorXd Object::get_hand(bool left)
{
	VectorXd out = VectorXd::Zero(7);
	Vector3d direction = grasp_axis;
	direction *= abs(double(grasp_axis.adjoint() * dim))/2 + expansion;
	direction *= left ? 1.0 : -1.0;
	out.segment(0,3) = sens_pos.segment(0,3) + quat2dc(sens_pos.segment(3,4)) * direction;
	Vector4d rot = local_rot(Vector3d(0,1,0), grasp_axis);
	rot = quat_mul(rot, grasp_rot);
	rot = quat_mul(rot, ang2quat(Vector3d(0,-M_PI/2,0)));
	out.segment(3,4) = quat_mul(sens_pos.segment(3,4), rot);
	return out;
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
	{
		cout << " Some mocap data port is not set up properly" << endl;
		return false;
	}

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

	sens_pos = P_obj_Fp;

	if(top_marker)
		sens_pos.segment(0,3) -= quat2dc(sens_pos.segment(3,4)) * Vector3d(0,0,dim[2]/2.0);

	optimize_grasp();

	return true;
}

Vector4d Object::local_rot(Vector3d v1, Vector3d v2)
{
	// if(v1.adjoint()*v2 < (-1 + 1e-3))
	// 	return ang2quat(Vector3d(0,M_PI,0));
	// if(v1.adjoint()*v2 > (1 - 1e-3))
	// 	return zero_quat;

	Vector4d q;
	Vector3d a = v1.cross(v2);
	if(v1.adjoint()*v2 < (-1 + 1e-3))
	 	a = abs(v1[0]) > abs(v1[2]) ? Vector3d(-v1[1], v1[0], 0) : Vector3d(0, -v1[2], v1[1]);
	q.segment(0,3) = a;
	q[3] = sqrt(v1.squaredNorm() * v2.squaredNorm()) + v1.adjoint()*v2;
	return q.normalized();
}

void Object::optimize_grasp()
{
	if(allow_search)
	{
		// now decide which side to grasp
		MatrixXd map = MatrixXd(6,3);
		map <<  1, 0, 0,
				-1, 0, 0,
				0, 1, 0,
				0, -1, 0,
				0, 0, 1,
				0, 0, -1;
		double min_diff = 1e6;
		Vector3d ax_diff = Vector3d(0,1,0);
		Vector4d rot_diff = zero_quat;
		for(int i=0;i<6;i++)
		{
			// search over grasping sides
			Vector3d ax = map.block(i,0,1,3).transpose();
			for(int j=0;j<4;j++)
			{
				// search over different grasp orientations around the grasp axis
				Vector4d rot_extra = ang2quat(Vector3d(0,1,0) * j * M_PI/2.0);
				Vector4d rot = local_rot(Vector3d(0,1,0), ax);
				rot = quat_mul(rot, rot_extra);
				double error = quat_log(quat_mul(sens_pos.segment(3,4), rot)).norm();
				if(error<min_diff)
				{
					ax_diff = ax;
					min_diff = error;
					rot_diff = rot_extra;
				}
			}
		}
		grasp_axis = ax_diff;
		grasp_rot = rot_diff;
	}
}

void Object::enable_optimal_grasp()
{
	allow_search = true;
	optimize_grasp();
}

void Object::disable_optimal_grasp()
{
	allow_search = false;
	grasp_axis = Vector3d(0, 1, 0);
	grasp_rot = zero_quat;
}