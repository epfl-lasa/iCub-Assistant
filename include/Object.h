#pragma once

#include "Basics.h"

class Object
{
public:

	// mocap positions, orientations
	VectorXd sens_pos;
	VectorXd sens_vel;

	// the xyz dimensions, assuming a box shape
	Vector3d dim;

	// How distant the hands should be with respect to the object
	double expansion;
	
	// the direction (from object center) in which the left grasp should be made
	Vector3d grasp_axis;
	Vector4d grasp_rot;

	// grasping behavior
	double max_force;

	// quaternion between two different grasp axis
	Vector4d local_rot(Vector3d v1, Vector3d v2);

	// whether the markers are attached on top. Otherwise they give the geometric center by default.
	bool top_marker;

	Object();
	~Object() {};

	// get the raw mocap data and process, if not valid stick to the previous positions
	bool update_position(VectorXd P_B, VectorXd P_lf, VectorXd P_rf, VectorXd P_R, VectorXd P_obj);

	// get the desired grasping points expanded
	VectorXd get_hand(bool left);

	// optimal grasp
	bool allow_search;
	void optimize_grasp();
	void enable_optimal_grasp();
	void disable_optimal_grasp();
};

//! add two transformations together
VectorXd trans7_add(VectorXd a, VectorXd b);

//! invert a transformation
VectorXd trans7_invert(VectorXd a);

//! describe a transformation a in b
VectorXd trans7_sub(VectorXd a, VectorXd b);

//! scale a transformation
VectorXd trans7_scale(VectorXd a, double b);