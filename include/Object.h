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
	
	// the direction in which the grasp should be made
	int grasp_axis;

	// whether the markers are attached on top. Otherwise they give the geometric center by default.
	bool top_marker;

	Object();
	~Object() {};

	// get the raw mocap data and process, if not valid stick to the previous positions
	bool update_position(VectorXd P_B, VectorXd P_lf, VectorXd P_rf, VectorXd P_R, VectorXd P_obj);

	// get the desired grasping points expanded
	VectorXd get_hand(bool left);
};

//! add two transformations together
VectorXd trans7_add(VectorXd a, VectorXd b);

//! invert a transformation
VectorXd trans7_invert(VectorXd a);

//! describe a transformation a in b
VectorXd trans7_sub(VectorXd a, VectorXd b);

//! scale a transformation
VectorXd trans7_scale(VectorXd a, double b);

//! threshold object position
void apply_mocap_thresholds(VectorXd &Object);

//! get grasp point
VectorXd get_hand_general(Object &object, VectorXd center, double shrink, bool left);
