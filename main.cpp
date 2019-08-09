#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>


// lpvDS stuff
#include <stdio.h>
#include <fstream>
#include <time.h>
#include "eigen3/Eigen/Dense"

#include "Wrapper.h"
#include "InverseKinematics.h"
#include "Walking.h"
#include "Collision.h"
#include "Navigation.h"
#include "Manipulation.h"

using namespace std;

enum state {BALANCE=0,  PICK_APPROACH, PICK, PICK_STAND, DROP_APPROACH, DROP, DROP_STAND, WALK_START, WALK};
enum trigger {NONE=0, GO, HALT, TAKE, RELEASE};

bool exit_sim = false;
void my_handler(int s)
{
	cout << "Exit Program Now ... " << endl;
	exit_sim = true;
}

void loadData(Wrapper &wrapper, Contact_Manager &points, Joints &joints)
{
	// read the wrapper and bring everything to the mid-foot frame
	wrapper.readSensors(joints.sens_pos, joints.sens_vel, joints.sens_tau);
	for(int i=0; i<4; i++)
	{
		points[i+1].T.F_sens 	= wrapper.FTsensor[i][0];
		points[i+1].R.F_sens 	= wrapper.FTsensor[i][1];
	}

	// read IMU and remove yaw angle
	double roll = atan2(wrapper.R(2,1), wrapper.R(2,2));
	double pitch = -asin(wrapper.R(2,0));
	MatrixXd rot = ang2dc(Vector3d(0,pitch,0)) * ang2dc(Vector3d(roll,0,0));
	joints.sens_pos					= setQ(joints.sens_pos, dc2quat(rot));
	joints.sens_vel.segment(3,3)	= wrapper.angvel;

	// update the model
	points.model.set_state(joints.sens_pos, joints.sens_vel);

	// bring the mid-foot points to zero and make feet point to +x direction
	Vector3d base 	= points.model.get_pos(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.model.get_pos(points[CN_RF].body, points[CN_RF].offset) * 0.5;
	Vector3d dbase 	= points.model.get_vel(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.model.get_vel(points[CN_RF].body, points[CN_RF].offset) * 0.5;

	// apply transformations
	joints.sens_pos.segment(0,3) 	-= base;
	joints.sens_vel.segment(0,3) 	-= dbase;

	// update the model
	points.model.set_state(joints.sens_pos, joints.sens_vel);
	VectorXd zero_acc = VectorXd::Zero(AIR_N_U);
	points.model.set_acc(zero_acc);
	points.update_kinematics();
}

int main(int argc, char *argv[])
{
	// setup existing condition
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// YARP Wrapper
	Wrapper wrapper;
	if(wrapper.checkRobot(argc, argv))
		{cout << "Robot name problem" << endl; return 1;}
	wrapper.initialize();
	wrapper.rePID(false);

	// Initialize reading robot/object pos from gazebo/mocap
	#ifdef HARDWARE
		wrapper.initObject(1, "/icub/DesiredCoMVelocity:o");
	#else
		wrapper.initObject(0, wrapper.robotName + "/get_root_link_WorldPose:o");
		wrapper.initObject(1, "/BoxSmall/GraspedObject_WorldPose:o");
		wrapper.initObject(2, "/Broom/GraspedObject_WorldPose:o");
		wrapper.initObject(3, "/Cart/GraspedObject_WorldPose:o");
	#endif
	yarp::os::Time::delay(1);
	for(int i=0; i<=3; i++)
		if(wrapper.readObject(i).isZero())
			{
				cout << "WARNING: Object" << i << "mocap problem" << endl; 
				// return 1;
			}

	// logger
	std::ofstream OutRecord;
	string path_log = "/tmp/log_" + wrapper.robotName + ".txt";
	OutRecord.open(path_log.c_str());

	// joint variables
	Joints joints;

	// set entire body to position-direct mode
	for(int i=6;i<AIR_N_U;i++) 
		joints.mode[i] = 1;

	// set arms to force control in simulation
	#ifndef HARDWARE
		for(int i=24;i<24+14;i++) 
			joints.mode[i] = 2;
	#endif

	// disable the neck joints
	for(int i=9;i<12;i++)
		joints.freeze[i] = 1;

	// inverse kinematics
	InverseKinematics IK;

	// self collision avoidance
	Collision collision;

	// contact definition
	Contact_Manager points;

	// update joint limits
	wrapper.initializeJoint(joints.mode.segment(6,AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	wrapper.getJointLimits(&points.model.qmin[0], &points.model.qmax[0]);

	// read data once
	yarp::os::Time::delay(1);
	loadData(wrapper, points, joints);
	joints.init_pos = joints.sens_pos;

	// robot default properties
	VectorXd des_com = vectorbig(Vector3d(0.0, 0.0, 0.53), zero_quat);
	VectorXd des_obj = vectorbig(Vector3d(0.2, 0.0, 0.65), zero_quat);
	VectorXd des_lf = vectorbig(Vector3d(0.0, 0.095, 0.0), zero_quat);
	VectorXd des_rf = vectorbig(Vector3d(0.0,-0.095, 0.0), zero_quat);
	Vector3d com_adjustment = zero_v3;
	Vector3d final_com_pos = des_com.segment(0,3);

	// navigation DS
	Navigation DS_nav;

	// walking controller
	Walking walking;
	walking.initialize(minTimeStep);

	// Object properties, by default facing forward
	Object Pfront(des_obj);
	Pfront.dim = Vector3d(0.2, 0.2, 0.2);
	Pfront.max_expansion = 0.1;
	Pfront.max_force = 20;

	// grasping controller
	Manipulation manip;
	Object *Ptarget = &Pfront;
	VectorXd Pdrop = des_obj;
	manip.set_ideal_point(Ptarget->get_hand_ideal());
	manip.set_start_point();

	// sensed from the environment
	Object BoxSmall = Pfront;
	#ifdef HARDWARE
		BoxSmall.top_marker = true;
	#endif
	
	Object Broom(des_obj);
	Broom.dim = Vector3d(0.02, 0.02, 0.4);
	Broom.max_expansion = 0.1;
	Broom.max_force = 0;
	Broom.grasp_offset = Vector3d(0,0.1,0);

	Object Cart(des_obj);
	Cart.ideal_pos[2] = 0.54;
	Cart.dim = Vector3d(0.02, 0.4, 0.02);
	Cart.max_expansion = 0.1;
	Cart.max_force = 0;
	Cart.grasp_offset = Vector3d(0,0.1,0);
	Cart.grasp_opposite = false;
	Cart.ideal_grasp_axis = Vector3d(0,0,1);

	Ptarget = &Broom;

	// prepare for reading the keyboard
	nonblock(1);

	// start the loop
	double calib_time = 3;  // WARNING
	state S = BALANCE;
	state SW = BALANCE;
	trigger G = NONE;
	trigger GW = NONE;
	bool navigate = false;
	bool follow = false;
	bool grasp = false;
	double start_time = wrapper.time;
	while(!exit_sim)
	{
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// sensing //////////////////////////////////////////////////////////////////////////////////////////////////
		double time = wrapper.time - start_time;
		
		// read sensors
		loadData(wrapper, points, joints);

		// hand force sensor filtering
		if(time<calib_time) 
			manip.initial_filtering(points, wrapper.dt);
		else
			manip.secondary_filtering(points, wrapper.dt);

		// leg forces filtering
		walking.update(time, wrapper.dt, points);

		// robot position
		VectorXd Root = wrapper.readObject(0);
		#ifdef HARDWARE
			// do whatever transform necessary to find root's pos/rot
		#else
			Root.segment(3,4) = quat_mul(Root.segment(3,4), ang2quat(Vector3d(0,0,M_PI))); 
		#endif

		// object world positions
		VectorXd Base = points.model.get_trans7(0,zero_v3);
		BoxSmall.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(1));
		Broom.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(2));
		Cart.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(3));

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// read keyboard signals ////////////////////////////////////////////////////////////////////////////////////
		double delta = 0.01;
		if(khbit() != 0)
		{
			char c = fgetc(stdin);
			fflush(stdin);
			switch(c)
			{
				// general signals
				case 'g': GW = GO; break;
				case 'h': GW = HALT; break;
				case 't': G = TAKE; break;
				case 'r': G = RELEASE; break;
				case 'x': G = HALT; break;
				case 'n': navigate = !navigate; break;
				case 'f': follow = !follow; break;
				case 'm': grasp = !grasp; break;

				// desired velocities
				case 'w': walking.ref_vx += 0.1; break;
				case 's': walking.ref_vx -= 0.1; break;
				case 'a': walking.ref_vy += 0.1; break;
				case 'd': walking.ref_vy -= 0.1; break;
				case 'q': walking.ref_w += 0.1; break;
				case 'e': walking.ref_w -= 0.1; break;

				// desired object position
				case 'i': Ptarget->sens_pos[0] += delta; break;
				case 'k': Ptarget->sens_pos[0] -= delta; break;
				case 'j': Ptarget->sens_pos[1] += delta; break;
				case 'l': Ptarget->sens_pos[1] -= delta; break;
				case 'p': Ptarget->sens_pos[2] += delta; break;
				case ';': Ptarget->sens_pos[2] -= delta; break;
				case 'o': Ptarget->sens_pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0,-delta*5)), Ptarget->sens_pos.segment(3,4)); break;
				case 'u': Ptarget->sens_pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0, delta*5)), Ptarget->sens_pos.segment(3,4)); break;
			}
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// state machine ////////////////////////////////////////////////////////////////////////////////////////////

		// picking state machine signals
		double eps_obj 		= 0.05; // if hands reached the destination
		double eps_rest		= 0.02; // if hands reached the default position

		// picking state machine
		switch(S)
		{
			case BALANCE: 
				if(G==TAKE)						{G=NONE; S=PICK_APPROACH;}
				if(G==RELEASE)					{G=NONE; S=DROP_APPROACH;} break;
			case PICK_APPROACH:
				if(manip.is_at_target(eps_obj))	{G=NONE; S=PICK; grasp=true;}
				if(G==HALT)						{G=NONE; S=PICK_STAND;} break;
			case PICK:
				if(Ptarget->grow < eps_obj)		{G=NONE; S=PICK_STAND; } 
				if(G==HALT)						{G=NONE; S=PICK_STAND;} break;
			case PICK_STAND:
				if(manip.is_at_rest(eps_rest))	{G=NONE; S=BALANCE;} break;
			case DROP_APPROACH:
				if(manip.is_at_target(eps_obj))	{G=NONE; S=DROP; grasp=false;} 
				if(G==HALT)						{G=NONE; S=DROP_STAND;} break;
			case DROP:
				if(Ptarget->grow > 1-eps_obj)	{G=NONE; S=DROP_STAND;}
				if(G==HALT)						{G=NONE; S=DROP_STAND;} break;
			case DROP_STAND:
				if(manip.is_at_rest(eps_rest))	{G=NONE; S=BALANCE;} break;
		}

		// define grasping behavior
		Ptarget->grow += ((grasp ? 0 : 1) - Ptarget->grow) * 1 * wrapper.dt;

		// define hand ideal positions
		VectorXd ideal(14);
		if(grasp)
			ideal = Ptarget->get_hand_ideal();
		else
			ideal = Pfront.get_hand_ideal();
		manip.set_ideal_point(ideal);

		// define next hand target positions
		VectorXd target(14);
		if(S==PICK_APPROACH || S==PICK)
			target = Ptarget->get_hand();
		else if(S==DROP_APPROACH || S==DROP)
			target = Ptarget->get_hand_moved(Pdrop);
		else
			target = ideal;
		manip.set_target_point(target);

		// check feasibility of the object, call collision right after the IK
		// after this point, points.model will be updated with ref_pos in the IK function, not the actual sens_pos
		points.load_tasks(des_com, des_lf, des_rf, target);
		VectorXd copy = joints.Ik_pos0;
		IK.solve(points, copy, joints.freeze);
		bool feasibility = SW==BALANCE && (IK.return_hand_error()< 0.17) && !collision.check(points);
		manip.update(feasibility, wrapper.dt, Ptarget, grasp);

		if(S==PICK_APPROACH || S==DROP_APPROACH)
			if((IK.return_hand_error()<0.13) && !collision.check(points) && SW!=BALANCE)
				GW = HALT;
			else if(((IK.return_hand_error()>0.17) || collision.check(points)) && SW==BALANCE)
				GW = GO;

		cout << S << "   " << G << "   |   " << SW << "   " << GW << "    |     " << 
				IK.return_hand_error() << "   " << collision.check(points) << endl;

		// walking state machine signals
		double eps_CoM 	= 0.005; // if com reached destination
		bool startOK  	= walking.early_phase(time, wrapper.dt) && walking.phase
						&& (des_com.segment(0,3) + com_adjustment - final_com_pos).norm() < eps_CoM;

		double eps_foot = 0.05; // if feet are close enough to stop
		bool stopOK  	= walking.early_phase(time, wrapper.dt)
						&& abs(points[CN_LF].p.pos[0] - points[CN_RF].p.pos[0]) < eps_foot;

		// walking state machine
		switch(SW)
		{
			case BALANCE: 
				if(GW==GO && manip.is_at_rest(eps_rest))	
					{
						GW=NONE; 
						SW=WALK_START; 
						com_adjustment = Vector3d(-0.015,0.025,0);
					} break;
			case WALK_START:
				if(startOK)				
					{
						GW=NONE; 
						SW=WALK; wrapper.rePID(true); 
						com_adjustment = Vector3d(-0.015,0,0); 
						walking.start=time;
					} break;
			case WALK:
				if(stopOK && GW==HALT)	
					{
						GW=NONE; 
						SW=BALANCE; wrapper.rePID(false); 
						com_adjustment = zero_v3;
					} break;
		}

		// other Cartesian tasks
		final_com_pos += (des_com.segment(0,3) + com_adjustment - final_com_pos) * 3 * wrapper.dt;
		points.load_tasks(vectorbig(final_com_pos,zero_quat), des_lf, des_rf, manip.get_hands());

		// whole-body IK 
		joints.ref_pos = joints.Ik_pos0;
		IK.solve(points, joints.ref_pos, joints.freeze);

		// walking stuff
		if(SW==WALK)
		{
			if(navigate) 
			{
				// here DS_nav determines reference walking velocities
				Vector3d x_dot = DS_nav.linear_DS(target.segment(0,7)/2.0+target.segment(7,7)/2.0);
				x_dot[0] *= x_dot[0]<0 ? 0.25 : 0.5;
				x_dot[1] *= 0.5;
				x_dot[2] *= 1.0;
				walking.demand_speeds(x_dot);
			}
			walking.cartesian_tasks(time, points);

			// impose lower-body motions and freeze the upper body
			joints.freeze.segment(24,14) = VectorXd::Ones(14);
			IK.solve(points, joints.ref_pos, joints.freeze);
			joints.freeze.segment(24,14) = VectorXd::Zero(14);

			// apply joint policies
			walking.joint_tasks(time, wrapper.dt, points, joints);
		}

		// compliant arm control
		double force = (1.0 - Ptarget->grow) * Ptarget->max_force;
		#ifdef HARDWARE
			if(time>calib_time)
				manip.compliance_control(points, force);
		#else
			manip.jacobian_transpose(points, joints, force);
		#endif

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// send final commands //////////////////////////////////////////////////////////////////////////
		double e = exp(-pow(time/1.0,2.0));  // WARNING
		joints.com_pos = joints.init_pos * e + joints.ref_pos * (1.0-e);
		wrapper.controlJoint(	joints.mode.segment(6,AIR_N_U-6), 
								joints.freeze.segment(6,AIR_N_U-6), 
								joints.com_pos.segment(6,AIR_N_U-6),
								joints.ref_tau.segment(6,AIR_N_U-6));
	}

	wrapper.initializeJoint(VectorXd::Zero(AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	wrapper.rePID(false);
	nonblock(0);
	wrapper.close();
	return 0;
}
