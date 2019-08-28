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

enum object_state {EMPTY=0, PICK_APPROACH, PICK, PICK_STAND, HOLD, DROP_APPROACH, DROP, DROP_STAND};
enum trigger {NONE=0, GO, HALT, TAKE, RELEASE};

// robot's default posture
# define des_com vectorbig(Vector3d(0.0, 0.0, 0.53), zero_quat)
# define des_obj vectorbig(Vector3d(0.2, 0.0, 0.65), zero_quat)
# define des_lf vectorbig(Vector3d(0.0, 0.11, 0.0), zero_quat)
# define des_rf vectorbig(Vector3d(0.0,-0.11, 0.0), zero_quat)

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

	// list of objects sensed from the environment
	Object BoxSmall(des_obj);
	BoxSmall.name = "BoxSmall";
	BoxSmall.max_expansion = 0.1;
	BoxSmall.max_force = 15;
	#if defined(ICUB5) || defined(ICUB33)
		BoxSmall.dim = Vector3d(0.16, 0.23, 0.13);
		BoxSmall.top_marker = true;
		BoxSmall.proximity = 0.05;
	#elif defined(ICUBSIM)
		BoxSmall.dim = Vector3d(0.2, 0.2, 0.2);
		BoxSmall.proximity = 0.1;
	#endif

	Object BoxBig(des_obj);
	BoxBig.name = "BoxBig";
	BoxBig.dim = Vector3d(1.0, 0.2, 0.2);
	BoxBig.max_expansion = 0.1;
	BoxBig.max_force = 10;
	#if defined(ICUB5) || defined(ICUB33)
		BoxBig.top_marker = true;
	#endif 
	
	Object Broom(vectorbig(Vector3d(0.3, 0.0, 0.7), zero_quat));
	Broom.name = "Broom";
	Broom.dim = Vector3d(0.02, 0.02, 0.4);
	Broom.max_expansion = 0.1;
	Broom.max_force = 0;
	Broom.hand_offset = Vector3d(0,0.1,0);
	Broom.curvature = 2.0;
	Broom.proximity = 0.03;

	Object Cart(vectorbig(Vector3d(0.2, 0.0, 0.54), zero_quat));
	Cart.name = "Cart";
	Cart.dim = Vector3d(0.02, 0.4, 0.02);
	Cart.max_expansion = 0.1;
	Cart.max_force = 0;
	Cart.hand_offset = Vector3d(0,0.1,0);
	Cart.grasp_opposite = false;
	Cart.ideal_grasp_axis = Vector3d(0,0,1);
	Cart.curvature = 2.0;
	Cart.proximity = 0.03;

		// YARP Wrapper
	Wrapper wrapper;
	if(wrapper.checkRobot(argc, argv))
		{cout << "Robot name problem" << endl; return 1;}
	wrapper.initialize();
	wrapper.rePID(false);

	// Initialize reading robot/object pos from gazebo/mocap
	#if defined(ICUB5) || defined(ICUB33)
		wrapper.initObject(0, wrapper.robotName + "/get_root_link_WorldPose:o");
		wrapper.initObject(1, "/BoxSmall/GraspedObject_WorldPose:o");
		wrapper.initObject(2, "/BoxBig/GraspedObject_WorldPose:o");
		wrapper.initObject(3, "/Broom/GraspedObject_WorldPose:o");
		wrapper.initObject(4, "/Cart/GraspedObject_WorldPose:o");

		wrapper.Object[0] = des_com;
		wrapper.Object[1] = BoxSmall.ideal_pos;
		wrapper.Object[2] = BoxBig.ideal_pos;
		wrapper.Object[3] = Broom.ideal_pos;
		wrapper.Object[4] = vectorbig(Vector3d(0.25, 0.0, 0.5), zero_quat);
	#elif defined(ICUBSIM)
		wrapper.initObject(0, wrapper.robotName + "/get_root_link_WorldPose:o");
		wrapper.initObject(1, "/BoxSmall/GraspedObject_WorldPose:o");
		wrapper.initObject(2, "/BoxBig/GraspedObject_WorldPose:o");
		wrapper.initObject(3, "/Broom/GraspedObject_WorldPose:o");
		wrapper.initObject(4, "/Cart/GraspedObject_WorldPose:o");
	#endif
	yarp::os::Time::delay(1);
	for(int i=0; i<=4; i++)
		if(wrapper.readObject(i).isZero())
			{
				cout << "Error: Object " << i << " mocap problem!" << endl; 
				return 1;
			}

	// logger
	std::ofstream OutRecord;
	string path_log = "log_icub.txt";
	OutRecord.open(path_log.c_str());

	// inverse kinematics
	InverseKinematics IK;

	// self collision avoidance
	Collision collision;

	// walking controller
	Walking walking;

	// navigation DS
	Navigation navigation;

	// contact definition
	Contact_Manager points;

	// joint variables
	Joints joints;

	// set entire body to position-direct mode
	for(int i=6;i<AIR_N_U;i++) 
		joints.mode[i] = 1;

	// set arms to force control in simulation
	#if defined(ICUBSIM)
		for(int i=24;i<24+14;i++) 
			joints.mode[i] = 2;
	#endif

	// disable the neck joints
	for(int i=9;i<12;i++)
		joints.freeze[i] = 1;

	// update joint limits
	wrapper.initializeJoint(joints.mode.segment(6,AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	wrapper.getJointLimits(&points.model.qmin[0], &points.model.qmax[0]);
	points.model.qmax[36-6] -= 10;
	points.model.qmax[29-6] -= 10;
	points.model.qmin[32-6] += 10;
	points.model.qmin[25-6] += 10;

	// read data once
	yarp::os::Time::delay(1);
	loadData(wrapper, points, joints);
	joints.init_pos = joints.sens_pos;

	// Object properties, by default facing forward
	Object Front(des_obj);
	Front.sens_pos = des_obj;
	Front.dim = Vector3d(0.2, 0.2, 0.2);
	Front.max_expansion = 0.1;
	Front.max_force = 0;
	Front.name = "Front";
	Object Joystick = Front;
	Joystick.name = "Joystick";

	// grasping controller
	Manipulation manip;
	VectorXd Pdrop = des_obj;
	manip.set_ideal_point(Front.get_hand_ideal());
	manip.set_start_point();

	// prepare for reading the keyboard
	nonblock(1);

	// start the loop
	Object *Ptarget = &Joystick;
	double calib_time = 10;
	object_state task = EMPTY;
	trigger signal = NONE;
	bool navigate = false;
	bool follow = false;
	bool user_grasp = false;
	bool user_walk = false;
	bool picking_walk = false;
	double start_time = wrapper.time;
	while(!exit_sim)
	{
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// sensing //////////////////////////////////////////////////////////////////////////////////////////////////
		double time = wrapper.time - start_time;
		cout << time << "  |   ";
		
		// read sensors
		loadData(wrapper, points, joints);

		// hand force sensor filtering
		if(time<calib_time) 
			manip.initial_filtering(points, wrapper.dt);
		else
			manip.secondary_filtering(points, wrapper.dt);

		// robot position
		VectorXd Root = wrapper.readObject(0);
		// do whatever transform necessary to find root's pos/rot
		#ifdef ICUB5
			Root.segment(0,3) += quat2dc(Root.segment(3,4)) * Vector3d(0.05, -0.05, 0.05);
		#elif defined(ICUB33)
			Root.segment(0,3) += quat2dc(Root.segment(3,4)) * Vector3d(0.05, -0.05, 0.05);
		#elif defined(ICUBSIM)
			Root.segment(3,4) = quat_mul(Root.segment(3,4), ang2quat(Vector3d(0,0,M_PI))); 
		#endif

		OutRecord << time << "   " << Root.segment(0,3).transpose() << "  " << 
					quat2ang(Root.segment(3,4)).transpose() << endl;

		// object world positions
		VectorXd Base = points.model.get_trans7(0,zero_v3);
		BoxSmall.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(1));
		BoxBig.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(2));
		Broom.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(3));
		Cart.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(4));

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// picking state machine ////////////////////////////////////////////////////////////////////////////////////

		// read keyboard signals
		double delta = 0.01;
		if(khbit() != 0)
		{
			char c = fgetc(stdin);
			fflush(stdin);
			switch(c)
			{
				// general signals
				case 't': signal = TAKE; break;
				case 'r': signal = RELEASE; break;
				case 'x': signal = HALT; break;
				case 'n': navigate = !navigate; break;
				case 'f': follow = !follow; break;
				case 'm': user_grasp = !user_grasp; break;
				case 'g': user_walk = !user_walk; break;

				case '0': Ptarget = &Joystick; break;
				case '1': Ptarget = &BoxSmall; break;
				case '2': Ptarget = &BoxBig; break;
				case '3': Ptarget = &Broom; break;
				case '4': Ptarget = &Cart; break;

				// desired velocities
				case 'w': walking.ref_vx += 0.1; break;
				case 's': walking.ref_vx -= 0.1; break;
				case 'a': walking.ref_vy += 0.1; break;
				case 'd': walking.ref_vy -= 0.1; break;
				case 'q': walking.ref_w += 0.1; break;
				case 'e': walking.ref_w -= 0.1; break;

				case 'c': wrapper.graspLeft(true); wrapper.graspRight(true); break;
				case 'v': wrapper.graspLeft(false); wrapper.graspRight(false); break;

				// desired object position
				case 'i': Joystick.ideal_pos[0] += delta; break;
				case 'k': Joystick.ideal_pos[0] -= delta; break;
				case 'j': Joystick.ideal_pos[1] += delta; break;
				case 'l': Joystick.ideal_pos[1] -= delta; break;
				case 'p': Joystick.ideal_pos[2] += delta; break;
				case ';': Joystick.ideal_pos[2] -= delta; break;
				case 'o': Joystick.ideal_pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0,-delta*5)), Joystick.ideal_pos.segment(3,4)); break;
				case 'u': Joystick.ideal_pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0, delta*5)), Joystick.ideal_pos.segment(3,4)); break;
			}
		}

		// state machine
		double eps1	= 0.5 * Ptarget->max_expansion; // if hands reached the point close to the object
		double eps2	= 2.0 * Ptarget->proximity; // if hands reached the point close to the object
		double eps_rest	= 0.02; // if hands reached the default position
		switch(task)
		{
			case EMPTY: 
				if(signal==TAKE)				{signal=NONE; task=PICK_APPROACH;}
				if(signal==HALT)				{signal=NONE; task=EMPTY;} break;
			case PICK_APPROACH:
				if(manip.is_at_target(eps1))	{signal=NONE; task=PICK;}
				if(signal==HALT)				{signal=NONE; task=DROP_STAND;} break;
			case PICK:
				if(Ptarget->grow < 0.1 &&
				manip.is_at_target(eps2))		{signal=NONE; task=PICK_STAND; } 
				if(signal==HALT)				{signal=NONE; task=DROP_STAND;} break;
			case PICK_STAND:
				if(manip.is_at_rest(eps_rest))	{signal=NONE; task=HOLD;} break;
			case HOLD: 
				if(signal==RELEASE)				{signal=NONE; task=DROP_APPROACH;}
				if(signal==HALT)				{signal=NONE; task=HOLD;} break;
			case DROP_APPROACH:
				if(manip.is_at_target(eps1))	{signal=NONE; task=DROP;} 
				if(signal==HALT)				{signal=NONE; task=PICK_STAND;} break;
			case DROP:
				if(Ptarget->grow > 0.9)			{signal=NONE; task=DROP_STAND;}
				if(signal==HALT)				{signal=NONE; task=DROP_STAND;} break;
			case DROP_STAND:
				if(manip.is_at_rest(eps_rest))	{signal=NONE; task=EMPTY;} break;
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// tasks ////////////////////////////////////////////////////////////////////////////////////////////////////

		// define grasping behavior
		bool grasp = task==PICK || task==PICK_STAND || task==HOLD || task==DROP_APPROACH || user_grasp;
		Ptarget->grow += ((double(grasp) ? 0 : 1) - Ptarget->grow) * 5.0 * wrapper.dt;

		// define hand ideal positions
		VectorXd ideal = Front.get_hand_ideal();
		if(grasp && !Ptarget->sens_pos.segment(0,3).isZero())
			ideal = Ptarget->get_hand_ideal();
		manip.set_ideal_point(ideal);

		// define next hand target positions
		VectorXd target = ideal;
		if(!Ptarget->sens_pos.segment(0,3).isZero())
			if(task==PICK_APPROACH || task==PICK)
				target = Ptarget->get_hand();
			else if(task==DROP_APPROACH || task==DROP)
				target = Ptarget->get_hand_moved(Pdrop);
		manip.set_target_point(target);

		// check feasibility of the object, call collision right after the IK
		// after this point, points.model will be updated with ref_pos in the IK function, not the actual sens_pos
		points.load_tasks(des_com, des_lf, des_rf, target);
		VectorXd copy = joints.Ik_pos0;
		IK.solve(points, copy, joints.freeze);
		bool feasibility = walking.state==IDLE && (IK.return_hand_error()< Ptarget->proximity*2.0) && !collision.check(points);
		manip.update(feasibility, wrapper.dt, Ptarget, grasp);
		points.load_tasks(des_com, des_lf, des_rf, manip.get_hands());

		cout << Ptarget->name << "   |   " <<  task << "   " << signal << "   |   " << 
				IK.return_hand_error() << "   " << collision.check(points) << "    |     ";

		// communication between picking and walking state machines
		if(task==PICK_APPROACH || task==DROP_APPROACH)
			if((IK.return_hand_error()>Ptarget->proximity*2.0) || collision.check(points))
				picking_walk = true;
		if((IK.return_hand_error()<Ptarget->proximity) && !collision.check(points))
				picking_walk = false;

		//compliant arm control
		double force = (1.0 - Ptarget->grow) * Ptarget->max_force;
		#if defined(ICUB5) || defined(ICUB33)
			if(time>calib_time)
				manip.compliance_control(points, force);
		#elif defined(ICUBSIM)
			manip.jacobian_transpose(points, joints, force);
		#endif

		// whole-body IK 
		joints.ref_pos = joints.Ik_pos0;
		IK.solve(points, joints.ref_pos, joints.freeze);

		// walking task
		if(navigate && walking.state == WALK) 
		{
			// here navigation determines reference walking velocities
			// Vector3d x_dot = navigation.linear_DS(target.segment(0,7)/2.0+target.segment(7,7)/2.0);
			// navigation with learned lpvds
			Vector3d x_dot = navigation.nonlinear_DS(wrapper.readObject(1), Root);
			x_dot[0] *= x_dot[0]<0 ? 0.25 : 1;
			x_dot[1] *= 1.0;
			x_dot[2] *= 1.0;

			cout << endl <<  "---------xdot-ds: " << x_dot.transpose() << endl;
			walking.demand_speeds(x_dot, wrapper.dt);
		}
		bool demand = (task==EMPTY || task==PICK_APPROACH || task==HOLD || task==DROP_APPROACH)
					&& manip.is_at_rest(eps_rest) && (picking_walk || user_walk);
		walking.update(time, wrapper.dt, demand, points, joints, wrapper);

		cout << walking.state << "   " << demand << "  " << picking_walk << "  " << navigate << "  |  ";
		cout << endl;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// send final commands //////////////////////////////////////////////////////////////////////////
		double e = exp(-pow(time/3.0,2.0));  // WARNING
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
