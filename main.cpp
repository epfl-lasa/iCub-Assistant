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
#include "IK.h"
#include "Walking.h"
#include "Collision.h"
#include "Navigation.h"
#include "Manipulation.h"

using namespace std;

/* Index reference and a natural posture of icub           */
/* global positions                                        */
/* pelvis pos:     pos[0] pos[1] pos[2]                    */
/* pelvis rot:     pos[3] pos[4] pos[5] pos[38]            */
/*                 // right      // left                   */
/* head                   pos[11]                          */
/* neck roll              pos[10]                          */
/* neck pitch             pos[9]                           */
/* shoulder pitch  pos[31]       pos[24]                   */
/* shoulder roll   pos[32]       pos[25]                   */
/* shoulder yaw    pos[33]       pos[26]                   */
/* elbow           pos[34]       pos[27]                   */
/* forearm yaw     pos[35]       pos[28]                   */
/* forearm roll    pos[36]       pos[29]                   */
/* forearm pitch   pos[37]       pos[30]                   */
/* torso pitch            pos[6]                           */
/* torso roll             pos[7]                           */
/* torso yaw              pos[8]                           */
/* hip pitch       pos[18]      pos[12]                    */
/* hip roll        pos[19]       pos[13]                   */
/* hip yaw         pos[20]       pos[14]                   */
/* knee            pos[21]       pos[15]                   */
/* ankle pitch     pos[22]       pos[16]                   */
/* ankle roll      pos[23]       pos[17]                   */

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

void rePID(Wrapper &wrapper, bool walk)
{
	#ifdef HARDWARE
		MatrixXd gain = MatrixXd(32,3);
		gain << 32000.00, 6000.00, 60.00,
				32000.00, 6000.00, 60.00,
				32000.00, 6000.00, 60.00,
				50.00, 500.00, 1.00,
				50.00, 500.00, 1.00,
				100.00, 700.00, 2.00,
				32000.00, 100.00, 60.00,
				-32000.00, -100.00, -60.00,
				32000.00, 100.00, 60.00, 
				-32000.00, -100.00, -60.00,
				-32000.00, -100.00, -60.00, 
				-32000.00, -100.00, -60.00, 
				32000.00, 100.00, 60.00, 
				-32000.00, -100.00, -60.00,
				32000.00, 100.00, 60.00, 
				-32000.00, -100.00, -60.00,
				-32000.00, -100.00, -60.00, 
				-32000.00, -100.00, -60.00, 
				32000.00, 50.00, 60.00, 
				32000.00, 50.00, 60.00, 
				10000.00, 0.00, 10.00, 
				32000.00, 20.00, 60.00,
				200.00, 1000.00, 1.00, 
				100.00, 100.00, 2.00, 
				100.00, 100.00, 2.00, 
				32000.00, 50.00, 60.00,
				32000.00, 50.00, 60.00, 
				10000.00, 0.00, 10.00, 
				32000.00, 20.00, 60.00, 
				200.00, 1000.00, 1.00,
				100.00, 100.00, 2.00,
				100.00, 100.00, 2.00;

		for(int i=12;i<38;i++)
			wrapper.setPidJoint(i-6,gain(i-6,0)*1, gain(i-6,1)*1, gain(i-6,2)*0);

		if(walk)
		{
			// relax ankle roll joints
			int i = 17; 
			wrapper.setPidJoint(i-6,gain(i-6,0)*0.3, gain(i-6,1)*0.3, gain(i-6,2)*0);
			i = 23;
			wrapper.setPidJoint(i-6,gain(i-6,0)*0.3, gain(i-6,1)*0.3, gain(i-6,2)*0);
		}
	#else
		// hips
		wrapper.setPidJoint(13-6, 10, 0.6, 0);
		wrapper.setPidJoint(19-6, 10, 0.6, 0);
		wrapper.setPidJoint(12-6, 10, 0.6, 0);
		wrapper.setPidJoint(18-6, 10, 0.6, 0);
		
		// knees
		wrapper.setPidJoint(15-6, 10, 0.3, 0);
		wrapper.setPidJoint(21-6, 10, 0.3, 0);

		// ankles
		if(walk)
		{
			wrapper.setPidJoint(14-6, 3, 0.3, 0);
			wrapper.setPidJoint(20-6, 3, 0.3, 0);
			wrapper.setPidJoint(16-6, 3, 0.1, 0);
			wrapper.setPidJoint(22-6, 3, 0.1, 0);
			wrapper.setPidJoint(17-6, 3, 0.1, 0);
			wrapper.setPidJoint(23-6, 3, 0.1, 0);
		}
		else
		{
			wrapper.setPidJoint(14-6, 10, 0.3, 0);
			wrapper.setPidJoint(20-6, 10, 0.3, 0);
			wrapper.setPidJoint(16-6, 10, 0.3, 0);
			wrapper.setPidJoint(22-6, 10, 0.3, 0);
			wrapper.setPidJoint(17-6, 10, 0.3, 0);
			wrapper.setPidJoint(23-6, 10, 0.3, 0);
		}

	#endif
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
		return 1;
	wrapper.initialize();
	rePID(wrapper, false);

	// Initialize reading robot pos from gazebo/mocap
	#ifdef HARDWARE

	#else
		wrapper.initObject(0, wrapper.robotName + "/get_root_link_WorldPose:o");
	#endif

	// Initialize reading object pos from gazebo/mocap
	#ifdef HARDWARE
		wrapper.initObject(1, "/icub/DesiredCoMVelocity:o");
	#else
		wrapper.initObject(1, "/BoxSmall/GraspedObject_WorldPose:o");
	#endif

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
	IK ik;
	double IK_time1, IK_time2, IK_time3;

	// self collision avoidance
	Collision collision;

	// contact definition
	Contact_Manager points;

	// set position/torque control modes, update limits
	wrapper.initializeJoint(joints.mode.segment(6,AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	wrapper.getJointLimits(&points.model.qmin[0], &points.model.qmax[0]);
	points.model.qmax[15-6] = -10;
	points.model.qmax[21-6] = -10;
	points.model.qmin[25-6] = 10;
	points.model.qmin[32-6] = 10;

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

	// Object properties
	Object object;
	object.sens_pos = des_obj;
	object.dim = Vector3d(0.2, 0.2, 0.2);
	object.expansion = 0.1;
	object.grasp_axis = 1;
	#ifdef HARDWARE
		object.top_marker = true;
	#endif

	// grasping controller
	Manipulation manip;
	manip.set_start_point(des_obj);
	manip.set_ideal_point(des_obj);

	// prepare for reading the keyboard
	nonblock(1);

	// start the loop
	double calib_time = 3;  // WARNING
	state S = BALANCE;
	trigger G = NONE;
	bool navigate = false;
	bool follow = false;
	bool grasp = false;
	double start_time = wrapper.time;
	while(!exit_sim)
	{
		//////////////////////////////////////////////////// sensing ////////////////////////////////////////////////	
		// get time
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
		#ifndef HARDWARE
			Root.segment(3,4) = quat_mul(Root.segment(3,4), ang2quat(Vector3d(0,0,M_PI))); 
		#endif
		// do whatever transform necessary to find root's pos/rot

		// object's world position
		VectorXd Base = points.model.get_trans7(0,zero_v3);
		object.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(1));

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// after this point, points.model will be updated with ref_pos in the IK function, not the actual sens_pos
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// check feasibility of the object, call collision right after the IK
		points.load_tasks(des_com, des_lf, des_rf, object.get_hand(true), object.get_hand(false));
		bool feasibility = false;
		if(S!=WALK)
		{
			// check if the object is feasible to grasp, i.e. reachable and w/o collisions
			VectorXd copy = joints.Ik_pos0;
			IK_time1 = ik.solve(points, copy, joints.freeze);
			double IK_error = ik.return_hand_error();
			feasibility = (IK_error< 0.17) && !collision.check(points);
		}

		// read keyboard signals //////////////////////////////////////////////////////////////////////
		double delta = 0.01;
		if(khbit() != 0)
		{
			char c = fgetc(stdin);
			fflush(stdin);
			switch(c)
			{
				// general signals
				case 'g': G = GO; break;
				case 'h': G = HALT; break;
				case 't': G = TAKE; break;
				case 'r': G = RELEASE; break;
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
				case 'i': des_obj[0] += delta; break;
				case 'k': des_obj[0] -= delta; break;
				case 'j': des_obj[1] += delta; break;
				case 'l': des_obj[1] -= delta; break;
				case 'p': des_obj[2] += delta; break;
				case ';': des_obj[2] -= delta; break;
				case 'o': des_obj.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0,-delta*5)), des_obj.segment(3,4)); break;
				case 'u': des_obj.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0, delta*5)), des_obj.segment(3,4)); break;
			}
		}
		manip.set_ideal_point(des_obj);

		// state machine //////////////////////////////////////////////////////////////////////////////
		double eps_CoM = 0.005;
		double eps_obj = 0.02;
		double eps_foot = 0.05;
		bool reached = (manip.xR-object.sens_pos.segment(0,3)).norm()<eps_obj; // if hands reached the destination
		bool upright = (manip.xR-manip.xd).norm()<eps_obj; // if hands reached the destination
		bool balanced = (des_com.segment(0,3) + com_adjustment - final_com_pos).norm()<eps_CoM;
		switch(S)
		{
			case BALANCE:
				if(G==GO && upright)
				{
					G = NONE;
					S = WALK_START;
					com_adjustment = Vector3d(-0.015,0.025,0);
				}
				if(G==TAKE)
				{
					G = NONE;
					S = PICK_APPROACH;
				}
				if(G==RELEASE)
				{
					grasp = false;
					G = NONE;
				}
				break;
			case PICK_APPROACH:
				if(reached || G==HALT)
				{
					G = NONE;
					S = PICK;
					grasp = true;
				}
				if(G==RELEASE)
				{
					G = NONE;
					S = DROP_APPROACH;
				}
				break;
			case PICK:
				if(manip.grasped(eps_obj))
				{
					G = NONE;
					S = PICK_STAND;
				}
				break;
			case PICK_STAND:
				if(reached)
				{
					G = NONE;
					S = BALANCE;
				}
				break;
			case DROP_APPROACH:
				if(reached)
				{
					G = NONE;
					S = DROP;
					grasp = false;
				}
				break;
			case DROP:
				if(manip.released(eps_obj))
				{
					G = NONE;
					S = DROP_STAND;
				}
				break;
			case DROP_STAND:
				if(reached)
				{
					G = NONE;
					S = BALANCE;
				}
				break;
			case WALK_START:
				if(balanced && walking.phase && walking.early_phase(time, wrapper.dt))
				{
					G = NONE;
					S = WALK;
					walking.walk_start = time;
					com_adjustment = Vector3d(-0.015,0,0);
					rePID(wrapper, true);
				}
				break;
			case WALK:
				if(G==HALT && walking.early_phase(time, wrapper.dt) && std::abs(points[CN_LF].p.pos[0]-points[CN_RF].p.pos[0]) < eps_foot)
				{
					G = NONE;
					S = BALANCE;
					com_adjustment = zero_v3;
					rePID(wrapper, false);
				}
				break;
		}

		// hand dynamical systems ////////////////////////////////////////////////////////////////////////////
		manip.update(feasibility, wrapper.dt, object, grasp);

		// Cartesian tasks ////////////////////////////////////////////////////////////////////////////////
		final_com_pos += (des_com.segment(0,3) + com_adjustment - final_com_pos) * 3 * wrapper.dt;
		points.load_tasks(vectorbig(final_com_pos,zero_quat), des_lf, des_rf, manip.get_hand(object, true), manip.get_hand(object, false));
		#ifdef HARDWARE
			if(time>calib_time) // WARNING
				manip.compliance_control(points);
		#else
			manip.jacobian_transpose(points, joints);
		#endif


		// whole-body IK ////////////////////////////////////////////////////////////////////////////////
		joints.ref_pos = joints.Ik_pos0;
		IK_time2 = ik.solve(points, joints.ref_pos, joints.freeze);

		// walking stuff
		if(S==WALK)
		{
			if(navigate) 
			{
				// here DS_nav determines reference walking velocities
				Vector3d x_dot = DS_nav.linear_DS(object.sens_pos);
				x_dot[0] *= x_dot[0]<0 ? 0.25 : 0.5;
				x_dot[1] *= 0.5;
				x_dot[2] *= 1.0;
				walking.demand_speeds(x_dot);
			}
			walking.cartesian_tasks(time, points);

			// impose lower-body motions and freeze the upper body
			joints.freeze.segment(24,14) = VectorXd::Ones(14);
			IK_time3 = ik.solve(points, joints.ref_pos, joints.freeze);
			joints.freeze.segment(24,14) = VectorXd::Zero(14);

			// apply joint policies
			walking.joint_tasks(time, wrapper.dt, points, joints);
		}

		// send final commands //////////////////////////////////////////////////////////////////////////
		double e = exp(-pow(time/1.0,2.0));  // WARNING
		joints.com_pos = joints.init_pos * e + joints.ref_pos * (1.0-e);
		wrapper.controlJoint(	joints.mode.segment(6,AIR_N_U-6), 
								joints.freeze.segment(6,AIR_N_U-6), 
								joints.com_pos.segment(6,AIR_N_U-6),
								joints.ref_tau.segment(6,AIR_N_U-6));
	}

	wrapper.initializeJoint(VectorXd::Zero(AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	rePID(wrapper, false);
	nonblock(0);
	wrapper.close();
	return 0;
}
