#include "Walking.h"

Vector4d projection_gains(double t, double T)
{
	// mass = 27.6, height = 1.05, iCub Robot
	double time = t/T;
	if (std::abs(T-0.4)<0.1)
	{
		// 0 <= time <= 1 : phase time
		// freq = 2.5 & log10(gain) = 1.1
		double g1[4] = {
			exp(-std::pow(time/0.595639,1.0)),
			exp(-std::pow(time/0.434266,1.0)),
			exp(-std::pow(time/0.458892,1.0)),
			exp(-std::pow(time/0.636159,1.0)),
			};
		double g2[4] = {
			exp(-std::pow((1.0-time)/0.1671792 ,2.0)),
			exp(-std::pow((1.0-time)/0.2202462 ,2.0)),
			exp(-std::pow((1.0-time)/0.5009862 ,2.0)),
			exp(-std::pow((1.0-time)/0.1700142 ,2.0)),
			};
		double K[4] = {
			0.228948 * g1[0] + 0.955843 * g2[0],
			2.538919 * g1[1] + -1.246919 * g2[1],
			0.033176 * g1[2] + 0.010854 * g2[2],
			0.640404 * g1[3] + -0.160717 * g2[3],
			};
		Vector4d KK(K);
		return KK;
	}
	return VectorXd::Zero(4);
}

double deadzone(double e,double dz)
{
    return e-atan(e*M_PI/2.0/dz)/M_PI*2.0*dz;
}

Walking::Walking()
{
	state_filt.init(10, 5);
	lp3_state = VectorXd::Zero(10);
	lp3_state = VectorXd::Zero(10);
	shift = 0.0;
	fblx = 0.0;
	fbrx = 0.0;
	fbly = 0.0;
	fbry = 0.0;
	Tstep = 0.4;
	tphase = 0.0;
	left_support = 0.0;
	ref_vx = 0.0;
	ref_vy = 0.0;
	ref_w = 0.0;
	force_lf = 0.0;
	force_rf = 0.0;
	hip_gain_K = 1.0;
	start_time = 0;
	state = IDLE;
	com_adjustment = zero_v3;
	torso_cost = -2;
}

void Walking::update(double time, double dt, bool demand, Contact_Manager &points, Joints &joints, Wrapper &wrapper)
{
	tphase = fmod(time,Tstep);
	left_support = fmod(time,2.0*Tstep) > Tstep; // 1: left support, right stateing

	// decay the desired velocities 
	ref_vx *= 1.0 - 1.0 * dt;
	ref_vy *= 1.0 - 1.0 * dt;
	ref_w  *= 1.0 - 1.0 * dt;

	// foot force estimation, determines ratio
	force_lf += (max(0.0, -points[CN_LF].T.F_sens[2]) - force_lf) * 40.0 * dt;
	force_rf += (max(0.0, -points[CN_RF].T.F_sens[2]) - force_rf) * 40.0 * dt;
	al = force_lf / (force_lf + force_rf);
	ar = force_rf / (force_lf + force_rf);

	// Pelvis to CoM shift filter
	shift += (points[CN_CM].ref_p.pos[0] - joints.ref_pos[0] - shift) * 5.0 * dt;

	// walking transition stuff
	Vector3d adjustment = double(state==WALK_START || state==WALK) * Vector3d(-0.02,0,0) 
						+ double(state==WALK_START) * Vector3d(0,0.025,0);
	com_adjustment += (adjustment - com_adjustment) * 5.0 * dt;
	
	// keep torso upright during walking
	double target = (state==IDLE || state==WALK_STOP) ? -2 : 2;
	torso_cost += (target - torso_cost) * 2.0 * dt;

	// walking state machine signals
	bool steppingOK = early_phase(time, dt) && left_support
					&& (adjustment - com_adjustment).norm() < 0.02 * 0.01;
	bool stopOK  	= early_phase(time, dt)
					&& abs(points[CN_LF].p.pos[0] - points[CN_RF].p.pos[0]) < 0.05; // if feet are close enough to stop
	bool idleOK  = abs(target - torso_cost) < 4.0 * 0.01; // if torso is relax enough

	// walking state machine
	switch(state)
	{
		case IDLE: 
			if(demand)				{state=WALK_START;} break;
		case WALK_START:
			if(steppingOK)			{state=WALK; wrapper.rePID(true); start_time=time;} break;
		case WALK:
			if(stopOK && !demand)	{state=WALK_STOP; wrapper.rePID(false);} break;
		case WALK_STOP:
			if(idleOK)				{state=IDLE;} break;
	}

	cartesian_tasks(time, points, joints);
	joint_tasks(time, wrapper.dt, points, joints);
}

void Walking::calculate_footstep_adjustments(double time, Contact_Manager &points, Joints &joints)
{
	// step width
	double step_width = points[CN_LF].ref_p.pos[1] - points[CN_RF].ref_p.pos[1];

	// Cartesian position/velocity estimation by filtering
	lp3_state[0] = time;
	lp3_state.segment(1,3) = points[CN_LF].p.pos.segment(0,3) + Vector3d(0,-step_width/2.0,0);
	lp3_state.segment(4,3) = joints.sens_pos.segment(0,3) + Vector3d(shift,0,0) - com_adjustment;
	lp3_state.segment(7,3) = points[CN_RF].p.pos.segment(0,3) + Vector3d(0,step_width/2.0,0);
	VectorXd lp3_pstate = state_filt.x[(state_filt.index+1)%state_filt.window];
	state_filt.update(lp3_state);
	lp3_dstate = (lp3_state - lp3_pstate)/std::max(minTimeStep,lp3_state[0]-lp3_pstate[0]);	

	Vector4d elx(	lp3_state[1]-lp3_state[7],   lp3_state[4]-lp3_state[7], 
					lp3_dstate[1]-lp3_dstate[7], lp3_dstate[4]-lp3_dstate[7]); //right sup.
	Vector4d erx(	lp3_state[7]-lp3_state[1],   lp3_state[4]-lp3_state[1], 
					lp3_dstate[7]-lp3_dstate[1], lp3_dstate[4]-lp3_dstate[1]); //left  sup.

	Vector4d ely(	lp3_state[2]-lp3_state[8],   lp3_state[5]-lp3_state[8], 
					lp3_dstate[2]-lp3_dstate[8], lp3_dstate[5]-lp3_dstate[8]);
	Vector4d ery(	lp3_state[8]-lp3_state[2],   lp3_state[5]-lp3_state[2], 
					lp3_dstate[8]-lp3_dstate[2], lp3_dstate[5]-lp3_dstate[2]);

	// dead-zone functions to filter noise
	#ifdef HARDWARE
		elx[0] = deadzone(elx[0]-(0.000), 0.001);
		elx[1] = deadzone(elx[1]-(-0.016), 0.002);
		elx[2] = deadzone(elx[2]-(0.003), 0.022);
		elx[3] = deadzone(elx[3]-(0.003), 0.018);
		erx[0] = deadzone(erx[0]-(-0.001), 0.001);
		erx[1] = deadzone(erx[1]-(-0.018), 0.001);
		erx[2] = deadzone(erx[2]-(0.003), 0.021);
		erx[3] = deadzone(erx[3]-(-0.000), 0.019);
		ely[0] = deadzone(ely[0]-(-0.034), 0.017);
		ely[1] = deadzone(ely[1]-(-0.020), 0.032);
		ely[2] = deadzone(ely[2]-(0.031), 0.216);
		ely[3] = deadzone(ely[3]-(-0.232), 0.147);
		ery[0] = deadzone(ery[0]-(0.042), 0.010);
		ery[1] = deadzone(ery[1]-(0.023), 0.033);
		ery[2] = deadzone(ery[2]-(0.032), 0.178);
		ery[3] = deadzone(ery[3]-(0.266), 0.145);
	#endif
	
	// smoothening the errors
	Vector4d ex = left_support==1 ? erx : elx;
	Vector4d ey = left_support==1 ? ery : ely;
	ex *= 1.0 - exp(-pow(tphase/(Tstep/3.0),2.0));
	ey *= 1.0 - exp(-pow(tphase/(Tstep/3.0),2.0));

	// projection gains			
	Vector4d K = projection_gains(tphase, Tstep);

	// footstep position feedbacks wide range
	fblx = left_support==1 ? fblx : truncate_command(double(ex.transpose()*K), 0.3, -0.3);
	fbrx = left_support==0 ? fbrx : truncate_command(double(ex.transpose()*K), 0.3, -0.3);
	fbly = left_support==1 ? fbly : truncate_command(double(ey.transpose()*K), 0.2, -0.2);
	fbry = left_support==0 ? fbry : truncate_command(double(ey.transpose()*K), 0.2, -0.2);

	// self collision avoidance
	// fbly = max(fbly, lp3_state[8]-lp3_state[5] - 0.1*0);
	// fbry = min(fbry, lp3_state[2]-lp3_state[5] + 0.1*0);
}

void Walking::apply_speed_limits()
{
	#ifdef HARDWARE
		ref_vx = truncate_command(ref_vx, 0.1, -0.1);
		ref_vy = truncate_command(ref_vy, 0.1, -0.1);
		ref_w  = truncate_command(ref_w , 0.5, -0.5);
	#else
		ref_vx = truncate_command(ref_vx, 0.3, -0.3);
		ref_vy = truncate_command(ref_vy, 0.3, -0.3);
		ref_w  = truncate_command(ref_w , 1.0, -1.0);
	#endif
}

void Walking::demand_speeds(Vector3d speeds)
{
	ref_vx = speeds[0];
	ref_vy = speeds[1];
	ref_w  = speeds[2];
}

void Walking::cartesian_tasks(double time, Contact_Manager &points, Joints &joints)
{
	// speed limits
	apply_speed_limits();

	// feet tasks during walking
	if(state==WALK)
	{
		// foot lifting control (footstepping is done in the joint space after the IK)
		double liftL = max(0.0, sin(M_PI*time/Tstep));
		double liftR = max(0.0, sin(M_PI*time/Tstep+M_PI));
		points[CN_LF].ref_p.pos.segment(0,3) += Vector3d(0, 0.0, liftL * 0.02);
		points[CN_RF].ref_p.pos.segment(0,3) += Vector3d(0, 0.0, liftR * 0.02);

		// foot orientation correction, should use quat formulas in future
		points[CN_LF].ref_p.pos.segment(3,4) = ang2quat(-quat2ang(points[CN_LF].p.pos.segment(3,4))*1);
		points[CN_RF].ref_p.pos.segment(3,4) = ang2quat(-quat2ang(points[CN_RF].p.pos.segment(3,4))*1);

		// turning velocity control control
		points[CN_LF].ref_p.pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0,ref_w*0.3 * ar)), points[CN_LF].ref_p.pos.segment(3,4));
		points[CN_RF].ref_p.pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0,ref_w*0.3 * al)), points[CN_RF].ref_p.pos.segment(3,4));

		// sagittal/lateral velocity control
		points[CN_CM].ref_p.pos.segment(0,3) += Vector3d(0.03*ref_vx, 0.03*ref_vy, 0);
	}

	// CoM tasks in walking and transition to walking
	if(state!=IDLE)
	{
		points[CN_CM].ref_p.pos.segment(0,3) += com_adjustment;

		Vector3d cost1 = points[CN_CM].R.slack_cost;
		Vector3d cost2 = points[CN_TO].R.slack_cost;
		double cost3 = points[CN_CM].T.slack_cost[2];
		points[CN_CM].R.slack_cost = Vector3d(1,1,1) * pow(10.0, torso_cost);
		points[CN_TO].R.slack_cost = Vector3d(1,1,1) * pow(10.0, torso_cost);
		points[CN_CM].T.slack_cost[2] = pow(10.0, torso_cost);

		// impose all these lower-body motions while freezing the upper body to avoid influence on the arms
		joints.freeze.segment(24,14) = VectorXd::Ones(14);
		InverseKinematics IK;
		IK.solve(points, joints.ref_pos, joints.freeze);
		joints.freeze.segment(24,14) = VectorXd::Zero(14);

		points[CN_CM].R.slack_cost = cost1;
		points[CN_TO].R.slack_cost = cost2;
		points[CN_CM].T.slack_cost[2] = cost3;
	}
}

void Walking::joint_tasks(double time, double dt, Contact_Manager &points, Joints &joints)
{
	if(state!=WALK)
		return;

	// extra leg lift by hip roll
	double desired_roll = sin(M_PI*time/Tstep) * 0.2;
	joints.ref_pos[7] = desired_roll;
	joints.ref_pos[13] += -desired_roll;
	joints.ref_pos[19] += desired_roll;

	// current base orientation/2
	MatrixXd cd = quat2dc(points[CN_CM].p.pos.segment(3,4));
	double roll = atan2(cd(2,1), cd(2,2));
	double pitch  = -asin(cd(2,0));

	// desired base orientations
	double desired_pitch = quat2ang(getQ(joints.ref_pos))[1];
	desired_roll += quat2ang(getQ(joints.ref_pos))[0];

	// this stabilizes backward walking, given the limited ankle ranges
	desired_pitch += -min(ref_vx, 0.0) * 0.2;

	// calculate fblx, fbrx, fbly, fbry
	calculate_footstep_adjustments(time, points, joints);

	// definitions
	double ll = points[CN_CM].ref_p.pos[2]; // leg length
	int index_l = 6+6;
	int index_r = 6+12;

	// enable stepping after a while
	bool enable = (time-start_time) > (2*Tstep);

	// hip pitch joints
	double Pitch = pitch - desired_pitch;
	double Pitch_feedback = Pitch * hip_gain_K * 2.0;
	joints.ref_pos[index_l] = 	ar * (joints.ref_pos[index_l] + Pitch + fblx/ll) + al * (-Pitch_feedback + joints.sens_pos[index_l]);
	joints.ref_pos[index_r] = 	al * (joints.ref_pos[index_r] + Pitch + fbrx/ll) + ar * (-Pitch_feedback + joints.sens_pos[index_r]);

	// hip roll joints
	double Roll = roll - desired_roll;
	double Roll_feedback = Roll * hip_gain_K;
	joints.ref_pos[index_l+1] =	ar * (joints.ref_pos[index_l+1] - Roll + fbly/ll) + al * (Roll_feedback + joints.sens_pos[index_l+1]);
	joints.ref_pos[index_r+1] = al * (joints.ref_pos[index_r+1] + Roll - fbry/ll) + ar * (-Roll_feedback + joints.sens_pos[index_r+1]);

	// apply hip limits to avoid dramatic failure!
	joints.ref_pos[index_l] = truncate_command(joints.ref_pos[index_l], 0.6, -0.5);
	joints.ref_pos[index_r] = truncate_command(joints.ref_pos[index_r], 0.6, -0.5);
	joints.ref_pos[index_l+1] = truncate_command(joints.ref_pos[index_l+1], 0.5, -0.3);
	joints.ref_pos[index_r+1] = truncate_command(joints.ref_pos[index_r+1], 0.5, -0.3);
}

bool Walking::early_phase(double time, double dt)
{
	return fmod(time,Tstep) <= dt*2.0;
}