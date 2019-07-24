#include "IK.h"
using namespace std;
using namespace Eigen;

#ifdef __cplusplus
	extern "C" {
	#include "IKCG_solver.h"
	}
#endif

IKCG_Vars IKCG_vars;
IKCG_Params IKCG_params;
IKCG_Workspace IKCG_work;
IKCG_Settings IKCG_settings;

IK::IK()
{
	damp = 0.01;
	num_iter = 15;
	Xr = VectorXd::Zero(N_TASK);
	X0 = VectorXd::Zero(N_TASK);
	slack = VectorXd::Zero(N_TASK);
	J = MatrixXd::Zero(N_TASK, AIR_N_U);
	damping = damp * VectorXd::Ones(AIR_N_U);
}

void IK::update_model(Contact_Manager &points, VectorXd q0)
{
	VectorXd dq0 = VectorXd::Zero(AIR_N_U);
	points.model.set_state(q0, dq0);
	int beg_index = 0;

	for(int index=0; index<points.size(); index++)
	{
		int body = points.EE[index].body;
		Vector3d offset = points.EE[index].offset;
		MatrixXd JJ = points.model.get_jacob(body, offset, CT_FULL);

		if(points.EE[index].contact_type==CT_FULL || points.EE[index].contact_type==CT_TRANSLATION)
		{
			X0.segment(beg_index,3)			= points.model.get_pos(body, offset);
			Xr.segment(beg_index,3)			= points.EE[index].ref_p.pos.segment(0,3);
			slack.segment(beg_index,3)		= points.EE[index].T.slack_cost;
			J.block(beg_index,0,3,AIR_N_U) 	= JJ.block(0,0,3,AIR_N_U);
			if(index==0 && points.ifCoM)
			{
				X0.segment(beg_index,3)		= points.model.get_cm();
				J.block(beg_index,0,3,AIR_N_U) = points.model.get_cm_J();
			}
			beg_index += 3;
		}
		if(points.EE[index].contact_type==CT_FULL || points.EE[index].contact_type==CT_ROTATION)
		{
			VectorXd qact 					= quat_sub(points.model.get_orient_quat(body), points.EE[index].ref_p.pos.segment(3,4));
			VectorXd qref 					= ang2quat(Vector3d(0,0,0));
			X0.segment(beg_index,3)			= qact.segment(0,3);
			Xr.segment(beg_index,3)			= qref.segment(0,3);
			slack.segment(beg_index,3)		= points.EE[index].R.slack_cost;
			J.block(beg_index,0,3,AIR_N_U) 	= JJ.block(3,0,3,AIR_N_U);
			beg_index += 3;
		}
	}
}

VectorXd IK::solve_QP(VectorXd &qref, VectorXd &qlow, VectorXd &qup)
{
	VectorXd dX   = Xr - X0;

	IKCG_set_defaults();
	IKCG_setup_indexing();

	for(int i=0;i<AIR_N_U;i++)
		memcpy(IKCG_params.J[i+1], &J(0,i),     sizeof(double)*(N_TASK));
	memcpy(IKCG_params.qlow,       &qlow[0],    sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.qup,        &qup[0],     sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.dx,         &dX[0],      sizeof(double)*(N_TASK));
	memcpy(IKCG_params.damping,    &damping[0], sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.qref,       &qref[0],    sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.slack,      &slack[0],   sizeof(double)*(N_TASK));

	VectorXd hard = slack;
	for(int i=0; i<hard.size(); i++)
		hard[i] = hard[i]==-1 ? 1 : 0;
	memcpy(IKCG_params.hard, &hard[0], sizeof(double)*(N_TASK));

	IKCG_settings.verbose = false;
	int iter = IKCG_solve();
	VectorXd dq = VectorXd::Zero(AIR_N_U);
	memcpy(&dq[0], IKCG_vars.dq, sizeof(double)*(AIR_N_U));
	return dq;
}

double IK::return_hand_error()
{
	// call exactly after solving IK
	return vectorbig((Xr-X0).segment(18,3), (Xr-X0).segment(24,3)).norm();
}

double IK::solve(Contact_Manager &points, VectorXd& ref_pos, VectorXd freeze)
{
	timeval start, end;
	gettimeofday(&start, NULL);

	damping = damp * VectorXd::Ones(AIR_N_U);
	for(int j=0;j<num_iter;j++)
	{
		// recalculate gradients
		update_model(points, ref_pos);

		for(int i=0;i<AIR_N_U;i++)
			if(freeze[i]==1)
				J.block(0,i,N_TASK,1) *= 0;

		VectorXd qlow = points.model.qmin/180.0*M_PI - ref_pos.segment(6,AIR_N_U-6);
		VectorXd qup  = points.model.qmax/180.0*M_PI - ref_pos.segment(6,AIR_N_U-6);

		VectorXd qref = VectorXd::Zero(AIR_N_U);
		qref.segment(6,AIR_N_U-6) = - (qup+qlow) * 0.5 * 0;
		qref = qref.cwiseProduct(VectorXd::Ones(AIR_N_U)-freeze);

		// solve linearized CVXGEN problem
		VectorXd dq = solve_QP(qref, qlow, qup);
		dq = dq.cwiseProduct(VectorXd::Ones(AIR_N_U)-freeze);

		// apply dq
		VectorXd q0 = getQ(ref_pos);
		ref_pos.segment(0,AIR_N_U) += dq;
		Vector4d w(0,0,0,0);
		w.segment(0,3) = quat2dc(q0) * dq.segment(3,3);
		VectorXd rotBase = quat_mul(quat_exp(0.5 * w), q0);
		ref_pos = setQ(ref_pos,rotBase);
	}


	gettimeofday(&end, NULL);
	return (end.tv_sec-start.tv_sec) + (end.tv_usec-start.tv_usec)/1e6;
}