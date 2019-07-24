#pragma once

#include "Contacts.h"
#include "mesh_utils.h"

class Collision
{
public:

	fcl::CollisionRequestd c_req;
	fcl::CollisionResultd c_res;
	VectorXi idx_map;
	vector<fcl::CollisionObjectd*> colObjs;
	vector<Eigen::VectorXi> collisions_grid;
	Body meshes_zero;

	Collision();
	~Collision() {};
	bool check(Contact_Manager &points);
};
