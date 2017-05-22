/*
 * TrainLA.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef TRAINLA_H_
#define TRAINLA_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "CGraph.h"
#include "CKB.h"
#include "VTKExtra.h"
#include "print.h"
#include "core.h"
#include "DBSCAN.h"

#define BOUNDARY_VAR 0.1
#define DBSCAN_EPS 0.015
#define DBSCAN_MIN 5

class TrainLA : public DBSCAN
{
public:
	TrainLA();
	virtual ~TrainLA();

	void ClearLA();
	int InitLA(
			int loc_int_,
			int sec_int_,
			int f_win_);
	int DecideBoundaryCuboidExt(
			Vector4d &point_,
			Vector3d box_min_,
			Vector3d box_max_);
	int LearnBoundary(
			vector<Vector4d> &centroids_);
	int ContactBoundary(
			vector<Vector4d> &centroids_);
	int SurfaceContactCheck(
			vector<Vector4d> &centroids_);
	int ClusteringExt(
			vector<Vector4d> &centroids_);
	int BuildLocationArea(
			CGraph *Graph_,
			CKB *kb_,
			vector<vector<Vector4d> > &pos_vel_acc_,
			vector<int> *contact_flag_,
			bool flag_);

private:
	vector<int> contact_flag; // contact 1/0
	vector<int> locations_flag; // whether location has change in contact 1/0
	vector<int> loc_idx_zero; // loc_idx_zero unused at all

	vector<Vector4d> points_avg, locations; // centroids
	vector<string>   goal_action;

	vector<Vector3d> surfaces_mid;
	vector<Vector3d> surfaces_min;
	vector<Vector3d> surfaces_max;
	vector<Vector4d> surfaces_eq; // equation of plane
	vector<double> 	 surfaces_limit; // surface distance limit
	vector<int>		 surfaces_flag;  // flag if surface is detected
	vector<Matrix3d> surfaces_rot;  // surface rotation

	VTKExtra *VTK;

protected:
	int loc_int;
	int sec_int;
	int f_win;

};

#endif /* TRAINLA_H_ */
