/*
 * TrainLA.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef TRAINLA_H_
#define TRAINLA_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "misc.h"
#include "core.h"
#include "DBSCAN.h"

class TrainLA : public DBSCAN
{
	public:

		TrainLA();
		virtual ~TrainLA();

		void ClearLA();

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
			Graph 						*Graph_,
			kb_t 						kb_,
			vector<vector<Vector4d> > 	&pos_vel_acc_,
			vector<int> 				contact_flag_,
			bool						flag_);

	private:

		vector<int> contact_flag, locations_flag, loc_idx_zero; // loc_idx_zero unused at all

		vector<Vector4d> points_avg, locations; // centroids
		vector<string>   goal_action;

		vector<Vector3d> surfaces_mid;
		vector<Vector3d> surfaces_min;
		vector<Vector3d> surfaces_max;
		vector<Vector4d> surfaces_eq; // equation of plane
		vector<double> 	 surfaces_limit; // surface distance limit
		vector<int>		 surfaces_flag;  // flag if surface is detected
		vector<Matrix3d> surfaces_rot;  // surface rotation

		vector<vector<unsigned char> > 	color_code;
};

#endif /* TRAINLA_H_ */
