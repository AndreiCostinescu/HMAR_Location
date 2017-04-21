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

		int DecideBoundaryExt(
			point_d 		&point1_,
			point_d 		&point2_,
			vector<point_d> locations_);

		int ContactBoundary(
			vector<point_d> &centroids_,
			bool learn_);

		int ContactCheck();

		int ClusteringExt(
				vector<point_d> &centroids_);

		int BuildLocationArea(
			Graph 						*Graph_,
			vector<vector<point_d> > 	&pos_vel_acc_,
			vector<int> 				contact_,
			bool						flag_);

	private:

		vector<int> contact, locations_flag, surfaces_flag, loc_idx_zero; // loc_idx_zero unused at all

		vector<point_d> points_avg, locations; // centroids
		vector<string>  goal_action;

		vector<point_d> surfaces;
		vector<vector<double> > surfaces_eq;
		vector<double> surfaces_limit;

		vector<vector<unsigned char> > 	color_code;
};

#endif /* TRAINLA_H_ */
