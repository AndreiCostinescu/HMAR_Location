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

		int LearnBoundary(
			vector<point_d> &centroids_);

		int ContactBoundary(
			vector<point_d> &centroids_);

		int SurfaceContactCheck(
			vector<point_d> centroids_);

		int ClusteringExt(
				vector<point_d> &centroids_);

		int BuildLocationArea(
			Graph 						*Graph_,
			vector<vector<point_d> > 	&pos_vel_acc_,
			vector<int> 				contact_,
			bool						flag_);

	private:

		vector<int> contact, locations_flag, loc_idx_zero; // loc_idx_zero unused at all

		vector<point_d> points_avg, locations; // centroids
		vector<string>  goal_action;

		vector<point_d> 		surfaces_mid;
		vector<point_d> 		surfaces_min;
		vector<point_d> 		surfaces_max;
		vector<point_d> 		surfaces_eq; // equation of plane
		vector<double> 			surfaces_limit; // surface distance limit
		vector<int>				surfaces_flag;  // flag if surface is detected
		vector<vector<double> >	surfaces_rot;  // surface rotation

		vector<vector<unsigned char> > 	color_code;
};

#endif /* TRAINLA_H_ */
