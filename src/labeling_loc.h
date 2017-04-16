/*
 * labelling.h
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#ifndef LABELING_LOC_H_
#define LABELING_LOC_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "dbscan.h"
#include "misc.h"
#include "core.h"

int decideBoundaryExt(
	point_d 		&point1_,
	point_d 		&point2_,
	vector<point_d> locations_);

int contactBoundary(
	vector<point_d> 		&points_,
	vector<point_d> 		&centroids_,
	vector<vector<double> > surfaces_eq_,
	vector<int> 			&surface_flag_,
	vector<double> 			&surfaces_limit_,
	bool 					learn_);

int contactCheck(
	vector<point_d> &points_,
	vector<point_d> &centroids_,
	vector<int> centroids_flag_,
	vector<point_d> surfaces_,
	vector<vector<double> > surfaces_eq_,
	vector<int> &surface_flag_,
	vector<double> &surfaces_limit_);

int clusteringExt(
	vector<point_d> 		&points_,
	vector<point_d> 		&centroids_,
	vector<int> 			&centroids_flag_,
	vector<int> 			contact_,
	vector<point_d> 		surfaces_,
	vector<vector<double> > surfaces_eq_,
	vector<int> 			&surfaces_flag_,
	vector<double> 			&surfaces_limit_);

int buildLocationArea(
	Graph 						*Graph_,
	vector<vector<point_d> > 	&pos_vel_acc_,
	vector<int> 				contact_,
	bool						flag_);

#endif /* LABELING_LOC_H_ */
