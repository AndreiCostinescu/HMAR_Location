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
	vector<point_d> &p,
	vector<point_d> &locations,
	bool 			learn);

int contactCheck(
	vector<point_d> &points_,
	vector<int> 	contact_,
	int 			num_locations_);

int clusteringExt(
	vector<point_d> &points_,
	vector<int> 	contact_,
	vector<point_d> &locations_,
	vector<string> 	&labels_,
	vector<string> 	labels_ref_,
	bool 			delete_=false);

int buildLocationArea(
	Graph 						*Graph_,
	vector<vector<point_d> > 	&pos_vel_acc_,
	vector<int> 				contact_);

#endif /* LABELING_LOC_H_ */
