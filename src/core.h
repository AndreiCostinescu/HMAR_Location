/*
 * core.h
 *
 *  Created on: Mar 26, 2017
 *      Author: chen
 */

#ifndef CORE_H_
#define CORE_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"

bool vectorDirectionCheck(
	vector<double> A,
	vector<double> B);

int decideBoundary(
	point_d 		&point1_,
	point_d 		&point2_,
	vector<point_d> locations_);

void decideLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int loc_offset_);

double decideLocationInterval(
	int &loc_idx_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int offset_);

int decideSectorInterval(
	int &sec_idx_,
	int loc_idx_,
	point_d &delta_t_,
	point_d point_,
	vector<point_d> mid_,
	vector<point_d> tangent_,
	vector<point_d> normal_);

int decideCurvature(
	point_d point_,
	vector<point_d> &curve_mem_,
	double &curve_,
	int num_points_);

#endif /* CORE_H_ */
