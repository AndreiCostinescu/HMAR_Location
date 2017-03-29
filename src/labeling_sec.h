/*
 * labeling_sec.h
 *
 *  Created on: Mar 17, 2017
 *      Author: chen
 */

#ifndef LABELING_SEC_H_
#define LABELING_SEC_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "dbscan.h"
#include "misc.h"
#include "core.h"

int findMovementConstraint(
	Graph &Graph_,
	vector<point_d> &points_,
	vector<point_d> vels_,
	int label1_,
	int label2_);

int fitCurve(
	vector<point_d> points_avg_,
	vector<point_d> &points_est_,
	vector<point_d> &coeffs_);

int decideSectorIntervalExt(
	int &sec_idx_,
	int loc_idx_,
	point_d &delta_t_,
	point_d point_,
	vector<point_d> mid_,
	vector<point_d> tangent_,
	vector<point_d> normal_);

double decideLocationIntervalExt(
	int &loc_idx_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int offset_);

double decideLocationIntervalExt(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int loc_offset_);

int adjustSectorMap(
	vector<double> &sector_map_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	vector<point_d> normal_,
	int loc_offset_,
	int option_);

int adjustCurve(
	Graph &Graph_,
	vector<point_d> coeffs_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_);

int fitSectorMapInit(
	Graph &Graph_,
	vector<point_d> &points_,
	int label1_,
	int label2_,
	int loc_offset_);

int fitSectorMap(
	Graph &Graph_,
	vector<point_d> &points_,
	int label1_,
	int label2_,
	int loc_offset_);

int findSectorMapConstraint(
	Graph &Graph_,
	int label1_,
	int label2_);

int findWindowConstraint(
	Graph &Graph_,
	int label1_,
	int label2_);

int updateSectorMap(
	Graph &Graph_,
	vector<point_d> points_avg_,
	int label1_,
	int label2_);

int buildSectorMap(
	Graph &Graph_,
	vector<vector<point_d> > pva_avg_,
	vector<int> contact_);

#endif /* LABELING_SEC_H_ */
