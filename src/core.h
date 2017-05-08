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

bool vectorDirectionCheck(
	vector<double> A,
	vector<double> B);

int checkBoundarySphere(
	point_d &point1_,
	point_d &point2_,
	vector<point_d> centroids_);

int checkBoundaryCuboid(
	point_d &point_,
	point_d box_min_,
	point_d box_max_);

int decideBoundarySphere(
	point_d &point_,
	vector<point_d> centroids_);

int decideBoundaryCuboid(
	point_d &point_,
	point_d box_min_,
	point_d box_max_);

int decideBoundaryClosest(
	point_d &point2_,
	vector<point_d> centroids_);

double checkSurfaceDistance(
	point_d centroids_,
	point_d	surfaces_);

bool decideSurface(
	point_d centroids_,
	point_d	surfaces_,
	double limit_);

double dLI(
	int &loc_idx_,
	int loc_last_idx_,
	point_d point_,
	vector<double> len_,
	vector<point_d> mid_,
	vector<point_d> tangent_,
	int loc_offset_,
	bool loc_init_);

double dLIPredict(
	int &loc_idx_,
	int loc_last_idx_,
	point_d point_,
	vector<point_d> mid_,
	vector<double> len_,
	vector<point_d> tangent_,
	int loc_offset_,
	int loc_init_);

double decideLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int loc_offset_,
	bool loc_init_=false);

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

int decideRateOfChangeOfDeltaT(
	point_d delta_t_,
	vector<point_d> &delta_t_mem_,
	double &dd_delta_t_,
	int num_points_);

int folderSelect1(
	const struct dirent *entry);

int folderSelect2(
	const struct dirent *entry);

int fileSelect(
	const struct dirent *entry);


#endif /* CORE_H_ */
