/*
 * core.h
 *
 *  Created on: Mar 26, 2017
 *      Author: chen
 */

#ifndef CORE_H_
#define CORE_H_

#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>
#include "algo.h"

#define BOUNDARY_VAR 0.1

bool directionCheck(
	Vector3d A,
	Vector3d B);

int checkBoundarySphere(
	Vector4d &point_,
	vector<Vector4d> centroids_);

int checkBoundaryCuboid(
	Vector4d &point_,
	Vector3d box_min_,
	Vector3d box_max_);

int decideBoundarySphere(
	Vector4d 		 &point_,
	vector<Vector4d> centroids_);

int decideBoundaryCuboid(
	Vector4d &point_,
	Vector3d box_min_,
	Vector3d box_max_);

int decideBoundaryClosest(
	Vector4d 		&point_,
	vector<Vector4d> centroids_);

double checkSurfaceDistance(
	Vector4d centroids_,
	Vector4d surface_eq_);

bool decideSurface(
	Vector4d centroids_,
	Vector4d surface_eq_,
	double limit_);

double dLI(
	int &loc_idx_,
	int loc_last_idx_,
	Vector4d point_,
	vector<double> len_,
	vector<Vector4d> mid_,
	vector<Vector3d> tangent_,
	int loc_offset_,
	bool loc_init_,
	int LOC_INT_);

double dLIPredict(
	int &loc_idx_,
	int loc_last_idx_,
	Vector4d point_,
	vector<Vector4d> mid_,
	vector<double> len_,
	vector<Vector3d> tangent_,
	int loc_offset_,
	int loc_init_, //0 for true
	int LOC_INT_);

int decideSectorInterval(
	int &sec_idx_,
	int loc_idx_,
	Vector3d &delta_t_,
	Vector4d point_,
	vector<Vector4d> mid_,
	vector<Vector3d> tangent_,
	vector<Vector3d> normal_,
	int SEC_INT_);

//int decideCurvature(
//	point_d point_,
//	vector<point_d> &curve_mem_,
//	double &curve_,
//	int num_points_);

int folderSelect1(
	const struct dirent *entry);

int folderSelect2(
	const struct dirent *entry);

int fileSelect(
	const struct dirent *entry);

bool copyFile(
		string SRC,
		string DEST);

bool directoryCheck(
		string path_);

#endif /* CORE_H_ */
