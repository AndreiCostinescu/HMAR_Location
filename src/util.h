/*
 * util.h
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "dbscan.h"
#include "Graph.h"

void writePointFile(
	point_t *p,
	unsigned int num_points);

bool checkMoveSlide(
	point_t pos_,
	point_t vel_,
	vector<double> surface_,
	double surface_limit_,
	double angle_limit_);

double checkMoveSlideOutside(
	point_t pos_,
	point_t vel_,
	double **surface_,
	unsigned int num_surfaces_);

bool checkSurfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_,
	double surface_limit_,
	double surface_range_limit_);

double surfaceDistance(
	point_t pos_,
	vector<double> surface_);

double surfaceAngle(
	point_t vel_,
	vector<double> surface_);

double surfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_);


// ============================================================================
// Labels
// ============================================================================

void labelMovement(
	string scene_,
	string object_,
	vector<string> &LABEL_MOV,
	int num_mov);

void labelLocation(
	string scene_,
	string object_,
	vector<point_t> &points,
	vector<point_t> &locations,
	vector<double> &location_boundary,
	vector<string> &LABEL_LOC,
	double epsilon,
	int minpts);

void labelSector(
	string scene_,
	string object_,
	Graph &Graph_,
	vector<vector<vector<sector_t> > > &sector_,
	vector<vector<vector<double> > > sector_constraint_,
	vector<point_t> locations_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	int num_location_intervals_,
	int num_sector_intervals_,
	vector<int> file_eof_,
	vector<unsigned char*> color_code_);

void readingLocation(
	vector<point_t> &locations,
	vector<double> &location_boundary,
	vector<string> &LABEL_LOC,
	int obj);

// ============================================================================
// Files
// ============================================================================

void writeSurfaceFile(
	string scene_,
	vector<vector<double> > surface_);

void writeMovLabelFile(
	string path_,
	vector<string> label_);

void writeLocLabelFile(
	string path_,
	vector<string> label_,
	vector<point_t> locations_,
	vector<double> boundary_);

void writeSectorFile(
	string path_,
	vector<vector<vector<sector_t> > > sector_,
	int maxmin_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_);

void writeSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > sector_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_);

void readFile(
	const char *name,
	vector<vector<string> > &data_full,
	char delimiter);

void readSurfaceFile(
	string scene_,
	vector<vector<double> > &surface_);

void readLocation(
	string path_,
	vector<string> &LABEL_LOC_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_);

void readMovement(
	string path_,
	vector<string> &LABEL_MOV_,
	int num_mov_);

void readSectorFile(
	string path_,
	vector<vector<vector<sector_t> > > &sector_,
	int maxmin_);

void readSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > &sector_);

// ============================================================================
// Data
// ============================================================================

void parseData2Point(
	vector<vector<string> > data_full,
	vector<point_t> &points);

void preprocessDataLive(
	point_t pos,
	vector< vector< vector<double> > > &pos_vel_acc_mem, // motion,xyz,length
	vector<point_t> &pos_vel_acc_avg,
	unsigned int window);

// ============================================================================
// dbscan
// ============================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_t *p);

void combineNearCluster(
	vector<point_t> &points,
	vector<point_t> &locations);

void decideBoundary(
	point_t &p,
	vector<point_t> location,
	vector<double> location_boundary);

void contactBoundary(
	vector<point_t> &p,
	vector<point_t> locations,
	vector<double> &location_boundary,
	bool learn);

// ============================================================================
// Sector
// ============================================================================

void generateSector(
	Graph &Graph_,
	vector<vector<vector<sector_t> > > 	&sector_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	vector<point_t> locations_,
	vector<point_t> tmp_dir_,
	vector<point_t> tmp_dir_normal_,
	vector<double> tmp_norm_,
	int num_location_intervals_,
	int num_sector_intervals_,
	vector<int>file_eof_,
	vector<vector<double> > kernel_);

void prepareSector(
	vector<point_t> &tmp_dir,
	vector<point_t> &tmp_dir_normal,
	vector<double> &tmp_norm,
	vector<point_t> location);

void updateSector(
	vector<vector<vector<sector_t> > > 	&sector_,
	point_t pos_,
	vector<point_t> locations_,
	vector<point_t> tmp_dir_,
	vector<point_t> tmp_dir_normal_,
	vector<double> tmp_norm_,
	int num_location_intervals_,
	int num_sector_intervals_,
	int tmp_id1_,
	int tmp_id2_,
	vector<vector<double> > kernel_);

void checkSector(
	vector<double> &prediction,
	vector<double> &t_val,
	vector<vector<vector<sector_t> > > 	&sector,
	point_t pos_,
	vector<point_t> location,
	vector<point_t> tmp_dir,
	vector<point_t> tmp_dir_normal,
	vector<double> tmp_norm,
	int num_location_intervals,
	int num_sector_intervals,
	int tmp_id,
	bool learn=false);

void checkSectorConstraint(
	vector<vector<vector<sector_t> > > 	sector,
	vector<vector<vector<double> > > &sector_constraint,
	int num_locations,
	int num_location_intervals,
	int num_sector_intervals);

































#endif /* UTIL_H_ */
