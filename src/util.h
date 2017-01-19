/*
 * util.h
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "dataDeclaration.h"

using namespace std;

double l2Norm(vector<double> A);

double l2Norm(point_t A);

static inline int min (int x,int y) { return (x<y)?x:y; }

static inline int max (int x,int y) { return (x>y)?x:y; }

static inline double min (double x,double y) { return (x<y)?x:y; }

static inline double max (double x,double y) { return (x>y)?x:y; }

static inline bool min_ (double x,double y) { return (x<y)?true:false; }

static inline bool max_ (double x,double y) { return (x>y)?true:false; }

static inline point_t min (point_t x, point_t y) { return (l2Norm(x)<l2Norm(y))?x:y; }

static inline vector<double> point2vector(point_t A)
{
	vector<double> B(3);
	B[0]=A.x;
	B[1]=A.y;
	B[2]=A.z;
	return B;
}

static inline point_t vector2point(vector<double> A)
{
	point_t B;
	B.x=A[0];
	B.y=A[1];
	B.z=A[2];
	B.z=UNCLASSIFIED;
	return B;
}

double pdfExp(double var, double mu, double x);

double normalPdf(double var, double mu, double x);

vector<double> addVector(vector<double> A, vector<double> B);

vector<double> minusVector(vector<double> A, vector<double> B);

point_t minusPoint(point_t A, point_t B);

point_t addPoint(point_t A, point_t B);

vector<double> crossProduct(vector<double> A, vector<double> B);

double dotProduct(vector<double> A, vector<double> B);

double movingAverage(double a, vector<double> &A, unsigned int win=5);

void averagePoint(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg,
	unsigned int window);

void averagePointIncrement(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg,
	unsigned int window);

void writePointFile(
	point_t *p,
	unsigned int num_points);

bool checkMoveSlide(
	point_t pos_,
	point_t vel_,
	double *surface_,
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
	double *surface_,
	double surface_limit_,
	double surface_range_limit_);

double surfaceDistance(
	point_t pos_,
	double *surface_);

double surfaceAngle(
	point_t vel_,
	double *surface_);

double surfaceRange(
	point_t pos_,
	point_t pos_surface_,
	double *surface_);

void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_);

// data file ==================================================================
void readFile(
	char *name,
	vector<vector<string> > &data_full,
	char delimiter);

void rewriteFileLoc(
	char *name_,
	point_t *location_,
	unsigned int num_locations,
	int line_,
	vector<string> new_,
	char delimiter_);

void rewriteFileObj(
	char *name_,
	int line_,
	vector<string> new_,
	char delimiter_);

void writeLocLabelFile(
	vector<string> label_,
	unsigned int obj_,
	point_t *location_,
	unsigned int num_locations_);

void writeObjLabelFile(
	vector<string> label_,
	unsigned int obj_);

void parseData2Point(
	vector<vector<string> > data_full,
	point_t *p,
	unsigned int num_points,
	bool SVM=false);

void preprocessData(
	point_t *p1,
	point_t **pos_vel_acc,
	unsigned int num_points,
	unsigned int *file_eof,
	unsigned int window);


// dbscan =====================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_t *p);

point_t* combineNearCluster(
	point_t *p,
	unsigned int num_points,
	unsigned int &num_locations);

void contactBoundary(
	point_t *p,
	point_t *location,
	double *location_boundary,
	unsigned int num_points,
	unsigned int num_locations,
	bool learn);

void decideBoundary(
	point_t &p,
	point_t *location,
	double *location_boundary,
	unsigned int num_locations);

// SVM ========================================================================

void classifierFeature(
	data_t motionData,
	point_t location,
	double &location_distance,
	double &location_angle);

void classifierSVM(
	vector<data_t> motionData,
	int *label,
	unsigned int num_points,
	point_t *location,
	unsigned int num_locations,
	bool train=true);

// Sector =====================================================================

void prepareSector(
	point_t *tmp_dir,
	point_t *tmp_dir_normal,
	double *tmp_norm,
	point_t *location,
	unsigned int num_locations);

void updateSector(
	sector_t ***sector,
	point_t pos_,
	point_t *location,
	unsigned int num_locations,
	point_t *tmp_dir,
	point_t *tmp_dir_normal,
	double *tmp_norm,
	unsigned int num_location_intervals,
	unsigned int num_sector_intervals,
	unsigned int tmp_id,
	unsigned int tmp_id2,
	vector<vector<double> >kernel_);

void checkSector(
	sector_t ***sector,
	double *prediction,
	double *t_val,
	point_t pos_,
	point_t *location,
	unsigned int num_locations,
	point_t *tmp_dir,
	point_t *tmp_dir_normal,
	double *tmp_norm,
	unsigned int num_location_intervals,
	unsigned int num_sector_intervals,
	unsigned int tmp_id,
	bool learn=false);

void generateSector(
	sector_t ***sector,
	point_t *tmp_dir,
	point_t *tmp_dir_normal,
	double *tmp_norm,
	point_t *pos_,
	unsigned int num_points,
	point_t *location,
	unsigned int num_locations,
	unsigned int num_location_intervals,
	unsigned int num_sector_intervals,
	unsigned int *file_eof,
	vector<vector<double> >kernel_);

void checkSectorConstraint(
	sector_t ***sector,
	double ***sector_constraint,
	unsigned int num_locations,
	unsigned int num_location_intervals,
	unsigned int num_sector_intervals);

// VTK ========================================================================
void colorCode(vector<unsigned char*> &container);

void showData(
	point_t *p,
	unsigned int num_points,
	vector<string> &label,
	vector<unsigned char*> color_,
	bool cluster = false,
	bool labeling = false);

void showConnection(
	sector_t ***sector,
	double ***sector_constraint,
	point_t *loc_loc_vec,
	point_t *loc_loc_normal,
	double *loc_loc_norm,
	point_t *location,
	unsigned int num_locations,
	unsigned int num_location_intervals,
	unsigned int num_sector_intervals,
	vector<unsigned char*> color_);


#endif /* UTIL_H_ */
