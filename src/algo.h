/*
 * algo.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef ALGO_H_
#define ALGO_H_

#include "dataDeclaration.h"
#include "vtkExtra.h"

//=============================================================================
// inline + template
//=============================================================================

static inline vector<double> point2vector(point_d A)
{
	vector<double> B; B.resize(3);
	B[0]=A.x;
	B[1]=A.y;
	B[2]=A.z;
	return B;
}

static inline point_d vector2point(vector<double> A)
{
	point_d B;
	B.x=A[0];
	B.y=A[1];
	B.z=A[2];
	B.l=0.0;
	return B;
}

template<typename T>
static inline bool min_ (T x,T y) { return (x<y)?true:false; }

template<typename T>
static inline bool max_ (T x,T y) { return (x>y)?true:false; }

template<typename T>
void reshapeVector(vector<T> &A, int size)
{
	A.clear();
	A.resize(size);
}

template<typename T>
void vector2array(vector<T> A, T *B)
{
	for(int i=0;i<A.size();i++) B[i] = A[i];
}

template<typename T>
void array2vector(T *A, int size, vector<T> &B)
{
	reshapeVector(B,size);
	for(int i=0;i<size;i++) B[i] = A[i];
}

template<typename T>
vector<T> addVector(vector<T> A, vector<T> B)
{
	vector<T> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]+B[i]);
	return C;
}

template<typename T>
vector<T> minusVector(vector<T> A, vector<T> B)
{
	vector<T> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]-B[i]);
	return C;
}

// DATA IS ASSUMED TO BE POSITIVE ONLY
template<typename T>
void normalizeData(vector<T> &data_)
{
	T tmp = 0;
	for(int i=0;i<data_.size();i++)
		tmp += data_[i];
	if (tmp>0)
		for(int i=0;i<data_.size();i++)
			data_[i]/=tmp;
//	else
//		printf("[WARNING] : Data is empty.\n");
}

//=============================================================================
// functions
//=============================================================================

double l2Norm(
	vector<double> A);

double l2Norm(
	point_d A);

double pdfExp(
	double var,
	double mu,
	double x);

double normalPdf(
	double var,
	double mu,
	double x);

point_d addPoint(
	point_d A,
	point_d B);

point_d minusPoint(
	point_d A,
	point_d B);

point_d multiPoint(
	point_d A,
	double B);

vector<double> crossProduct(
	vector<double> A,
	vector<double> B);

double dotProduct(
	vector<double> A,
	vector<double> B);


double addFunction (
	double x,
	double y);

double average(
	vector<double> A);

int movingAverage(
	int a,
	vector<int> &A);

int movingAverageIncrement(
	int a,
	vector<int> &A);

double movingAverage(
	double a,
	vector<double> &A);

double movingAverageIncrement(
	double a,
	vector<double> &A);

point_d movingAverage(
	point_d a,
	vector<point_d> &A);

point_d averagePoint(
	vector<point_d> A);

point_d averagePointIncrement(
	point_d A,
	vector< point_d > &A_mem);

void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_);

point_d rodriguezVec(
	double angle_,
	point_d axis_,
	point_d vec_);

vector<double> rodriguezRot(
	point_d vec_1,
	point_d vec_2);

vector<double> transInv(
	vector<double> A);

void cal_tangent_normal(
	double t_mid_,
	point_d &p_tan_,
	point_d &p_nor_,
	vector<point_d> coeff,
	int dim,
	bool normal);

void reshapePredict(
	predict_t &P,
	int size);

double determinant(
	vector<vector<double> > x);

// ============================================================================
// B-spline
// ============================================================================

double curveIntegral (
	double x,
	void * params);

//void curveFit(
//	vector<point_d> points_,
//	vector<point_d> &curves_);

void polyCurveFit(
	vector<double> points_,
	vector<double> &coeff_,
	vector<double> &cov_);

void polyCurveFitPoint(
	vector<point_d> points_,
	vector<point_d> &points_est_,
	vector<point_d> &coeffs_,
	vector<point_d> &covs_,
	bool flag_est_);

void polyCurveFitEst(
	vector<double> &points_,
	int num_points_,
	vector<double> coeffs_,
	vector<double> covs_);

void polyCurveLength(
	double &length_,
	double a_,
	double b_,
	vector<point_d> coeffs_);

// ============================================================================
// Surface
// ============================================================================



double surfaceDistance(
	point_d pos_,
	vector<double> surface_);

double surfaceAngle(
	point_d vel_,
	vector<double> surface_);

double surfaceRange(
	point_d pos_,
	point_d pos_surface_,
	vector<double> surface_);

#endif /* ALGO_H_ */
