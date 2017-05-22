/*
 * algo.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef ALGO_H_
#define ALGO_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <gsl/gsl_integration.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_linalg.h>

using namespace std;
using namespace Eigen;

#define Sqr(x) ((x)*(x))

//=============================================================================
// inline + template
//=============================================================================

static inline Vector4d V3d4d(Vector3d A)
{
	Vector4d B;
	B(0)=A(0);
	B(1)=A(1);
	B(2)=A(2);
	B(3)=0.0;
	return B;
}

static inline Vector3d V4d3d(Vector4d A)
{
	Vector3d B;
	B(0)=A(0);
	B(1)=A(1);
	B(2)=A(2);
	return B;
}

template<typename T> static inline bool min_ (T x,T y) { return (x<y)?true:false; }
template<typename T> static inline bool max_ (T x,T y) { return (x>y)?true:false; }

template<typename T>
void reshapeVector(vector<T> &A, int size)
{
	A.clear();
	A.resize(size);
}

template<typename T>
void vectorToArray(vector<T> A, T *B)
{
	for(int i=0;i<A.size();i++) { B[i] = A[i]; }
}

template<typename T>
void arrayTovector(T *A, int size, vector<T> &B)
{
	reshapeVector(B,size);
	for(int i=0;i<size;i++) { B[i] = A[i]; }
}

template<typename T>
vector<T> addvector(vector<T> A, vector<T> B)
{
	vector<T> C;
	for(int i=0;i<A.size();i++) { C.push_back(A[i]+B[i]); }
	return C;
}

template<typename T>
vector<T> minusvector(vector<T> A, vector<T> B)
{
	vector<T> C;
	for(int i=0;i<A.size();i++) { C.push_back(A[i]-B[i]); }
	return C;
}

//=============================================================================
// functions
//=============================================================================

double l2Norm(
	vector<double> A);

double pdfExp(
	double var,
	double mu,
	double x);

double normalPdf(
	double var,
	double mu,
	double x);

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

Vector4d movingAverage(
	Vector4d a,
	vector<Vector4d> &A);

Vector4d averagePoint(
	vector<Vector4d> A);

Vector4d averagePointIncrement(
	Vector4d A,
	vector< Vector4d > &A_mem);

void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_);

Vector3d rodriguezVec(
	AngleAxisd aa_,
	Vector3d vec_);

Matrix3d rodriguezRot(
	Vector3d vec_1,
	Vector3d vec_2);

vector<double> transposeInv(
	vector<double> A);

void cal_tangent_normal(
	double t_mid_,
	Vector3d &p_tan_,
	Vector3d &p_nor_,
	vector<Vector3d> coeff,
	int dim,
	bool normal);

double determinant(
	vector<vector<double> > x);

// ============================================================================
// B-spline
// ============================================================================

double curveIntegral (
		double x,
		void *params);

void polyCurveFit(
		vector<double> points_,
		vector<double> &coeff_,
		vector<double> &cov_,
		int DEGREE_);

void polyCurveFitPoint(
		vector<Vector4d> points_,
		vector<Vector4d> &points_est_,
		vector<Vector3d> &coeffs_,
		vector<Vector3d> &covs_,
		int DEGREE_,
		bool flag_est_);

void polyCurveFitEst(
		vector<double> &points_,
		int num_points_,
		vector<double> coeffs_,
		vector<double> covs_,
		int DEGREE_);

void polyCurveLength(
		double &length_,
		double a_,
		double b_,
		vector<Vector3d> coeffs_,
		int DEGREE_);

#endif /* ALGO_H_ */
