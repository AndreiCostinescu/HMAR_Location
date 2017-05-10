/*
 * algo.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "algo.h"

//#define BSPLINE

//=============================================================================
// functions
//=============================================================================

double l2Norm(
	vector<double> A)
{
    double a=0.0;
    for(int i=0;i<A.size();i++) {a+=Sqr(A[i]);}
    return sqrt(a);
}

double l2Norm(
	point_d A)
{
    return sqrt(Sqr(A.x)+Sqr(A.y)+Sqr(A.z));
}

double pdfExp(
	double var,
	double mu,
	double x)
{
	return exp(-Sqr(x-mu)/(2*var));
}

double normalPdf(
	double var,
	double mu,
	double x)
{
	return (1/sqrt(2*var*M_PI)) * exp(-Sqr(x-mu)/(2*var));
}

point_d addPoint(
	point_d A,
	point_d B)
{
	point_d C;
	C.x = A.x + B.x;
	C.y = A.y + B.y;
	C.z = A.z + B.z;
	C.l = A.l;
	return C;
}

point_d minusPoint(
	point_d A,
	point_d B)
{
	point_d C;
	C.x = A.x - B.x;
	C.y = A.y - B.y;
	C.z = A.z - B.z;
	C.l = A.l;
	return C;
}

point_d multiPoint(
	point_d A,
	double  B)
{
	point_d C;
	C.x = A.x*B;
	C.y = A.y*B;
	C.z = A.z*B;
	C.l = A.l;
	return C;
}

point_d normPoint(point_d A)
{
	return multiPoint(A, 1.0/l2Norm(A));
}

vector<double> transposeInv(
		vector<double> A)
{
	double det =	+ A[0]*(A[4]*A[8]-A[7]*A[5])
					- A[1]*(A[3]*A[8]-A[5]*A[6])
					+ A[2]*(A[3]*A[7]-A[4]*A[6]);
	double invdet = 1/det;
	vector<double> B; B.resize(9);
	B[0] =  (A[4]*A[8]-A[7]*A[5])*invdet;
	B[3] = -(A[1]*A[8]-A[2]*A[7])*invdet;
	B[6] =  (A[1]*A[5]-A[2]*A[4])*invdet;
	B[1] = -(A[3]*A[8]-A[5]*A[6])*invdet;
	B[4] =  (A[0]*A[8]-A[2]*A[6])*invdet;
	B[7] = -(A[0]*A[5]-A[3]*A[2])*invdet;
	B[2] =  (A[3]*A[7]-A[6]*A[4])*invdet;
	B[5] = -(A[0]*A[7]-A[6]*A[1])*invdet;
	B[8] =  (A[0]*A[4]-A[3]*A[1])*invdet;
	return B;
}

double addFunction (
	double x,
	double y)
{
	return fabs(x)+fabs(y);
}

int average(
	vector<int> A)
{
	if (accumulate( A.begin(), A.end(), 0.0, addFunction) == 0)
		return 0;
	else
		return round(accumulate(A.begin(), A.end(), 0.0, addFunction)/A.size());
}

double average(
	vector<double> A)
{
	if (accumulate( A.begin(), A.end(), 0.0, addFunction) == 0)
		return 0.0;
	else
		return accumulate( A.begin(), A.end(), 0.0, addFunction)/A.size();
}

int movingAverage(
	int a,
	vector<int> &A)
{
	A.erase(A.begin());
	A.push_back(a);
	return average(A);
}

int movingAverageIncrement(
	int a,
	vector<int> &A)
{
	A.push_back(a);
	return average(A);
}

double movingAverage(
	double a,
	vector<double> &A)
{
	A.erase(A.begin());
	A.push_back(a);
	return average(A);
}

double movingAverageIncrement(
	double a,
	vector<double> &A)
{
	A.push_back(a);
	return average(A);
}

point_d movingAverage(
	point_d a,
	vector<point_d> &A)
{
	A.erase(A.begin());
	A.push_back(a);
	return averagePoint(A);
}

point_d averagePoint(
	vector<point_d> A)
{
	point_d avg = {};
	for (int i=0;i<A.size();i++)
	{
		avg = addPoint(avg, A[i]);
	}
	avg = multiPoint(avg, 1.0/A.size());
	return avg;
}

point_d averagePointIncrement(
	point_d A,
	vector< point_d > &A_mem)
{
	vector< point_d > tmp = A_mem;
	tmp.push_back(A);
	point_d avg = averagePoint(tmp);
	A_mem.push_back(avg);
	return avg;
}

Vector4d movingAverage(
	Vector4d a,
	vector<Vector4d> &A)
{
	A.erase(A.begin());
	A.push_back(a);
	return averagePoint(A);
}

Vector4d averagePoint(
	vector<Vector4d> A)
{
	Vector4d avg = Vector4d::Zero();
	for (int i=0;i<A.size();i++)
	{
		avg = avg + A[i];
	}
	avg = avg / A.size();
	return avg;
}

Vector4d averagePointIncrement(
	Vector4d A,
	vector< Vector4d > &A_mem)
{
	vector< Vector4d > tmp = A_mem;
	tmp.push_back(A);
	Vector4d avg = averagePoint(tmp);
	A_mem.push_back(avg);
	return avg;
}

Vector3d rodriguezVec(
	AngleAxisd aa_,
	Vector3d vec_)
{
	return 	(vec_*cos(aa_.angle())) +
			((aa_.axis()).cross(vec_)*sin(aa_.angle())) +
			(aa_.axis()*((aa_.axis()).dot(vec_))*(1-cos(aa_.angle())));
}

Matrix3d rodriguezRot(
	Vector3d vec_1,
	Vector3d vec_2)
{
	Vector3d axis;
	Matrix3d A;
	double angle;
	axis  = vec_1.cross(vec_2).normalized();
	angle = acos(vec_1.dot(vec_2) / (vec_1.norm()*vec_2.norm()));
	A(0,0) = 0.0;
	A(0,1) = -axis(2);
	A(0,2) =  axis(1);
	A(1,0) =  axis(2);
	A(1,1) = 0.0;
	A(1,2) = -axis(0);
	A(2,0) = -axis(1);
	A(2,1) =  axis(0);
	A(2,2) = 0.0;
	return Matrix3d::Identity(3,3) + (A*sin(angle)) + ((A*A)*(1-cos(angle)));
}

void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_)
{
    double sum = 0.0;
    for (int x = -(numx_/2); x <= (numx_/2); x++)
    {
        for(int y = -(numy_/2); y <= (numy_/2); y++)
        {
        	kernel_[x+(numx_/2)][y+(numy_/2)] =
        			(1/(2*M_PI*var_)) *
        			exp(-(Sqr(sqrt(Sqr(x) + Sqr(y)))/(2*var_)));
            sum += kernel_[x+(numx_/2)][y+(numy_/2)];
        }
    }
    for(int i = 0; i < numx_; ++i)
        for(int j = 0; j < numy_; ++j)
        	kernel_[i][j] /= sum;
}

void cal_tangent_normal(
	double t_mid_,
	Vector3d &p_tan_,
	Vector3d &p_nor_,
	vector<Vector3d> coeff,
	int dim,
	bool normal)
{
	Vector3d out1, out2, out4;
	out4 = out2 = out1 = Vector3d::Zero();

	for(int i=0;i<dim;i++)
	{
		out1 += (i * coeff[i] * pow(t_mid_,i-1));
	}
	p_tan_ = out1;

	if(normal)
	{
		for(int i=0;i<dim;i++)
		{
			out2 += (i * (i-1) * coeff[i] * pow(t_mid_,i-2));
		}
		double out3N = (out1.cwiseProduct(out2)).sum()*2;
		out4 = ((out1.norm()*out2) - (0.5*out1*out3N*(1/out1.norm()))) /
				Sqr(out1.norm());
		p_nor_ = out4;
	}
	else
	{
		p_nor_ = out4;
	}
}

void reshapePredictEdge(
	predict_e &P_,
	int size)
{
	P_.acc		 = 0.0;
	P_.vel		 = 0.0;
	P_.curvature = 0.0;
	reshapeVector(P_.range, 		size);
	reshapeVector(P_.err, 			size);
	reshapeVector(P_.pct_err, 		size);
	reshapeVector(P_.err_diff,		size);
	reshapeVector(P_.pct_err_diff,	size);
	reshapeVector(P_.oscillate, 	size);
	reshapeVector(P_.ddl, 			size);
	reshapeVector(P_.window,	 	size);
}

void reshapePredictNode(
	predict_n &P_,
	int size)
{
	P_.acc		 	= 0.0;
	P_.vel		 	= 0.0;
	P_.surface_dist = 0.0;
}

double determinant(
	vector<vector<double> > x)
{
	int sign = 0;
	int * signum = &sign;
	double det = 0.0;
	gsl_permutation * p = gsl_permutation_calloc(x.size());
	gsl_matrix * A = gsl_matrix_calloc(x.size(), x.size());
	for(int j=0;j<x.size();j++)
	{
		for(int i=0;i<x.size();i++)
		{
			gsl_matrix_set(A, i, j, x[j][i]);
		}
	}
	gsl_linalg_LU_decomp(A, p, signum);
	det = gsl_linalg_LU_det(A, *signum);
	gsl_permutation_free(p);
	gsl_matrix_free(A);
	return det;
}

// ============================================================================
// B-spline
// ============================================================================

// example
void curveFit(
	vector<point_d> points_,
	vector<point_d> &curves_)
{
	const size_t n = points_.size();
	const size_t ncoeffs = NCOEFFS;
	const size_t nbreak = NBREAK;

	size_t i, j;
	gsl_bspline_workspace *bw;
	gsl_vector *B;
	gsl_vector *c, *w;
	gsl_vector *x, *y;
	gsl_vector *yerr;
	gsl_matrix *X, *cov;
	gsl_multifit_linear_workspace *mw;
	double chisq;

	for(int ii=0;ii<3;ii++)
	{
		/* allocate a cubic bspline workspace (k = 4) */
		bw 	= gsl_bspline_alloc(4, nbreak);
		B 	= gsl_vector_alloc(ncoeffs);

		x 		= gsl_vector_alloc(n);
		y 		= gsl_vector_alloc(n);
		X 		= gsl_matrix_alloc(n, ncoeffs);
		c 		= gsl_vector_alloc(ncoeffs);
		w 		= gsl_vector_alloc(n);
		cov 	= gsl_matrix_alloc(ncoeffs, ncoeffs);
		mw 		= gsl_multifit_linear_alloc(n, ncoeffs);
		yerr 	= gsl_vector_alloc(n);

		/* this is the data to be fitted */
		for (i=0;i<n;++i)
		{
			gsl_vector_set(x, i, double(i));
			switch (ii)
			{
				case 0:
					gsl_vector_set(y, i, points_[i].x);
					break;
				case 1:
					gsl_vector_set(y, i, points_[i].y);
					break;
				case 2:
					gsl_vector_set(y, i, points_[i].z);
					break;
			}
		}

		/* use uniform breakpoints on [0, 15] */
		gsl_bspline_knots_uniform(0.0, (double)n, bw);

		/* construct the fit matrix X */
		for (i=0;i<n;++i)
		{
			double xi = gsl_vector_get(x, i);

			/* compute B_j(xi) for all j */
			gsl_bspline_eval(xi, B, bw);

			/* fill in row i of X */
			for (j = 0; j < ncoeffs; ++j)
			{
				double Bj = gsl_vector_get(B, j);
				gsl_matrix_set(X, i, j, Bj);
			}
		}

		/* do the fit */
		gsl_multifit_linear(X, y, c, cov, &chisq, mw);

		/* output the smoothed curve */
		for (i=0;i<n;i++)
		{
			double yerr_tmp;
			double xi = gsl_vector_get(x, i);
			gsl_bspline_eval(xi, B, bw);
			switch (ii)
			{
				case 0:
					gsl_multifit_linear_est(B, c, cov, &curves_[i].x, &yerr_tmp);
					break;
				case 1:
					gsl_multifit_linear_est(B, c, cov, &curves_[i].y, &yerr_tmp);
					break;
				case 2:
					gsl_multifit_linear_est(B, c, cov, &curves_[i].z, &yerr_tmp);
					break;
			}
			gsl_vector_set(yerr, i, yerr_tmp);
		}

		gsl_bspline_free(bw);
		gsl_vector_free(B);
		gsl_vector_free(x);
		gsl_vector_free(y);
		gsl_matrix_free(X);
		gsl_vector_free(c);
		gsl_vector_free(w);
		gsl_matrix_free(cov);
		gsl_multifit_linear_free(mw);
	}
}

void polyCurveFit(
	vector<double> points_,
	vector<double> &coeff_,
	vector<double> &cov_)
{

#ifdef BSPLINE

	int num_points = points_.size();
	double chisq; //residual error

	gsl_bspline_workspace *bw;
	gsl_multifit_linear_workspace *mws;
	gsl_matrix *cov, *X;
	gsl_vector *y, *c, *B;

	B = gsl_vector_alloc(NCOEFFS);
	X = gsl_matrix_alloc(num_points, NCOEFFS);
	y = gsl_vector_alloc(num_points);
	c = gsl_vector_alloc(NCOEFFS);
	cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);
	bw = gsl_bspline_alloc(4, NBREAK);
	mws = gsl_multifit_linear_alloc(num_points, NCOEFFS);

	gsl_bspline_knots_uniform(0.0, num_points, bw);

	for(int i=0;i<num_points;i++)
	{
		gsl_bspline_eval((double)i, B, bw);
		for (int j = 0; j < NCOEFFS; ++j)
		{
			double Bj = gsl_vector_get(B, j);
			gsl_matrix_set(X, i, j, Bj);
		}
		gsl_vector_set(y, i, points_[i]);
	}

	gsl_multifit_linear(X, y, c, cov, &chisq, mws);

	reshapeVector(coeff_, NCOEFFS);
	for(int i=0;i<NCOEFFS;i++) { coeff_[i] = gsl_vector_get(c, i); }
	reshapeVector(cov_, Sqr(NCOEFFS));
	for(int i=0;i<Sqr(NCOEFFS);i++)
	{
		cov_[i] = gsl_matrix_get(cov, i/NCOEFFS, i%NCOEFFS);
	}

	// for visualizing the curve-fitting
	if(0)
	{
		vector<double> y0(num_points);
		{
			double yerr;
			for(int i=0;i<num_points;i++)
			{
				gsl_bspline_eval((double)i, B, bw);
				gsl_multifit_linear_est(B, c, cov, &y0[i], &yerr);
			}
		}
		vector<double> x0;
		for(int i=0;i<num_points;i++)
		{ x0.push_back(i); }
		plotData(x0, points_, x0, y0);
	}

	gsl_multifit_linear_free(mws);
	gsl_bspline_free(bw);
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(c);
	gsl_vector_free(B);

#else

	gsl_multifit_linear_workspace *mws;
	gsl_matrix *cov, *X;
	gsl_vector *y, *c;
	double chisq; //residual error
	int num_points = points_.size();
	X = gsl_matrix_alloc(num_points, DEGREE);
	y = gsl_vector_alloc(num_points);
	c = gsl_vector_alloc(DEGREE);
	cov = gsl_matrix_alloc(DEGREE, DEGREE);
	for(int i=0;i<num_points;i++)
	{
		for(int j=0;j<DEGREE;j++)
		{
			gsl_matrix_set(X, i, j, pow(i, j));
		}
		gsl_vector_set(y, i, points_[i]);
	}
	mws = gsl_multifit_linear_alloc(num_points, DEGREE);
	gsl_multifit_linear(X, y, c, cov, &chisq, mws);
	reshapeVector(coeff_, DEGREE);
	for(int i=0;i<DEGREE;i++) { coeff_[i] = gsl_vector_get(c, i); }
	reshapeVector(cov_, Sqr(DEGREE));
	for(int i=0;i<Sqr(DEGREE);i++)
	{
		cov_[i] = gsl_matrix_get(cov, i/DEGREE, i%DEGREE);
	}
	gsl_multifit_linear_free(mws);
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(c);

#endif

}

void polyCurveFitEst(
	vector<double> &points_,
	int num_points_,
	vector<double> coeffs_,
	vector<double> covs_)
{

#ifdef BSPLINE

	gsl_matrix *cov, *X;
	gsl_vector *c, *Xj, *B;
	gsl_bspline_workspace *bw;

	B = gsl_vector_alloc(NCOEFFS);
	bw = gsl_bspline_alloc(4, NBREAK);

	double cc[NCOEFFS];
	X  = gsl_matrix_alloc(num_points_, NCOEFFS);
	Xj = gsl_vector_alloc(NCOEFFS);
	c  = gsl_vector_alloc(NCOEFFS);
	cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);

	for(int i=0;i<NCOEFFS;i++) { gsl_vector_set(c, i, coeffs_[i]); }

	for(int i=0;i<Sqr(NCOEFFS);i++)
	{
		gsl_matrix_set(cov, i/NCOEFFS, i%NCOEFFS, covs_[i]);
	}

	gsl_bspline_knots_uniform(0.0, points_.size(), bw);

	double yerr;
	for(int i=0;i<points_.size();i++)
	{
        gsl_bspline_eval((double)i, B, bw);
        gsl_multifit_linear_est(B, c, cov, &points_[i], &yerr);
	}

	// Vizualize
	if(0)
	{
		vector<double> x0;
		for(int i=0;i<points_.size();i++) {x0.push_back(i);}
		plotData(x0, points_);
	}

	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(Xj);
	gsl_vector_free(c);
	gsl_vector_free(B);

#else

	gsl_matrix *cov, *X;
	gsl_vector *c, *Xj;
	double cc[DEGREE];
	X  = gsl_matrix_alloc(num_points_, DEGREE);
	Xj = gsl_vector_alloc(DEGREE);
	c  = gsl_vector_alloc(DEGREE);
	cov = gsl_matrix_alloc(DEGREE, DEGREE);
	for(int i=0;i<DEGREE;i++) { cc[i] = coeffs_[i]; }
	for(int i=0;i<points_.size();i++)
	{
		points_[i] =
				gsl_poly_eval(
						cc,
						DEGREE,
						((double)i+1)/(points_.size()/num_points_));
	}
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(Xj);
	gsl_vector_free(c);

#endif

}

void polyCurveFitPoint(
	vector<Vector4d> points_,
	vector<Vector4d> &points_est_,
	vector<Vector3d> &coeffs_,
	vector<Vector3d> &covs_,
	bool flag_est_)
{
	int num_points = points_.size();
	vector<double> x; x.resize(num_points);
	vector<double> y; y.resize(num_points);
	vector<double> z; z.resize(num_points);
	vector<double> cx; cx.resize(DEGREE);
	vector<double> cy; cy.resize(DEGREE);
	vector<double> cz; cz.resize(DEGREE);
	vector<double> covx; covx.resize(Sqr(DEGREE));
	vector<double> covy; covy.resize(Sqr(DEGREE));
	vector<double> covz; covz.resize(Sqr(DEGREE));
	for(int i=0;i<num_points;i++)
	{
		x[i] = points_[i](0);
		y[i] = points_[i](1);
		z[i] = points_[i](2);
	}
	polyCurveFit(x,cx,covx);
	polyCurveFit(y,cy,covy);
	polyCurveFit(z,cz,covz);
	reshapeVector(coeffs_,DEGREE);
	for(int i=0;i<DEGREE;i++)
	{
		coeffs_[i](0) = cx[i];
		coeffs_[i](1) = cy[i];
		coeffs_[i](2) = cz[i];
	}
	reshapeVector(covs_,Sqr(DEGREE));
	for(int i=0;i<Sqr(DEGREE);i++)
	{
		covs_[i](0) = covx[i];
		covs_[i](1) = covy[i];
		covs_[i](2) = covz[i];
	}
	int num_points2 = points_est_.size();
	vector<double> x2; x2.resize(num_points2);
	vector<double> y2; y2.resize(num_points2);
	vector<double> z2; z2.resize(num_points2);
	if(flag_est_)
	{
		polyCurveFitEst(x2, num_points, cx, covx);
		polyCurveFitEst(y2, num_points, cy, covy);
		polyCurveFitEst(z2, num_points, cz, covz);
		for(int i=0;i<num_points2;i++)
		{
			points_est_[i](0) = x2[i];
			points_est_[i](1) = y2[i];
			points_est_[i](2) = z2[i];
			points_est_[i](3) = UNCLASSIFIED;
		}
		// ##TODO hack
		for(int i=0;i<4;i++)
		{
			points_est_.erase(points_est_.begin());
			points_est_.pop_back();
		}
	}
}

double curveIntegral (
	double 	x,
	void 	*params)
{
	double *cc = (double *)params;
	double dfx[2], dfy[2], dfz[2];
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	for(int i=0;i<DEGREE;i++)
	{
		cx[i] = cc[i*3+0];
		cy[i] = cc[i*3+1];
		cz[i] = cc[i*3+2];
	}
	gsl_poly_eval_derivs (cx, DEGREE, x, dfx, 2);
	gsl_poly_eval_derivs (cy, DEGREE, x, dfy, 2);
	gsl_poly_eval_derivs (cz, DEGREE, x, dfz, 2);
	double f = sqrt(Sqr(dfx[1])+Sqr(dfy[1])+Sqr(dfz[1]));
	return f;
}

void polyCurveLength(
	double &length_,
	double a_,
	double b_,
	vector<Vector3d> coeffs_)
{
	gsl_function F;
	gsl_integration_glfixed_table *table;
	double cc[3*DEGREE];
	table = gsl_integration_glfixed_table_alloc(DEGREE*2);
//	table = gsl_integration_glfixed_table_alloc((DEGREE+1)/2);
	for(int i=0;i<DEGREE;i++)
	{
		for(int ii=0;ii<3;ii++)
		{
			cc[i*3+ii] = coeffs_[i](ii);
		}
	}
	F.function = &curveIntegral;
	F.params = cc;
	length_ = gsl_integration_glfixed (&F, a_, b_, table);
	gsl_integration_glfixed_table_free (table);
}

//// ============================================================================
//// Surface
//// ============================================================================
//
//double surfaceRange(
//	point_d pos_,
//	point_d pos_surface_,
//	vector<double> surface_)
//{
//	vector<double> surface_tmp; surface_tmp.resize(3);
//	vector<double> surface_parallel_tmp;
//	vector<double> p_tmp;
//	double norm_tmp = 0.0;
//
//	surface_tmp[0] = surface_[0];
//	surface_tmp[1] = surface_[1];
//	surface_tmp[2] = surface_[2];
//
//	p_tmp = point2vector(minusPoint(pos_, pos_surface_));
//
//	// surface_parallel_tmp can be obtained at the beginning by just taking the normalized vector between 2 points on the surface
//	{
//		surface_parallel_tmp = crossProduct(surface_tmp, p_tmp);
//
//		for(int i=0;i<3;i++)
//			norm_tmp += Sqr(surface_parallel_tmp[i]);
//		norm_tmp = sqrt(norm_tmp);
//
//		for(int i=0;i<3;i++)
//			surface_parallel_tmp[i] /= norm_tmp;
//	}
//
//	return dotProduct(p_tmp, surface_parallel_tmp);
//}
//
//double surfaceDistance(
//	point_d pos_,
//	vector<double> surface_)
//{
//	return surface_[0]*pos_.x +
//		   surface_[1]*pos_.y +
//		   surface_[2]*pos_.z -
//		   surface_[3];
//}
//
//double surfaceAngle(
//	point_d vel_,
//	vector<double> surface_)
//{
//	vector<double> tmp; tmp.resize(3);
//	tmp[0] = surface_[0];
//	tmp[1] = surface_[1];
//	tmp[2] = surface_[2];
//	return 	l2Norm(crossProduct(tmp,point2vector(vel_)))/
//			(l2Norm(tmp) * l2Norm(point2vector(vel_)));
//}
