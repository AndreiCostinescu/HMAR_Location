/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

//#define DBSCAN

#include "util.h"
#include "vtkExtra.h"

// For backward compatibility with new VTK generic data arrays
//#define InsertNextTypedTuple InsertNextTupleValue

// ============================================================================
// dbscan
// ============================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_t *p)
{
	if(num_points)
    {
        dbscan(p, num_points, epsilon, minpts, euclidean_dist);

#ifdef DBSCAN
        printf("Epsilon: %lf\n", epsilon);
        printf("Minimum points: %u\n", minpts);
        print_points(p, num_points);
#endif

    }
	else
		cerr << "NO POINTS FOR CLUSTERING!!!";
}

void combineNearCluster(
	vector<point_t> &p,
	vector<point_t> &locations)
{
	int num_locations = 0;
	for(int i=0;i<p.size();i++)
		num_locations = max(p[i].cluster_id,num_locations);
	num_locations += 1;

	int num_points = p.size();

	// calculating the centroid of cluster
	vector<point_t> p_tmp0(num_locations);
	vector<point_t> p_tmp1(num_locations);
	for(int i=0;i<num_points;i++)
	{
		if(p[i].cluster_id >= 0)
		{
			p_tmp1[p[i].cluster_id].cluster_id += 1;
			p_tmp0[p[i].cluster_id].x += p[i].x;
			p_tmp0[p[i].cluster_id].y += p[i].y;
			p_tmp0[p[i].cluster_id].z += p[i].z;
		}
	}

	for(int i=0;i<num_locations;i++)
	{
		p_tmp1[i].x = p_tmp0[i].x/p_tmp1[i].cluster_id;
		p_tmp1[i].y = p_tmp0[i].y/p_tmp1[i].cluster_id;
		p_tmp1[i].z = p_tmp0[i].z/p_tmp1[i].cluster_id;
		p_tmp1[i].cluster_id = UNCLASSIFIED;
	}

	// combine cluster if it is less than 0.1m
	bool limit = false;
	for(int i=0;i<num_locations;i++)
	{
		for(int j=0;j<num_locations;j++)
		{
			if(j<=i) continue;

			for(int ii=0;ii<num_points;ii++)
				if(p[ii].cluster_id == i && !limit)
					for(int jj=0;jj<num_points;jj++)
						if(p[jj].cluster_id == j)
							if(l2Norm(minusPoint(p[ii],p[jj]))<0.1)
								limit = true;

			if(limit)
			{
				limit = false;

				if(p_tmp1[i].cluster_id>=0 && p_tmp1[j].cluster_id>=0)
				{
					int big   = max(p_tmp1[i].cluster_id,
							        p_tmp1[j].cluster_id);
					int small = min(p_tmp1[i].cluster_id,
							        p_tmp1[j].cluster_id);
					for(int ii=0;ii<num_locations;ii++)
					{
						if(p_tmp1[ii].cluster_id == big)
						   p_tmp1[ii].cluster_id = small;
					}
				}
				else if(p_tmp1[i].cluster_id>=0)
					    p_tmp1[j].cluster_id = p_tmp1[i].cluster_id;
				else if(p_tmp1[j].cluster_id>=0)
					    p_tmp1[i].cluster_id = p_tmp1[j].cluster_id;
				else
				{
					if(i<j)
					{
						p_tmp1[i].cluster_id = i;
						p_tmp1[j].cluster_id = i;
					}
					else
					{
						p_tmp1[i].cluster_id = j;
						p_tmp1[j].cluster_id = j;
					}
				}
			}
			else
			{
				if(p_tmp1[i].cluster_id!=(int)i && p_tmp1[i].cluster_id<0)
				   p_tmp1[i].cluster_id = i;
				if(p_tmp1[j].cluster_id!=(int)j && p_tmp1[j].cluster_id<0)
				   p_tmp1[j].cluster_id = j;
			}
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].cluster_id);
	}

	// removing the missing cluster labels
	int c = 1;
	for(int i=1;i<num_locations;i++)
	{
		if(p_tmp1[i].cluster_id > p_tmp1[i-1].cluster_id &&
		   p_tmp1[i].cluster_id == i)
		{
			p_tmp1[i].cluster_id = c;
			for(int ii=i+1;ii<num_locations;ii++)
				if(p_tmp1[ii].cluster_id == i)
				   p_tmp1[ii].cluster_id = c;
			c++;
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].cluster_id );
	}

	// updating cluster label
	for(int i=0;i<num_points;i++)
	{
		if (p[i].cluster_id >= 0)
			p[i].cluster_id = p_tmp1[p[i].cluster_id].cluster_id;
		//printf("Location %02d: %02d\n", line_, p[i].cluster_id );
	}

	// calculate the centroid of combined clusters
	vector<point_t> p_tmp2  (c);
	vector<point_t> p_center(c);

	for(int i=0;i<num_points;i++)
	{
		if(p[i].cluster_id >= 0)
		{
			p_center[p[i].cluster_id].cluster_id += 1;
			p_tmp2  [p[i].cluster_id].x += p[i].x;
			p_tmp2  [p[i].cluster_id].y += p[i].y;
			p_tmp2  [p[i].cluster_id].z += p[i].z;
		}
		//printf("Location %02d: %02d %02d\n", i, p[i].cluster_id, p_center  [p[i].cluster_id].cluster_id );
	}

	for(int i=0;i<c;i++)
	{
		p_center[i].x = p_tmp2[i].x/p_center[i].cluster_id;
		p_center[i].y = p_tmp2[i].y/p_center[i].cluster_id;
		p_center[i].z = p_tmp2[i].z/p_center[i].cluster_id;
		p_center[i].cluster_id = UNCLASSIFIED;
		//printf("Location %02d: %+.4f %+.4f %+.4f\n", i, p_center[i].x, p_center[i].y, p_center[i].z );
	}

	//cout << num_locations << c << endl;

	locations.clear(); locations = p_center;
}

void decideBoundary(
	point_t &p,
	vector<point_t> location,
	vector<double> location_boundary)
{
	double location_contact = 0.0;
	point_t tmp_diff;
	location_contact = 0.0;
	p.cluster_id = UNCLASSIFIED;
	for(int ii=0;ii<location.size();ii++)
	{
		tmp_diff = minusPoint(p, location[ii]);
		if (max_( pdfExp( 0.05, 0.0, l2Norm(tmp_diff) ), location_contact ) )
		{
			location_contact = pdfExp( 0.05, 0.0, l2Norm(tmp_diff) );
			if(max_(location_contact,location_boundary[ii]))
				p.cluster_id = ii;
		}
		//cout << location[ii].x << " "<< location[ii].y << " "<< location[ii].z << " ";
		//cout << pdfExp( 0.05, 0.0, l2Norm(tmp_diff) ) << "";
	}
	//cout << p.cluster_id << "" << endl;
}

void contactBoundary(
	vector<point_t> &p,
	vector<point_t> locations,
	vector<double> &location_boundary,
	bool learn)
{
	//if (learn)
		for(int i=0;i<locations.size();i++)
			location_boundary[i] = 0.95; //1.0 is the max

	for (int i=0;i<p.size();i++)
	{
		if (learn)
		{
//			if (p[i].cluster_id<0) continue;
//			point_t tmp_diff = minusPoint(point2point3D(p[i]), location[p[i].cluster_id]);
//			location_boundary[p[i].cluster_id] =
//					min( pdfExp( 0.05, 0.0, l2Norm(tmp_diff) ),
//					     location_boundary[p[i].cluster_id]);
			continue;
		}
		else
			decideBoundary(p[i], locations, location_boundary);
	}
}

// ============================================================================
// B-spline
// ============================================================================

double curveIntegral (double x, void * params)
{
	double dfx[2], dfy[2], dfz[2];
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double *cc = (double *)params;
	for(int i=0;i<DEGREE;i++) cx[i] = cc[i+(DEGREE*0)];
	for(int i=0;i<DEGREE;i++) cy[i] = cc[i+(DEGREE*1)];
	for(int i=0;i<DEGREE;i++) cz[i] = cc[i+(DEGREE*2)];
	gsl_poly_eval_derivs (cx, DEGREE, x, dfx, 2);
	gsl_poly_eval_derivs (cy, DEGREE, x, dfy, 2);
	gsl_poly_eval_derivs (cz, DEGREE, x, dfz, 2);
	double f = sqrt(Sqr(dfx[1])+Sqr(dfy[1])+Sqr(dfz[1]));
	return f;
}

void curveFit(
	vector<point_t> points_,
	vector<point_t> &curves_)
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
	int num_points = points_.size();

	double chisq; //residual error

	gsl_multifit_linear_workspace *ws;
	gsl_matrix *cov, *X;
	gsl_vector *y, *c;

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

	ws = gsl_multifit_linear_alloc(num_points, DEGREE);
	gsl_multifit_linear(X, y, c, cov, &chisq, ws);

	reshapeVector(coeff_, DEGREE);
	for(int i=0;i<DEGREE;i++) coeff_[i] = gsl_vector_get(c, i);
//	coeff_[0] = 0.0;

	reshapeVector(cov_, Sqr(DEGREE));
	for(int i=0;i<Sqr(DEGREE);i++) cov_[i] = gsl_matrix_get(cov, i/DEGREE, i%DEGREE);

	gsl_multifit_linear_free(ws);
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(c);
}

void polyCurveFitPoint(
	vector<point_t> points_,
	vector<point_t> &points_est_,
	vector<point_t> &coeffs_,
	vector<point_t> &covs_,
	bool est_)
{
	int num_points = points_.size();

	vector<double> x(num_points);
	vector<double> y(num_points);
	vector<double> z(num_points);

	vector<double> cx(DEGREE);
	vector<double> cy(DEGREE);
	vector<double> cz(DEGREE);

	vector<double> covx(Sqr(DEGREE));
	vector<double> covy(Sqr(DEGREE));
	vector<double> covz(Sqr(DEGREE));

	for(int i=0;i<num_points;i++)
	{
		x[i] = points_[i].x;
		y[i] = points_[i].y;
		z[i] = points_[i].z;
	}

	polyCurveFit(x,cx,covx);
	polyCurveFit(y,cy,covy);
	polyCurveFit(z,cz,covz);

	reshapeVector(coeffs_,DEGREE);

	for(int i=0;i<DEGREE;i++)
	{
		coeffs_[i].x = cx[i];
		coeffs_[i].y = cy[i];
		coeffs_[i].z = cz[i];
	}

	reshapeVector(covs_,Sqr(DEGREE));

	for(int i=0;i<Sqr(DEGREE);i++)
	{
		covs_[i].x = covx[i];
		covs_[i].y = covy[i];
		covs_[i].z = covz[i];
	}

	int num_points2 = points_est_.size();

	vector<double> x2(num_points2);
	vector<double> y2(num_points2);
	vector<double> z2(num_points2);

	if(est_)
	{
		polyCurveFitEst(x2, num_points, cx, covx);
		polyCurveFitEst(y2, num_points, cy, covy);
		polyCurveFitEst(z2, num_points, cz, covz);

		for(int i=0;i<num_points2;i++)
		{
			points_est_[i].x = x2[i];
			points_est_[i].y = y2[i];
			points_est_[i].z = z2[i];
		}
	}
}

void polyCurveLength(
	double &length_,
	double a_,
	double b_,
	vector<point_t> coeffs_)
{
	double cc[DEGREE*3];

	gsl_function F;
	gsl_integration_glfixed_table *table;

	table = gsl_integration_glfixed_table_alloc ((DEGREE+1)/2);

	for(int i=0;i<DEGREE;i++)
	{
		cc[i+(DEGREE*0)] = coeffs_[i].x;
		cc[i+(DEGREE*1)] = coeffs_[i].y;
		cc[i+(DEGREE*2)] = coeffs_[i].z;
	}

	F.function = &curveIntegral;
	F.params = cc;

	length_ = gsl_integration_glfixed (&F, a_, b_, table);

	gsl_integration_glfixed_table_free (table);
}

void polyCurveFitEst(
	vector<double> &points_,
	int num_points_,
	vector<double> coeffs_,
	vector<double> covs_)
{
	gsl_matrix *cov, *X;
	gsl_vector *c, *Xj;

	double cc[DEGREE];

	X  = gsl_matrix_alloc(num_points_, DEGREE);
	Xj = gsl_vector_alloc(DEGREE);
	c  = gsl_vector_alloc(DEGREE);
	cov = gsl_matrix_alloc(DEGREE, DEGREE);

	for(int i=0;i<DEGREE;i++) cc[i] = coeffs_[i];

	for(int i=0;i<points_.size();i++)
		points_[i] =
				gsl_poly_eval(cc, DEGREE, (double)i/(points_.size()/num_points_));

	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(Xj);
	gsl_vector_free(c);
}

// ============================================================================
// Data
// ============================================================================

void parseData2Point(
	vector<vector<string> > data_full,
	vector<point_t> &points)
{
	for(int i=0;i<points.size();i++)
	{
		points[i].x = atof(data_full[i][2].c_str());
		points[i].y = atof(data_full[i][3].c_str());
		points[i].z = atof(data_full[i][4].c_str());
		points[i].cluster_id = UNCLASSIFIED;
	}
}

void preprocessDataLive(
	point_t pos_,
	vector< vector< point_t > > &pos_vel_acc_mem_, // motion -> length(empty at beginning)
	vector<point_t> &pos_vel_acc_avg_, //motion
	unsigned int window_)
{
	point_t vel = minusPoint(pos_, pos_vel_acc_avg_[0]);
	point_t acc = minusPoint(vel , pos_vel_acc_avg_[1]);

	vector<point_t> tmp(3);
	tmp[0] = pos_; tmp[1] = vel; tmp[2] = acc;

	for(int i=0;i<3;i++)
	{
		if(pos_vel_acc_mem_[i].size() == window_)
		{
			pos_vel_acc_avg_[i] =
					movingAverage(tmp[i], pos_vel_acc_mem_[i]);
		}
		else if (pos_vel_acc_mem_[i].size()>0)
		{
			pos_vel_acc_avg_[i] =
					averagePointIncrement(tmp[i], pos_vel_acc_mem_[i]);
		}
		else
		{
			pos_vel_acc_mem_[i].push_back(tmp[i]);
			pos_vel_acc_avg_[i] = tmp[i];
			for(int ii=i+1;ii<3;ii++)
			{
				pos_vel_acc_avg_[ii].cluster_id = UNCLASSIFIED;
				pos_vel_acc_avg_[ii].x =
				pos_vel_acc_avg_[ii].y =
				pos_vel_acc_avg_[ii].z = 0;
			}
			break;
		}
		pos_vel_acc_avg_[i].cluster_id = UNCLASSIFIED;
	}
}

// ============================================================================
// Files
// ============================================================================

void writeSurfaceFile(
	Graph Graph_)
{
	string path = "./Scene/" + Graph_.getScene() + "/surface.txt";

	vector<vector<double> > surface = Graph_.getSurface();

	if (!ifstream(path))
	{
		ofstream write_file(path, std::ios::app);
		for(int i=0;i<surface.size();i++)
		{
			write_file << i;
			for(int ii=0;ii<4;ii++)
			{
				write_file << ",";
				write_file << surface[i][ii];
			}
		}
		write_file << "\n";
	}
}

void writeLocLabelFile(
	string path_,
	vector<string> label_,
	vector<point_t> locations_,
	vector<double> boundary_,
	vector<int> surface_num_container_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		for(int i=0;i<locations_.size();i++)
		{
			write_file << label_[i+1]     << ","
					   << locations_[i].x << ","
					   << locations_[i].y << ","
					   << locations_[i].z << ","
					   << boundary_[i]    << ","
					   << surface_num_container_[i];
			if (i<locations_.size()-1)
				write_file << ",";
		}
		write_file << "\n";
	}
}

void writeMovLabelFile(
	string path_,
	vector<string> label_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		for(int i=0;i<label_.size();i++)
		{
			write_file << label_[i];
			if (i<label_.size()-1)
				write_file << ",";
		}
		write_file << "\n";
	}
}

void writeSectorFile(
	Graph Graph_,
	string path_,
	int maxminconst_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	sector_para_t para = Graph_.getSectorPara();

	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << "Locations," 			<< num_locations 	<< "\n";
		write_file << "Location Intervals," << para.loc_int 	<< "\n";
		write_file << "Sector Intervals," 	<< para.sec_int 	<< "\n";
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				vector<sector_t> sector     = edges[i][ii].sector_map;
				vector<double> sector_const = edges[i][ii].sector_const;
				write_file << "Edge,"    << i
						   << ",Number," << ii << "\n";
				for(int iii=0;iii<sector.size();iii++)
				{
					if (maxminconst_ == 0)
						write_file << sector[iii].max;
					else if(maxminconst_ == 1)
						write_file << sector[iii].min;
					else
						write_file << sector_const[iii];

					if (iii<sector.size()-1)
						write_file << ",";
					else
						write_file << "\n";
				}
			}
		}
	}
}

void writeSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > sector_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << num_locations_ 		  << "\n";
		write_file << num_location_intervals_ << "\n";
		write_file << num_sector_intervals_   << "\n";
		for(int i=0;i<sector_.size();i++)
		{
			for(int ii=0;ii<sector_[i].size();ii++)
			{
				for(int iii=0;iii<sector_[i][ii].size();iii++)
				{
					write_file << sector_[i][ii][iii];
					if (i<sector_[i][ii].size()-1)
						write_file << ",";
				}
				write_file << "\n";
			}
		}
	}
}

void readFile(
	const char *name,
	vector<vector<string> > &data_full,
	char delimiter)
{
	ifstream src_file( name );
	while (src_file)
	{
		string file_line_;
		if (!getline( src_file, file_line_ )) break;
		istringstream line_( file_line_ );
		vector <string> data_line_;
		while (line_)
		{
		   string word;
		   if (!getline( line_, word, delimiter)) break;
		   data_line_.push_back( word );
		}
		data_full.push_back( data_line_ );
	}

	if (!src_file.eof())
		cerr << "FILE ERROR!!!\n";
}

void readSurfaceFile(
	Graph &Graph_)
{
	string path;
	vector<vector<string> > data;

	path = "./Scene/" + Graph_.getScene() + "/surface.txt";
	readFile(path.c_str(), data, ',');

	vector<vector<double> > surface_;
	reshapeVector(surface_, data.size());

	for(int i=0;i<data.size();i++)
	{
		int tmp = atoi(data[i][0].c_str());
		for(int ii=0;ii<4;ii++)
			surface_[tmp].push_back(atof(data[i][ii+1].c_str()));
	}

	Graph_.addSurface(surface_);
}

void readLocation(
	string path_,
	vector<string> &label_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_,
	vector<int> &surface_num_)
{
	int num_locations = locations_.size();

	vector<vector<string> > data;
	readFile(path_.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Location data is empty.");
	}
	else
	{
		int data_tmp  = 6;
		num_locations = data[0].size()/data_tmp;
		reshapeVector(surface_num_,		  num_locations);
		reshapeVector(locations_,		  num_locations);
		reshapeVector(location_boundary_, num_locations);
		reshapeVector(label_,		      num_locations+1);
		label_[0]  = {"CONNECTION"};
		for(int ii=0;ii<num_locations;ii++)
		{
			label_[ii+1] 		  =      data[0][ii*data_tmp+0];
			locations_[ii].x 		  = atof(data[0][ii*data_tmp+1].c_str());
			locations_[ii].y 		  = atof(data[0][ii*data_tmp+2].c_str());
			locations_[ii].z 		  = atof(data[0][ii*data_tmp+3].c_str());
			locations_[ii].cluster_id = UNCLASSIFIED;
			location_boundary_[ii] 	  = atof(data[0][ii*data_tmp+4].c_str());
			surface_num_[ii] 	  	  = atof(data[0][ii*data_tmp+5].c_str());
		}
	}

	printf("Reviewing location labels...\n");
	for(int i=0;i<num_locations;i++)
		printf("LLabel %d : %s\n", i+1, label_[i+1].c_str());

	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<num_locations;i++)
			{
				printf("Enter new label for LLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					label_[i+1] = mystr;
				}
			}
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
}

void readLocation_(
	Graph &Graph_)
{
	string path;
	path =  "./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data.txt";

	vector<string> 	label;
	vector<point_t> locations;
	vector<double> 	location_boundary;
	vector<int> 	surface_num;
	vector<data_t> 	data_;

	readLocation(path, label, locations, location_boundary, surface_num);

	for(int i=0;i<locations.size();i++)
		Graph_.addNode(label[i+1], i, -1,
					   locations[i], location_boundary[i],
					   surface_num[i], data_);

	remove(path.c_str());
	writeLocLabelFile(path, label, locations, location_boundary, surface_num);
}

void readMovement(
	Graph &Graph_)
{
	vector<string> label;

	string path;
	path =  "./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/mov_data.txt";

	vector<vector<string> > data;
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Movement data is empty.");
	}
	else
	{
		reshapeVector(label, data[0].size());
		for(int ii=0;ii<data[0].size();ii++)
			label[ii] = data[0][ii];
	}
	printf("Reviewing movement labels for OBJ...\n");
	for(int i=0;i<data[0].size();i++)
		printf("MLabel %d : %s\n", i, label[i].c_str());

	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<data[0].size();i++)
			{
				printf("Enter new label for MLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					label[i] = mystr;
				}
			}
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}

	Graph_.updateMovLabel(label);

	remove(path.c_str());
	writeMovLabelFile(path,label);
}

void readSectorFile(
	Graph &Graph_,
	int maxminconst_)
{
	vector<data_t> data_zero;

	string path;
	switch(maxminconst_)
	{
		case 0:
			path =  "./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_max.txt";
			break;
		case 1:
			path =  "./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_min.txt";
			break;
		case 2:
			path =  "./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_const.txt";
			break;
	}

	vector<vector<string> > data;
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Sector data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][1].c_str());
		num_location_intervals 	= atoi(data[1][1].c_str());
		num_sector_intervals 	= atoi(data[2][1].c_str());

		if (Graph_.getEdgeList().empty())
			Graph_.initEdge(num_location_intervals,num_sector_intervals);

		vector<vector<edge_tt> > edges = Graph_.getEdgeList();

		for(int i=0;i<Sqr(num_locations);i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				vector<sector_t> sector_map = edges[i][ii].sector_map;
				vector<double> sector_const = edges[i][ii].sector_const;
				for(int iii=0;iii<num_location_intervals*num_sector_intervals;iii++)
				{
					int tmp = 3 + (2*i) + 1;
					switch(maxminconst_)
					{
						case 0:
							sector_map[iii].max = atof(data[tmp][iii].c_str());
							break;
						case 1:
							sector_map[iii].min = atof(data[tmp][iii].c_str());
							break;
						case 2:
							sector_const[iii]   = atof(data[tmp][iii].c_str());
							break;
					}
				}
				switch(maxminconst_)
				{
					case 0:
						Graph_.addEdge(data_zero, sector_map, i/num_locations, i%num_locations, ii);
						break;
					case 1:
						Graph_.addEdge(data_zero, sector_map, i/num_locations, i%num_locations, ii);
						break;
					case 2:
						Graph_.updateEdgeConst(sector_const, i/num_locations, i%num_locations, ii);
						break;
				}
			}
		}
	}
	// Should we add the option to edit the data?
}

void readSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > &sector_)
{
	vector<vector<string> > data;
	readFile(path_.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Sector data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][0].c_str());
		num_location_intervals 	= atoi(data[1][0].c_str());
		num_sector_intervals 	= atoi(data[2][0].c_str());

	    reshapeVector(sector_,Sqr(num_locations));
	    vector<vector<double> > sector1(num_location_intervals);
		       vector<double> 	sector2(num_sector_intervals);

		for(int i=0;i<Sqr(num_locations);i++)
		{
			sector_[i] = sector1;
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				sector_[i][ii] = sector2;
			}
		}

		for(int i=0;i<num_locations;i++)
		{
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				int tmp = i*num_location_intervals+ii+3;
				for(int iii=0;iii<num_sector_intervals;iii++)
				{
					sector_[i][ii][iii] = atof(data[tmp][iii].c_str());
				}
			}
		}
	}
	// Should we add the option to edit the data?
}

// ============================================================================
// Sector Curve
// ============================================================================

void cal_inverse_intergal(
	point_t t_,
	point_t &p_tan_,
	point_t &p_nor_,
	vector<point_t> coeff,
	int dim) //degree+1
{
	vector<point_t> coeff_diff(dim-1);
	for(int i=0;i<dim-1;i++)
	{
		coeff_diff[i].x = i * coeff[i+1].x;
		coeff_diff[i].y = i * coeff[i+1].y;
		coeff_diff[i].z = i * coeff[i+1].z;
	}
	vector<point_t> coeff_sqr(2*(dim-1)-1);
	int c=0;
	for(int i=0;i<dim-1;i++)
	{
		for(int ii=0;ii<dim-1;ii++)
		{
			coeff_sqr[c+ii].x = coeff[i].x * coeff[ii].x;
			coeff_sqr[c+ii].y = coeff[i].y * coeff[ii].y;
			coeff_sqr[c+ii].z = coeff[i].z * coeff[ii].z;
		}
		c++;
	}
	//...
}

void determineLocationInterval(
	int &ind_loc_,
	int &ind_loc_last,
	int total_loc_int_,
	point_t point_,
	vector<point_t> tangent_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_)
{
	double d1,d2,d3,d4,d5;
	point_t proj_dir_tmp;
	for(int l=(ind_loc_last<0?0:ind_loc_last);l<total_loc_int_;l++)
	{
		proj_dir_tmp =
				multiPoint(
						tangent_[l],
						dotProduct(
								point2vector(minusPoint(point_,mid_[l])),
								point2vector(tangent_[l])));
		d1 = l2Norm(minusPoint(beg_[l],mid_[l]));
		d2 = l2Norm(minusPoint(end_[l],mid_[l]));
		d3 = l2Norm(minusPoint(beg_[l],addPoint(mid_[l],proj_dir_tmp)));
		d4 = l2Norm(minusPoint(end_[l],addPoint(mid_[l],proj_dir_tmp)));
		d5 = l2Norm(minusPoint(beg_[l],end_[l]));
		if (checkDirection(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			if (d4<=d2 && (d3-d5)<0.001) {ind_loc_ = l;break;} //### small error deviation (deadzone)
		}
		else
		{
			if (d3<=d1 && (d4-d5)<0.001) {ind_loc_ = l;break;}
		}
	}
	// to prevent unknown locations at start and end
	if (ind_loc_<0) ind_loc_ = ind_loc_last; ind_loc_last = ind_loc_;
}

void determineSectorInterval(
	int &ind_sec_,
	int ind_loc_,
	int total_sec_int_,
	point_t &delta_t_,
	point_t point_,
	vector<point_t> tangent_,
	vector<point_t> normal_,
	vector<point_t> mid_)
{
	point_t proj_dir =
			multiPoint(
					tangent_[ind_loc_],
					dotProduct(
							point2vector(
									minusPoint(
											point_,
											mid_[ind_loc_])),
							point2vector(tangent_[ind_loc_])));
	delta_t_ =
			minusPoint(
					point_,
					addPoint(proj_dir, mid_[ind_loc_]));
	double angle_tmp =
			atan2(
					l2Norm(
							crossProduct(
									point2vector(delta_t_),
									point2vector(normal_[ind_loc_]))),
					dotProduct(
							point2vector(delta_t_),
							point2vector(normal_[ind_loc_])));
	if (!checkDirection(
			crossProduct(
					point2vector(normal_[ind_loc_]),
					point2vector(multiPoint(delta_t_,1/l2Norm(delta_t_)))),
			point2vector(tangent_[ind_loc_])))
	{angle_tmp *= -1;}
	angle_tmp = fmod((2*M_PI + angle_tmp),(2*M_PI));
	ind_sec_ = ceil(angle_tmp*(total_sec_int_/2)/M_PI) - 1 ;
}

void adjustSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg,
	int edge_xy_,
	int curr_num_,
	int tmp_id1_,
	int tmp_id2_,
	int tmp_id3_,
	vector<vector<double> > kernel_)
{
	int loc_int = Graph_.getSectorPara().loc_int;
	int sec_int = Graph_.getSectorPara().sec_int;
	double N 	= Graph_.getCounter(edge_xy_,0);
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double total_len 			= Graph_.getEdgeList()[edge_xy_][0].total_len;
	vector<point_t> loc_beg 	= Graph_.getEdgeList()[edge_xy_][0].loc_start;
	vector<point_t> loc_mid		= Graph_.getEdgeList()[edge_xy_][0].loc_mid;
	vector<point_t> loc_end		= Graph_.getEdgeList()[edge_xy_][0].loc_end;
	vector<point_t> tangent 	= Graph_.getEdgeList()[edge_xy_][0].tangent;
	vector<point_t> normal 		= Graph_.getEdgeList()[edge_xy_][0].normal;
	vector<sector_t> sector_map = Graph_.getEdgeList()[edge_xy_][0].sector_map;
	vector<point_t> loc_beg_mem = loc_beg;
	vector<point_t> loc_mid_mem	= loc_mid;
	vector<point_t> loc_end_mem = loc_end;
	vector<point_t> tangent_mem = tangent;
	vector<point_t> normal_mem  = normal;
	vector<point_t> points_tmp, points_est_zero, covs, coeffs;
	for(int i=tmp_id3_;i<curr_num_;i++) points_tmp.push_back(pva_avg[i][0]);
	reshapeVector(points_est_zero,(points_tmp.size())*10);
	reshapeVector(coeffs,DEGREE);

	// [CURVE FIT]*************************************************************
	polyCurveFitPoint(points_tmp, points_est_zero, coeffs, covs, true);
	polyCurveLength(total_len, 0, points_tmp.size()-1, coeffs);
	for(int i=0;i<DEGREE;i++)
	{cx[i] = coeffs[i].x; cy[i] = coeffs[i].y; cz[i] = coeffs[i].z;}
	for(int i=0;i<loc_int;i++)
	{
		double t_beg 	= -1;
		double t_mid  	= -1;
		double t_end  	= -1;
		double len_inc1 =  total_len*(i)  /loc_int;
		double len_inc2 = (total_len*(i)  /loc_int) +
						  (0.5*total_len  /loc_int);
		double len_inc3 =  total_len*(i+1)/loc_int;
		// ### Shortcut: resample points along curve and cal length.
		for(int ii=0;ii<(points_tmp.size()-1)*100 + 1;ii++)
		{
			double tmplen 	= 0.0;
			double t 		= (double)ii/100;
			polyCurveLength(tmplen, 0, t, coeffs);
			if      (tmplen>=len_inc1 && t_beg<0)
			{t_beg = t;}
			else if (tmplen>=len_inc2 && t_mid<0)
			{t_mid  = t;}
			else if (tmplen>=len_inc3 && t_end<0)
			{t_end  = t; break;}
		}
		point_t p_mid, p_tan, p_nor;
		p_mid.x = gsl_poly_eval (cx, DEGREE, t_mid);
		p_mid.y = gsl_poly_eval (cy, DEGREE, t_mid);
		p_mid.z = gsl_poly_eval (cz, DEGREE, t_mid);
		if (N>0)
			cal_tangent_normal(t_mid, p_tan, p_nor, coeffs, DEGREE, false);
		else
			cal_tangent_normal(t_mid, p_tan, p_nor, coeffs, DEGREE, true);
		loc_mid	 [i] = p_mid;
		loc_beg  [i] = addPoint( p_mid , multiPoint(p_tan, t_beg-t_mid) );
		loc_end  [i] = addPoint( p_mid , multiPoint(p_tan, t_end-t_mid) );
		normal   [i] = multiPoint( p_nor     , 1/l2Norm(p_nor)     );
		tangent  [i] = multiPoint( p_tan     , 1/l2Norm(p_tan)     );
	}
	// *************************************************************[CURVE FIT]

	// [ADJUSTMENT]************************************************************
	vector<sector_t> sector_map_new(sec_int*loc_int);
	if (N>0)
	{
		// [AVERAGE]***********************************************************
		int loc_last = -1;
		for(int l=0;l<loc_int;l++)
		{
			loc_beg[l] = addPoint( multiPoint( loc_beg_mem[l] , N/(N+1) ) ,
								   multiPoint( loc_beg[l] 	  , 1/(N+1) ) );
			loc_mid[l] = addPoint( multiPoint( loc_mid_mem[l] , N/(N+1) ) ,
								   multiPoint( loc_mid[l] 	  , 1/(N+1) ) );
			loc_end[l] = addPoint( multiPoint( loc_end_mem[l] , N/(N+1) ) ,
								   multiPoint( loc_end[l] 	  , 1/(N+1) ) );
			tangent[l] = addPoint( multiPoint( tangent_mem[l] , N/(N+1) ) ,
								   multiPoint( tangent[l] 	  , 1/(N+1) ) );
			tangent[l] = multiPoint( tangent[l], 1/l2Norm(tangent[l]) );
			vector<double> tmpRTI =
					transInv(rodriguezRot(tangent_mem[l],tangent[l]));
			normal[l].x = tmpRTI[0]*normal_mem[l].x +
						  tmpRTI[1]*normal_mem[l].y +
						  tmpRTI[2]*normal_mem[l].z;
			normal[l].y = tmpRTI[3]*normal_mem[l].x +
						  tmpRTI[4]*normal_mem[l].y +
						  tmpRTI[5]*normal_mem[l].z;
			normal[l].z = tmpRTI[6]*normal_mem[l].x +
						  tmpRTI[7]*normal_mem[l].y +
						  tmpRTI[8]*normal_mem[l].z;
		}
		// ***********************************************************[AVERAGE]
		for(int l=0;l<loc_int;l++)
		{
			for(int s=0;s<sec_int;s++)
			{
				int ind_loc = -1;
				int ind_sec = -1;
				point_t delta_t, tmpN, p_old;
				// [OLD POINT]*************************************************
				tmpN = rodriguezVec(
								2*M_PI*fmod((s+0.5),(double)sec_int)/sec_int,
								tangent_mem[l],
								normal_mem[l]);
				p_old = addPoint(
								loc_mid_mem[l],
								multiPoint(
										tmpN,
										sector_map[l*sec_int+s].max));
				// *************************************************[OLD POINT]
				// [LOCATION INTERVAL]*****************************************
				determineLocationInterval(
						ind_loc, loc_last, loc_int, p_old, tangent,
						loc_beg, loc_mid, loc_end);
				// *****************************************[LOCATION INTERVAL]
				// [SECTOR INTERVAL]*******************************************
				determineSectorInterval(
						ind_sec, ind_loc, sec_int, delta_t, p_old,
						tangent, normal, loc_mid);
				// *******************************************[SECTOR INTERVAL]
				// [SECTOR MAP]************************************************
				int ind_ls  = ind_loc*sec_int + ind_sec;
				if (ind_loc < loc_int &&
					ind_sec < sec_int)
				{
					sector_map_new[ind_ls].max =
							max(
									sector_map_new[ind_ls].max,
									l2Norm(delta_t));
					for(int i=0;i<kernel_.size();i++)
					{
						for(int ii=0;ii<kernel_[0].size();ii++)
						{
							// extra calculation due to roundness of sector
							// ignoring kernel elements that go out of location areas
							int tmpl = ii - (kernel_[0].size()/2) + ind_loc;
							int tmps = ((i - (kernel_.size()/2) + ind_sec) +
										sec_int) % sec_int;
							int tmp3 = tmpl*sec_int + tmps;
							if (tmpl < 0 || tmpl >= loc_int)
								continue;
							sector_map_new[tmp3].max =
									max(
											sector_map_new[tmp3].max,
											sector_map_new[ind_ls].max); //###NO GUASSIAN
						}
					}
				}
				// ************************************************[SECTOR MAP]
			} //s
		} //l
	}
	// ************************************************************[ADJUSTMENT]

	Graph_.updateEdgeLocStartMidEnd(loc_beg, loc_mid, loc_end, tmp_id1_, tmp_id2_, 0);
	Graph_.updateEdgeLocDist(total_len, tmp_id1_, tmp_id2_, 0);
	Graph_.updateEdgeNormal(normal, tmp_id1_, tmp_id2_, 0);
	Graph_.updateEdgeTangent(tangent, tmp_id1_, tmp_id2_, 0);
	if (N>0) Graph_.updateEdgeSector(sector_map_new, tmp_id1_, tmp_id2_, 0);

}

void checkSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	int edge_xy_,
	int curr_num_,
	int tmp_id1_,
	int tmp_id2_,
	int tmp_id3_,
	vector<vector<double> > kernel_)
{

	int sec_int 				= Graph_.getSectorPara().sec_int;
	int loc_int 				= Graph_.getSectorPara().loc_int;
	int loc_last 				= 0;
	double total_len 			= Graph_.getEdgeList()[edge_xy_][0].total_len;
	vector<sector_t> sector_map = Graph_.getEdgeList()[edge_xy_][0].sector_map;
	vector<data_t> data_old 	= Graph_.getEdgeList()[edge_xy_][0].data;
	vector<point_t> tangent 	= Graph_.getEdgeList()[edge_xy_][0].tangent;
	vector<point_t> normal 		= Graph_.getEdgeList()[edge_xy_][0].normal;
	vector<point_t> loc_beg		= Graph_.getEdgeList()[edge_xy_][0].loc_start;
	vector<point_t> loc_mid		= Graph_.getEdgeList()[edge_xy_][0].loc_mid;
	vector<point_t> loc_end		= Graph_.getEdgeList()[edge_xy_][0].loc_end;
	data_t motion_data;
	vector<data_t> edge_data; edge_data.clear();
	point_t delta_t;

	for(int ii=tmp_id3_;ii<curr_num_;ii++)
	{
		int ind_loc = -1;
		int ind_sec = -1;
		// [LOCATION INTERVAL]*************************************************
		determineLocationInterval(
				ind_loc, loc_last, loc_int, pos_vel_acc_avg_[ii][0], tangent,
				loc_beg, loc_mid, loc_end);
		// *************************************************[LOCATION INTERVAL]
		// [SECTOR INTERVAL]***************************************************
		determineSectorInterval(
				ind_sec, ind_loc, sec_int, delta_t, pos_vel_acc_avg_[ii][0],
				tangent, normal, loc_mid);
		// ***************************************************[SECTOR INTERVAL]
		// [SECTOR MAP]********************************************************
		int ind_ls  = ind_loc*sec_int + ind_sec;
		if (ind_loc < loc_int && ind_sec < sec_int)
		{
			sector_map[ind_ls].max =
					max(l2Norm(delta_t), sector_map[ind_ls].max);
			sector_map[ind_ls].min =
					min(l2Norm(delta_t), sector_map[ind_ls].min);
			double mid = (sector_map[ind_ls].max + sector_map[ind_ls].min)/2;
			double tmp_ratio1 = (sector_map[ind_ls].max - mid) /
								 kernel_[(kernel_.size()-1)/2]
										[(kernel_[0].size()-1)/2];
			for(int i=0;i<kernel_.size();i++)
			{
				for(int ii=0;ii<kernel_[0].size();ii++)
				{
					// extra calculation due to roundness of sector
					// ignoring kernel elements that go out of location areas
					int tmpl = ii - (kernel_[0].size()/2) + ind_loc;
					int tmps = ((i - (kernel_.size()/2) + ind_sec) +
								sec_int) % sec_int;
					int tmp3 = tmpl*sec_int + tmps;
					if (tmpl < 0 || tmpl >= loc_int)
						continue;
					sector_map[tmp3].max =
							max((kernel_[i][ii]*tmp_ratio1) + mid ,
								sector_map[tmp3].max);
					sector_map[tmp3].min =
							min(mid-(kernel_[i][ii]*tmp_ratio1) ,
								sector_map[tmp3].min);
				}
			}
		}
		// ********************************************************[SECTOR MAP]
		motion_data.pos = pos_vel_acc_avg_[curr_num_][0];
		motion_data.vel = pos_vel_acc_avg_[curr_num_][1];
		motion_data.acc = pos_vel_acc_avg_[curr_num_][2];
		edge_data.push_back(motion_data);
	}
	if (!data_old.empty())
	{
		if(edge_data.size()==0)
			cout << "[WARNING] : Data to extend edge is empty." << endl;
		else if(edge_data.size()==1)
			data_old.push_back(edge_data[0]);
		else
			data_old.insert(
					data_old.end(), edge_data.begin(), edge_data.end());
	}
	Graph_.updateEdgeSector(sector_map, tmp_id1_, tmp_id2_, 0);
	Graph_.updateEdgeData(edge_data, tmp_id1_, tmp_id2_, 0);
	edge_data.clear();
}

void updateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg,
	vector<point_t> locations_,
	int curr_num_,
	int tmp_id1_,
	int tmp_id2_,
	int tmp_id3_,
	vector<vector<double> > kernel_)
{
	int edge_xy = tmp_id1_*locations_.size() + tmp_id2_;

	if (Graph_.getCounter(edge_xy,0) == 0)
	{
		adjustSectorCurve(
				Graph_, pva_avg, edge_xy, curr_num_,
				tmp_id1_, tmp_id2_, tmp_id3_, kernel_);
		checkSectorCurve(
				Graph_, pva_avg, edge_xy, curr_num_,
				tmp_id1_, tmp_id2_, tmp_id3_, kernel_);
		Graph_.incrementCounter(edge_xy,0);
	}
	else if (Graph_.getCounter(edge_xy,0) < 50)
	{
		checkSectorCurve(
				Graph_, pva_avg, edge_xy, curr_num_,
				tmp_id1_, tmp_id2_, tmp_id3_, kernel_);

//		vector<point_t> point_zero; vector<string> label_zero; bool flag = false;
//		for(int i=0;i<curr_num_;i++) point_zero.push_back(pva_avg[i][0]);
//		vector<unsigned char*> color_code(12);
//		for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
//		colorCode(color_code);
//		showConnection(point_zero, label_zero, Graph_, color_code, true);
//		printf("Viewing sector......Complete\n");

		adjustSectorCurve(
				Graph_, pva_avg, edge_xy, curr_num_,
				tmp_id1_, tmp_id2_, tmp_id3_, kernel_);

//		showConnection(point_zero, label_zero, Graph_, color_code, true);
//		printf("Viewing sector......Complete\n");

		Graph_.incrementCounter(edge_xy,0);
	}
	else
	{
		checkSectorCurve(
				Graph_, pva_avg, edge_xy, curr_num_,
				tmp_id1_, tmp_id2_, tmp_id3_, kernel_);
	}

//	vector<point_t> point_zero; vector<string> label_zero; bool flag = false;
//	for(int i=0;i<curr_num_;i++) point_zero.push_back(pva_avg[i][0]);
//	vector<unsigned char*> color_code(12);
//	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
//	colorCode(color_code);
//	showConnection(point_zero, label_zero, Graph_, color_code, true);
//	printf("Viewing sector......Complete\n");
}

void generateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg,
	vector<int> file_eof_,
	vector<vector<double> > kernel_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	int tmp_id1 = -1, tmp_id2 = -1, tmp_id3 = -1, file_num = 0;
	bool flag_next 	= false;

	for(int i=0;i<pva_avg.size();i++)
	{
		if(i==file_eof_[file_num]-1)
		{
			file_num++;
			flag_next = false;
			tmp_id1 = -1; // from
			tmp_id2 = -1; // to
			tmp_id3 = -1; // data number of from
			continue;
		}
		if (pva_avg[i][0].cluster_id >= 0)
		{
			// Initial location
			if (tmp_id1 < 0)
			{
				tmp_id1 = pva_avg[i][0].cluster_id;
				continue;
			}
			else
			{
				// Check if location has changed
				if (flag_next)
				{
					tmp_id2 = pva_avg[i][0].cluster_id;
					updateSectorCurve(Graph_, pva_avg, locations,
									  i, tmp_id1, tmp_id2, tmp_id3, kernel_);
					flag_next = false;
					tmp_id1 = tmp_id2;
					tmp_id2 = -1;
					tmp_id3 = -1;
				}
			}
		}
		else
		{
			// saves the data number of initial location
			if (tmp_id1 >=0 && tmp_id3 < 0)
			{
				tmp_id3 = i;
				flag_next = true;
			}
		}
	}
}

// ============================================================================
// Sector
// ============================================================================

void generateSector(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	vector<int> file_eof_,
	vector<vector<double> > kernel_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	int tmp_id1 = -1, tmp_id2 = -1, tmp_id3 = -1, file_num = 0;
	bool flag_next = false, flag_motion_start = true;
	data_t motion_data;
	vector<data_t> edge_data;
	edge_data.clear();

	for(int i=0;i<pos_vel_acc_avg_.size();i++)
	{
		if(i==file_eof_[file_num]-1)
		{
			file_num++;
			flag_next = false;
			tmp_id1 = -1;
			tmp_id2 = -1;
			tmp_id3 = -1;
			continue;
		}

		if (pos_vel_acc_avg_[i][0].cluster_id >= 0)
		{
			if (tmp_id1 < 0)
			{
				tmp_id1 = pos_vel_acc_avg_[i][0].cluster_id;
				continue;
			}
			else
			{
				if (flag_next)
				{
					tmp_id2 = pos_vel_acc_avg_[i][0].cluster_id;

					vector<sector_t> sector_map =
							Graph_.getEdgeList()[tmp_id1*num_locations+tmp_id2][0].sector_map;
					vector<data_t> data_old =
							Graph_.getEdgeList()[tmp_id1*num_locations+tmp_id2][0].data;

					for(int ii=tmp_id3;ii<i;ii++)
					{
						updateSector(sector_map, pos_vel_acc_avg_[ii][0], locations,
									 Graph_.getSectorPara(), tmp_id1, tmp_id2, kernel_);
						motion_data.pos = pos_vel_acc_avg_[ii][0];
						motion_data.vel = pos_vel_acc_avg_[ii][1];
						motion_data.acc = pos_vel_acc_avg_[ii][2];
						edge_data.push_back(motion_data);
					}
					if (!data_old.empty())
					{
						if(edge_data.size()==0)
							cout << "[WARNING] : Data to extend edge is empty." << endl;
						else if(edge_data.size()==1)
							data_old.push_back(edge_data[0]);
						else
							data_old.insert(data_old.end(), edge_data.begin(), edge_data.end());
					}

					Graph_.addEdge(edge_data,sector_map,tmp_id1,tmp_id2,0);

					edge_data.clear();
					flag_next = false;
					tmp_id1 = tmp_id2;
					tmp_id2 = -1;
					tmp_id3 = -1;
				}
			}
		}
		else
		{
			if (tmp_id1 >=0 && tmp_id3 < 0)
			{
				tmp_id3 = i;
				flag_next = true;
			}
		}
	}
}

void prepareSector(
	Graph &Graph_)
{
	sector_para_t sector_para = Graph_.getSectorPara();
	vector<node_tt> nodes = Graph_.getNodeList();

	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	int c = 0;
	for(int i=0;i<num_locations;i++)
	{
		for(int ii=0;ii<num_locations;ii++)
		{
			if(i == ii)
			{
				sector_para.dir [c].x =
				sector_para.dir [c].y =
				sector_para.dir [c].z = 0.0;
				sector_para.dist[c] = 0.0;
				sector_para.dir_n[c].x =
				sector_para.dir_n[c].y =
				sector_para.dir_n[c].z = 0.0;
				++c;
				continue;
			}
			sector_para.dir[c] = minusPoint(locations[ii],locations[i]);
			sector_para.dist[c] = l2Norm(sector_para.dir[c]);
			sector_para.dir[c].x /= sector_para.dist[c];
			sector_para.dir[c].y /= sector_para.dist[c];
			sector_para.dir[c].z /= sector_para.dist[c];
			vector<double> B(3);
			B[0] = 0.0; B[1] = 1.0; B[2] = 0.0;

			sector_para.dir_n[c] =
					vector2point(
							crossProduct(
									point2vector(sector_para.dir[c]), B));
			++c;
		}
	}

	Graph_.updateSectorPara(sector_para);
}

void updateSector(
	vector<sector_t> &sector_,
	point_t pos_,
	vector<point_t> locations_,
	sector_para_t sector_para_,
	int tmp_id1_,
	int tmp_id2_,
	vector<vector<double> > kernel_)
{
	double angle_tmp = 0.0, proj = 0.0;
	int xx = 0, yy = 0, zz = 0, yz = 0;
	point_t proj_dir, delta_t;

	xx   = tmp_id1_*locations_.size() + tmp_id2_;

	proj = dotProduct(point2vector(minusPoint(pos_,
								   locations_[tmp_id1_])),
					  point2vector(sector_para_.dir[xx]));

	proj_dir    = sector_para_.dir[xx]; // along the direction vector
	proj_dir.x *= proj;
	proj_dir.y *= proj;
	proj_dir.z *= proj;
	delta_t     = minusPoint(minusPoint(pos_,
							 locations_[tmp_id1_]),proj_dir);

	angle_tmp = atan2(l2Norm(crossProduct(point2vector(delta_t),
										  point2vector(sector_para_.dir_n[xx]))),
					  dotProduct(point2vector(delta_t),
								 point2vector(sector_para_.dir_n[xx])));

	if (!checkDirection(crossProduct(point2vector(delta_t),
									 point2vector(sector_para_.dir_n[xx])),
					    point2vector(minusPoint(locations_[tmp_id2_],
					    						locations_[tmp_id1_]))))
	{angle_tmp *= -1;}

	angle_tmp = fmod((2*M_PI + angle_tmp),(2*M_PI));

	// the value of yy can be > or <  "num_location_intervals"
	// > object moved further away from the location area
	// < object is moving initially away from the intended goal location
	yy = ceil(proj*sector_para_.loc_int/sector_para_.dist[xx]) - 1;
	zz = ceil(angle_tmp*(sector_para_.sec_int/2)/M_PI) - 1;

	yz = yy*sector_para_.sec_int + zz;

	//int tmp = sector_[xx].size()-1;

	if (yy < sector_para_.loc_int &&
	    zz < sector_para_.sec_int &&
	    yy >= 0 &&
	    zz >= 0)
	{
		sector_[yz].max =
				max(l2Norm(delta_t), sector_[yz].max);
		sector_[yz].min =
				min(l2Norm(delta_t), sector_[yz].min);
		double mid = (sector_[yz].max + sector_[yz].min)/2;
		double tmp_ratio1 = (sector_[yz].max - mid) /
				             kernel_[(kernel_.size()-1)/2]
				                    [(kernel_[0].size()-1)/2];

		for(int i=0;i<kernel_.size();i++)
		{
			for(int ii=0;ii<kernel_[0].size();ii++)
			{
				// extra calculation due to roundness of sector
				// ignoring kernel elements that go out of location area range

				int tmpl = ii - (kernel_[0].size()/2) + yy;

				int tmps = ((i - (kernel_.size()/2) + zz) +
							sector_para_.sec_int) % sector_para_.sec_int;

				int tmp3 = tmpl*sector_para_.sec_int + tmps;

				if (tmpl < 0 || tmpl >= sector_para_.loc_int)
					continue;

				sector_[tmp3].max =
						max((kernel_[i][ii]*tmp_ratio1) + mid ,
							sector_[tmp3].max);
				sector_[tmp3].min =
						min(mid-(kernel_[i][ii]*tmp_ratio1) ,
							sector_[tmp3].min);
			}
		}
	}
}

void checkSectorConstraint(
	Graph &Graph_,
	double max_range_,
	int kernel_size_x_,
	int kernel_size_y_)
{
	vector<vector<edge_tt> > sector = Graph_.getEdgeList();
	sector_para_t sector_para = Graph_.getSectorPara();

	int num_locations = Graph_.getNodeList().size();

	for(int i=0;i<Sqr(num_locations);i++)
	{
		for(int ii=0;ii<sector[i].size();ii++)
		{
			vector<sector_t> sector_tmp = sector[i][ii].sector_map;
			vector<double> sector_const = sector[i][ii].sector_const;
			for(int iii=0;iii<sector_para.loc_int*sector_para.sec_int;iii++)
			{
				double range = sector_tmp[iii].max - sector_tmp[iii].min;
				sector_const[iii] =
						(range > 0) && (range - max_range_ > 0) ? range : 0;

				int loc_tmp = iii / sector_para.sec_int;
				int sec_tmp = iii % sector_para.sec_int;

				for(int l=0;l<kernel_size_x_;l++)
				{
					for(int s=0;s<kernel_size_y_;s++)
					{
						int tmpl =   l - (kernel_size_x_/2) + loc_tmp;
						int tmps = ((s - (kernel_size_y_/2) + sec_tmp) +
									sector_para.sec_int) % sector_para.sec_int;

						int tmp1 = tmpl*sector_para.sec_int + tmps;

						if (tmpl < 0 || tmpl >= sector_para.loc_int)
							continue;

						if (sector_tmp[tmp1].max <= 0)
							sector_const[iii] = 0;
					}
				}
			}
			Graph_.updateEdgeConst(sector_const, i/num_locations, i%num_locations,ii);
		}
	}



//
//						if ((sector[i][ii][(iii+(int)(5/2)+num_sector_intervals)%num_sector_intervals].max <= 0) ||
//							(sector[i][ii][(iii-(int)(5/2)+num_sector_intervals)%num_sector_intervals].max <= 0))
//							sector_constraint[i][ii][iii] = 0;

}

bool checkDirection(
	vector<double> A,
	vector<double> B)
{
	bool out = true;
	for(int i=0;i<A.size();i++)
	{
		if (((A[i] >= 0) && (B[i] < 0)) || ((A[i] < 0) && (B[i] >= 0)))
		{
			out = false;
			break;
		}
	}
	return out;
}

// ============================================================================
// Labels
// ============================================================================

void labelMovement(
	Graph Graph_)
{
	vector<string> label; label.clear();

	vector<vector<string> > data;
	string path;
	path =  "./Scene/" + Graph_.getScene()  + "/"
			           + Graph_.getObject() + "/mov_data.txt";
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		label.push_back("MOVE");
		label.push_back("SLIDE");
		writeMovLabelFile(path, label);
	}
	else
	{
		for(int ii=0;ii<data[0].size();ii++)
			label.push_back(data[0][ii]);
	}
	printf("Reviewing movement labels for OBJ...\n");
	for(int i=0;i<label.size();i++)
		printf("MLabel %d : %s\n", i, label[i].c_str());

	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<label.size();i++)
			{
				printf("Enter new label for MLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					label[i] = mystr;
				}
			}
			remove(path.c_str());
			writeMovLabelFile(path,label);
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}

	Graph_.updateMovLabel(label);
}

void labelLocation(
	string path_,
	vector<point_t> &points_,
	vector<point_t> &locations_,
	vector<double>  &location_boundary_,
	vector<string>  &label_,
	vector<int>     &surface_num_,
	double epsilon_,
	int    minpts_)
{
	int num_points = points_.size();
	int num_locations;

	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	point_t *points_array = Calloc(point_t,points_.size());
	vector2array(points_, points_array);

	vector<vector<string> > data;
	readFile(path_.c_str(), data , ',');

	if(data.empty())
	{
		locations_.clear();
		location_boundary_.clear();
		//[CLUSTERING]*********************************************************
		dbscanCluster(epsilon_, (unsigned int)minpts_, num_points, points_array);
		printf("Clustering training data......Complete\n");
		reshapeVector(points_, num_points);
		array2vector(points_array, num_points, points_);
		combineNearCluster(points_, locations_);
		printf("Combining nearby clusters......Complete\n");
		num_locations = locations_.size();
		reshapeVector(location_boundary_, num_locations);
		contactBoundary(points_, locations_, location_boundary_, true);
		contactBoundary(points_, locations_, location_boundary_, false);
		//*********************************************************[CLUSTERING]
		reshapeVector(label_, num_locations + 1);
		label_[0] = {"CONNECTION"};
//		showData(points_, label_, color_code, true, true);
	}
	else
	{
		int data_tmp  = 6;
		num_locations = data[0].size()/data_tmp;
		reshapeVector(locations_,		  num_locations);
		reshapeVector(location_boundary_, num_locations);
		reshapeVector(label_,		  	  num_locations+1);
		reshapeVector(surface_num_,	  	  num_locations);
		label_[0]  = {"CONNECTION"};
		for(int ii=0;ii<num_locations;ii++)
		{
			label_[ii+1] 		      =      data[0][ii*data_tmp+0];
			locations_[ii].x 		  = atof(data[0][ii*data_tmp+1].c_str());
			locations_[ii].y 		  = atof(data[0][ii*data_tmp+2].c_str());
			locations_[ii].z 		  = atof(data[0][ii*data_tmp+3].c_str());
			locations_[ii].cluster_id = UNCLASSIFIED;
			location_boundary_[ii] 	  = atof(data[0][ii*data_tmp+4].c_str());
			surface_num_[ii] 	      = atof(data[0][ii*data_tmp+5].c_str());
		}
		contactBoundary(points_, locations_, location_boundary_, false);
//		showData(points_, label_, color_code, true, false);
	}

	printf("Reviewing location labels...\n");
	for(int i=0;i<num_locations;i++)
		printf("LLabel %d : %s\n", i+1, label_[i+1].c_str());

	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<num_locations;i++)
			{
				printf("Enter new label for LLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					label_[i+1] = mystr;
				}
			}
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
}

void labelLocation_(
	Graph &Graph_,
	vector<vector<point_t> > &pos_vel_acc_,
	double epsilon_,
	int minpts_)
{
	vector<string>  label;
	vector<point_t> locations;
	vector<double> 	location_boundary;
	vector<int> 	surface_num1;
	label.clear();
	locations.clear();
	location_boundary.clear();
	surface_num1.clear();

	vector<vector<double> > surface = Graph_.getSurface();

	int num_points = pos_vel_acc_.size();
	int num_surfaces = surface.size();

	// [LABELLING LOCATION]****************************************************
	vector<point_t> points_avg;
	for(int i=0;i<pos_vel_acc_.size();i++)
		points_avg.push_back(pos_vel_acc_[i][0]);

	string path;
	path =  "./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data.txt";

	labelLocation(path, points_avg,
				  locations, location_boundary, label, surface_num1,
				  epsilon_, minpts_);

	int num_locations = locations.size();

	for(int i=0;i<pos_vel_acc_.size();i++)
		pos_vel_acc_[i][0].cluster_id = points_avg[i].cluster_id;
	printf("Labeling of clusters (action locations)......Complete\n");
	// ****************************************************[LABELLING LOCATION]

	// [NODES]*****************************************************************
	vector<vector<data_t> > node_data;
	vector<vector<double> > node_data_tmp;
	reshapeVector(node_data,num_locations);

	data_t motion_data;

	for(int i=0;i<num_points;i++)
	{
		if (pos_vel_acc_[i][0].cluster_id >= 0)
		{
			motion_data.pos = pos_vel_acc_[i][0];
			motion_data.vel = pos_vel_acc_[i][1];
			motion_data.acc = pos_vel_acc_[i][2];
			node_data[pos_vel_acc_[i][0].cluster_id].push_back(motion_data);
		}
	}

	if(surface_num1.empty())
	{
		for(int i=0;i<num_locations;i++)
		{
			vector<double> tmp_vec(3);
			int surface_num  =  -1;
			double dist_tmp  = 0.0;
			double dist_tmp2 = 0.0; //### need improvement
			double tmp_spd 	 = 0.0;
			double tmp_dir 	 = 0.0;

			for(int ii=0;ii<node_data[i].size();ii++)
			{
				for(int iii=0;iii<num_surfaces;iii++)
				{
					dist_tmp = surfaceDistance(node_data[i][ii].pos,surface[iii]);
					if (dist_tmp < 0.1 && min_(dist_tmp,dist_tmp2))
						surface_num = iii;
					dist_tmp2 = dist_tmp;

					tmp_vec[0] = surface[iii][0];
					tmp_vec[1] = surface[iii][1];
					tmp_vec[2] = surface[iii][2];
				}
				tmp_spd += l2Norm(node_data[i][ii].vel);
				tmp_dir += l2Norm(crossProduct(tmp_vec,point2vector(node_data[i][ii].vel)))/
						   (l2Norm(tmp_vec) * l2Norm(point2vector(node_data[i][ii].vel)));
			}

			if (Graph_.checkNode(i))
				Graph_.extendNode(i, node_data[i]);
			else
				Graph_.addNode(label[i], i, -1,
							   locations[i], location_boundary[i],
							   surface_num, node_data[i]);

			surface_num1.push_back(surface_num);

	//		if (surface_num >= 0 &&
	//			tmp_spd/node_data[i].size() > vel_limit &&
	//			tmp_dir/node_data[i].size() > 0.96) // sind(75)
	//		{
	//			if (Graph[0].checkNode(i))
	//				Graph[0].extendNode(node_data[i],i);
	//			else
	//				Graph[0].addNode(LABEL_LOC[i],1,surface_num,location_boundary[i],node_data[i]);
	//		}
	//		else
	//		{
	//			if (Graph[0].checkNode(i))
	//				Graph[0].extendNode(node_data[i],i);
	//			else
	//				Graph[0].addNode(LABEL_LOC[i],0,surface_num,location_boundary[i],node_data[i]);
	//		}
		}
	}
	else
	{
		for(int i=0;i<num_locations;i++)
		{
			if (Graph_.checkNode(i))
				Graph_.extendNode(i, node_data[i]);
			else
				Graph_.addNode(label[i], i, -1,
							   locations[i], location_boundary[i],
							   surface_num1[i], node_data[i]);
		}
	}
	// *****************************************************************[NODES]

	remove(path.c_str());
	writeLocLabelFile(path, label, locations, location_boundary, surface_num1);

}

void labelSector(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	double max_range_,
	int kernel_size_x_,
	int kernel_size_y_,
	vector<int> file_eof_,
	vector<unsigned char*> color_code_)
{
	// THE use of gaussian kernel helps to smoothen can create a tube like structure.
	// However it is still possible to have like bumps because the sampling is just not enough.
	vector<vector<double> > kernel(kernel_size_x_);
	for(int i=0;i<kernel_size_x_;i++) kernel[i].resize(kernel_size_y_);
	gaussKernel(kernel, kernel_size_x_, kernel_size_y_, 1.0);

	// Graph_.getEdgeList() = [#loc*#loc -> #edges -> #loc*#sec]

	generateSectorCurve(Graph_, pos_vel_acc_avg_, file_eof_, kernel);
	printf("Generating sectors......Complete\n");

//	checkSectorConstraint(Graph_, max_range_, kernel_size_x_, kernel_size_y_);
//	printf("Checking sector constraints......Complete\n");
//
	vector<point_t> point_zero; vector<string> label_zero; bool flag = false;
	for(int i=0;i<pos_vel_acc_avg_.size();i++)
	{
//		if (pos_vel_acc_avg_[i][0].cluster_id==0) flag = true;
//		if (pos_vel_acc_avg_[i][0].cluster_id>0) flag = false;
		point_zero.push_back(pos_vel_acc_avg_[i][0]);
	}
	showConnection(point_zero, label_zero, Graph_, color_code_, true);
	printf("Viewing sector......Complete\n");
//
//	string path;
//	path = 	"./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_max.txt";
//	writeSectorFile(Graph_, path, 0);
//
//	path = 	"./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_min.txt";
//	writeSectorFile(Graph_, path, 1);
//
//	path = 	"./Scene/" + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_const.txt";
//	writeSectorFile(Graph_, path, 2);
}

// ============================================================================
// Prediction
// ============================================================================

void triggerContact(
	point_t &p_,
	Graph Graph_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();

	vector<point_t> locations(num_locations);
	vector<double>  location_boundary(num_locations);

	for(int i=0;i<num_locations;i++)
	{
		locations[i]         = nodes[i].location;
		location_boundary[i] = nodes[i].boundary;
	}

	decideBoundary(p_, locations, location_boundary);
}

void checkMotion(
	point_t pos_,
	point_t vel_,
	vector<string> label_,
	vector<vector<double> > surface_,
	double surface_limit_,
	double angle_limit_,
	double vel_limit_,
	label_t &LABEL_)
{
	bool slide = false;
	int surface_num_tmp = 0;

	if (l2Norm(vel_) < vel_limit_)
	{
		LABEL_.mov = -1;
	}
	else
	{
		for(int ii=0;ii<surface_.size();ii++)
		{
			if (!slide)
			{
				slide =	checkMoveSlide(pos_, vel_, surface_[ii],
									   surface_limit_, angle_limit_); //####need to add
				surface_num_tmp = ii;
			}
		}

		if (slide)
		{
			LABEL_.mov = 1;
			LABEL_.surface[surface_num_tmp] = 1;
		}
		else
		{
			LABEL_.mov = 0;
		}
	}
}

void checkSector(
	vector<int> &prediction_,
	vector<double> &t_val_,
	point_t pos_,
	Graph &Graph_,
	Graph Graph_mem_,
	int tmp_id_,
	bool learn_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	double angle_tmp = 0.0, proj = 0.0;
	int xx, yy, zz, yz;
	xx = yy = zz = yz = 0;
	point_t tmp_diff, proj_dir, delta_t;

	for(int i=0;i<num_locations;i++)
	{
		prediction_[i] = EXCEED_RANGE;
		t_val_[i]	   = 0;

		if(tmp_id_ == i) continue;

		xx       = tmp_id_*num_locations+i;

		tmp_diff = minusPoint(pos_, locations[tmp_id_]);
		proj     = dotProduct(point2vector(tmp_diff),
				              point2vector(Graph_.getSectorPara().dir[xx]));
		proj_dir    = Graph_.getSectorPara().dir[xx]; // along the direction vector
		proj_dir.x *= proj;
		proj_dir.y *= proj;
		proj_dir.z *= proj;
		delta_t     = minusPoint(tmp_diff,proj_dir);

		angle_tmp = atan2(l2Norm(crossProduct(point2vector(delta_t),
											  point2vector(Graph_.getSectorPara().dir_n[xx]))),
						  dotProduct(point2vector(delta_t),
									 point2vector(Graph_.getSectorPara().dir_n[xx])));
		angle_tmp = fmod((2*M_PI + angle_tmp),(2*M_PI));

		yy = ceil(proj*Graph_.getSectorPara().loc_int/Graph_.getSectorPara().dist[xx]) - 1;
		zz = ceil(angle_tmp*(Graph_.getSectorPara().sec_int/2)/M_PI) - 1;
		yz = yy*Graph_.getSectorPara().sec_int + zz;

		vector<edge_tt> tmp_edge  = Graph_mem_.getEdgeList()[xx];
		vector<edge_tt> tmp_edge2 = Graph_.getEdgeList()[xx];

		for(int ii=0;ii<tmp_edge.size();ii++)
		{
			if(yy<Graph_.getSectorPara().loc_int && yy>=0 &&
			   zz<Graph_.getSectorPara().sec_int && zz>=0)
			{
				if (l2Norm(delta_t) <= tmp_edge[ii].sector_map[yz].max &&
					l2Norm(delta_t) >= tmp_edge[ii].sector_map[yz].min)
				{
					prediction_[i] = WITHIN_RANGE;
					t_val_[i]      = l2Norm(delta_t);
				}
				else
				{
					if(learn_ && i==3)
					{
						prediction_[i] = OUT_OF_RANGE;
						if(l2Norm(delta_t) - tmp_edge[ii].sector_map[yz].max > 0)
							t_val_[i] = l2Norm(delta_t) - tmp_edge[ii].sector_map[yz].max;
						else
							t_val_[i] = tmp_edge[ii].sector_map[yz].min - l2Norm(delta_t);
						tmp_edge2[ii].sector_map[yz].max = max(l2Norm(delta_t),tmp_edge2[ii].sector_map[yz].max);
						tmp_edge2[ii].sector_map[yz].min = min(l2Norm(delta_t),tmp_edge2[ii].sector_map[yz].min);
					}
					else
					{
						prediction_[i] = OUT_OF_RANGE;
						if(l2Norm(delta_t) - tmp_edge[ii].sector_map[yz].max > 0)
							t_val_[i] = l2Norm(delta_t) - tmp_edge[ii].sector_map[yz].max;
						else
							t_val_[i] = tmp_edge[ii].sector_map[yz].min - l2Norm(delta_t);
					}
				}
			}
			Graph_.updateEdgeSector(tmp_edge2[ii].sector_map, tmp_id_, i, ii);
		}
	}
}

void motionPrediction(
	vector<int> &prediction_,
	vector<double> &t_val_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	vector<double> &predict_in_,
	vector<double> &predict_err_,
	vector<double> &predict_in_last_,
	double &pow_dec_,
	Graph Graph_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();

	reshapeVector(predict_in_,num_locations);
	for(int ii=0;ii<num_locations;ii++)
	{
		if ((int)prediction_[ii] == WITHIN_RANGE)
		{
			flag_predict_      = false;
			flag_predict_last_ = true;
			predict_in_[ii]	   = 1.0;
			predict_err_[ii]   = 0.0;
		}
		else if ((int)prediction_[ii] == OUT_OF_RANGE)
		{
			predict_in_[ii]  = 0.0;
			predict_err_[ii] = pdfExp(0.03, 0.0, t_val_[ii]); // NEED TO CHANGE ######
		}
		else
		{
			predict_in_[ii]  = 0.0;
			predict_err_[ii] = 0.0;
		}
	}

	normalizeData(predict_in_);
	normalizeData(predict_err_);

	if (flag_predict_)
	{
		for(int ii=0;ii<num_locations;ii++)
		{
			if (predict_in_last_[ii] > 0 && prediction_[ii] > WITHIN_RANGE)
				predict_err_[ii] = predict_err_[ii] * (1-pow(0.5,pow_dec_)) + pow(0.5,pow_dec_);
			else if (predict_err_[ii] != 1.0) // prevent only 1 valid location prediction case
				predict_err_[ii] = predict_err_[ii] * (1-pow(0.5,pow_dec_));
		}
		pow_dec_ += 0.5;
	}
	else
	{
		for(int ii=0;ii<num_locations;ii++)
			predict_in_last_[ii] = predict_in_[ii];
		pow_dec_ = 1;
	}

	flag_predict_ = flag_predict_last_;

}

void locationPrediction(
	int location_num_,
	point_t pos_,
	point_t vel_,
	Graph Graph_,
	double surface_limit_,
	double angle_limit_,
	double vel_limit_,
	label_t &LABEL_)
{
	// check if label is empty
	if (!strcmp(Graph_.getNode(location_num_).name.c_str(),""))
	{
		LABEL_.loc[location_num_] = -1;
		checkMotion(pos_, vel_, Graph_.getMovLabel(), Graph_.getSurface(),
				    surface_limit_, angle_limit_, vel_limit_, LABEL_);
	}
	else
		LABEL_.loc[location_num_] = 1;
}


















//---------------------------------------------------------------------------------------------------------------------
double surfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_)
{
	vector<double> surface_tmp(3);
	vector<double> surface_parallel_tmp;
	vector<double> p_tmp;
	double norm_tmp = 0.0;

	surface_tmp[0] = surface_[0];
	surface_tmp[1] = surface_[1];
	surface_tmp[2] = surface_[2];

	p_tmp = point2vector(minusPoint(pos_, pos_surface_));

	// surface_parallel_tmp can be obtained at the beginning by just taking the normalized vector between 2 points on the surface
	{
		surface_parallel_tmp = crossProduct(surface_tmp, p_tmp);

		for(int i=0;i<3;i++)
			norm_tmp += Sqr(surface_parallel_tmp[i]);
		norm_tmp = sqrt(norm_tmp);

		for(int i=0;i<3;i++)
			surface_parallel_tmp[i] /= norm_tmp;
	}

	return dotProduct(p_tmp, surface_parallel_tmp);
}

double surfaceDistance(
	point_t pos_,
	vector<double> surface_)
{
	return surface_[0]*pos_.x +
		   surface_[1]*pos_.y +
		   surface_[2]*pos_.z -
		   surface_[3];
}

double surfaceAngle(
	point_t vel_,
	vector<double> surface_)
{
	vector<double> tmp(3);
	tmp[0] = surface_[0];
	tmp[1] = surface_[1];
	tmp[2] = surface_[2];
	return 	l2Norm(crossProduct(tmp,point2vector(vel_)))/
			(l2Norm(tmp) * l2Norm(point2vector(vel_)));
}


//---------------------------------------------------------------------------------------------------------------------
// General Functions

void writePointFile(
	point_t *p,
	unsigned int num_points)
{
	remove("data.txt");
	for(unsigned int i=0;i<num_points;i++)
	{
		// write values into data.txt
		std::ofstream write_file("data.txt", std::ios::app);
		write_file << p[i].x << ","
				   << p[i].y << ","
				   << p[i].z << ","
				   << p[i].cluster_id
				   << "\n";
	}
}




bool checkSurfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_,
	double surface_limit_,
	double surface_range_limit_)
{
	if(surfaceDistance(pos_, surface_) < surface_limit_) // less than 10 cm off the surface
		if(surfaceRange(pos_, pos_surface_, surface_) < surface_range_limit_)
			return true;
		else
			return false;
	else
		return false;
}

bool checkMoveSlide(
	point_t pos_,
	point_t vel_,
	vector<double> surface_,
	double surface_limit_,
	double angle_limit_)
{
	if(surfaceDistance(pos_, surface_) < surface_limit_) // less than 10 cm off the surface
		if(surfaceAngle(vel_, surface_) > angle_limit_)
			return true;
		else
			return false;
	else
		return false;
}



double checkMoveSlideOutside(
		point_t pos_,
		point_t vel_,
		double **surface_,
		unsigned int num_surfaces_)
{
	vector<double> A(3);
	double dir_tmp = INFINITY, dist_tmp = INFINITY;
	for(int i=0;i<num_surfaces_;i++)
	{
		A[0] = surface_[i][0];
		A[1] = surface_[i][1];
		A[2] = surface_[i][2];
		dir_tmp  = l2Norm(crossProduct(A,point2vector(vel_)));
		dist_tmp = surface_[i][0]*pos_.x +
						  surface_[i][1]*pos_.y +
						  surface_[i][2]*pos_.z -
						  surface_[i][3];
//		cout << dir_tmp << " " << dist_tmp << " ";
	}
	return 0.0;
}

















