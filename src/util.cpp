/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#include "util.h"
#include "dbscan.h"

#include "svm.h"
#include "svm-train.c"
#include "svm-predict.c"

using namespace std;

#define Sqr(x) ((x)*(x))
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define Calloc(type,n) (type *)calloc( n, sizeof(type))

//---------------------------------------------------------------------------------------------------------------------
// Basic algo
double l2Norm(vector<double> A)
{
    double a=0.0;
    for (unsigned int i=0;i<A.size();i++)
        a+=Sqr(A[i]);
    return sqrt(a);
}

double l2Norm(point_t A)
{
    return sqrt(Sqr(A.x)+Sqr(A.y)+Sqr(A.z));
}

double normalPdf(double var, double mu, double x)
{
	return (1/sqrt(2*var*M_PI)) * exp( - Sqr(x-mu)/(2*var) );
}

double pdfExp(double var, double mu, double x)
{
	return exp( - Sqr(x-mu)/(2*var) );
}

vector<double> addVector(vector<double> A, vector<double> B)
{
	vector<double> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]+B[i]);
	return C;
}

vector<double> minusVector(vector<double> A, vector<double> B)
{
	vector<double> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]-B[i]);
	return C;
}

point_t addPoint(point_t A, point_t B)
{
	point_t C;
	C.x = A.x + B.x;
	C.y = A.y + B.y;
	C.z = A.z + B.z;
	C.cluster_id = UNCLASSIFIED;
	return C;
}

point_t minusPoint(point_t A, point_t B)
{
	point_t C;
	C.x = A.x - B.x;
	C.y = A.y - B.y;
	C.z = A.z - B.z;
	C.cluster_id = UNCLASSIFIED;
	return C;
}

vector<double> crossProduct(vector<double> A, vector<double> B)
{
	vector<double> C(3);
	C[0] = A[1]*B[2] - A[2]*B[1];
	C[1] = A[2]*B[0] - A[0]*B[2];
	C[2] = A[0]*B[1] - A[1]*B[0];
	if(C[0]*C[0]+C[1]*C[1]+C[2]*C[2] == 0){ // prevent degenerate case
		printf("[WARNING] : CROSS PRODUCT VECTORS ARE COLLINEAR !!!\n");
		C[0]=0; C[1]=0; C[2]=0;
	}
	if(A[0] == 0 && A[1] == 0 && A[2] == 0)
		printf("[WARNING] : CROSS PRODUCT VECTOR 1 IS A ZERO VECTOR !!!\n");
	if(B[0] == 0 && B[1] == 0 && B[2] == 0)
		printf("[WARNING] : CROSS PRODUCT VECTOR 2 IS A ZERO VECTOR !!!\n");
	return C;
}

double dotProduct(vector<double> A, vector<double> B)
{
	double ans;
	vector<double> C(3);
	C[0] = A[0]*B[0];
	C[1] = A[1]*B[1];
	C[2] = A[2]*B[2];
	ans = C[0]+C[1]+C[2];
	if(A[0] == 0 && A[1] == 0 && A[2] == 0)
		printf("[WARNING] : DOT PRODUCT VECTOR 1 IS A ZERO VECTOR !!!\n");
	if(B[0] == 0 && B[1] == 0 && B[2] == 0)
		printf("[WARNING] : DOT PRODUCT VECTOR 2 IS A ZERO VECTOR !!!\n");
	return ans;
}

void normalization(vector<double> &data_)
{
	double tmp = 0.0;
	for(int i=0;i<data_.size();i++)
		tmp += data_[i];
	for(int i=0;i<data_.size();i++)
		data_[i]/=tmp;

}

double movingAverage(
	double a,
	vector<double> &A,
	unsigned int win)
{
	double average = 0.0;
	for(unsigned int i=0;i<win-1;i++)
		A[i] = A[i+1];
	A[win-1] = a;
	for (unsigned int i=0;i<win;i++)
		average += A[i];
	average = average / win;
	return average;
}

void averagePoint(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg,
	unsigned int window)
{
	Xavg.x =  movingAverage( X.x, X_tmp[0], window );
	Xavg.y =  movingAverage( X.y, X_tmp[1], window );
	Xavg.z =  movingAverage( X.z, X_tmp[2], window );
}

void averagePointIncrement(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg,
	unsigned int window)
{
	if(X_tmp[0].empty())
	{
		X_tmp[0].push_back(X.x);
		X_tmp[1].push_back(X.y);
		X_tmp[2].push_back(X.z);
		Xavg.x =  X.x;
		Xavg.y =  X.y;
		Xavg.z =  X.z;
	}
	else
	{
		X_tmp[0].push_back(X.x);
		X_tmp[1].push_back(X.y);
		X_tmp[2].push_back(X.z);
		Xavg.x =  movingAverage( X.x, X_tmp[0], window );
		Xavg.y =  movingAverage( X.y, X_tmp[1], window );
		Xavg.z =  movingAverage( X.z, X_tmp[2], window );
	}
}

double surfaceRange(
	point_t pos_,
	point_t pos_surface_,
	double *surface_)
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
	double *surface_)
{
	return surface_[0]*pos_.x +
		   surface_[1]*pos_.y +
		   surface_[2]*pos_.z -
		   surface_[3];
}

double surfaceAngle(
	point_t vel_,
	double *surface_)
{
	vector<double> tmp(3);
	tmp[0] = surface_[0];
	tmp[1] = surface_[1];
	tmp[2] = surface_[2];
	return 	l2Norm(crossProduct(tmp,point2vector(vel_)))/
			(l2Norm(tmp) * l2Norm(point2vector(vel_)));
}

void gaussKernel(vector<vector<double> > &kernel_, int numx_, int numy_, double var_)
{
    double sum = 0.0;
    for (int x = -(numx_/2); x <= (numx_/2); x++)
    {
        for(int y = -(numy_/2); y <= (numy_/2); y++)
        {
        	kernel_[x+(numx_/2)][y+(numy_/2)] =
        			(1/(2*M_PI*var_)) * exp(-(Sqr(sqrt(Sqr(x) + Sqr(y)))/(2*var_)));
            sum += kernel_[x+(numx_/2)][y+(numy_/2)];
        }
    }
    for(int i = 0; i < numx_; ++i)
        for(int j = 0; j < numy_; ++j)
        	kernel_[i][j] /= sum;
}

//---------------------------------------------------------------------------------------------------------------------
// General Functions

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
	vector<vector<double> > kernel_)
{
	int tmp_id, tmp_id2, file_num;
	tmp_id = tmp_id2 = file_num = 0;
	bool flag_next = true;

	for(int i=0;i<num_points;i++)
	{
		if(i==file_eof[file_num]-1)
		{
			flag_next = true;
			tmp_id  = 0;
			tmp_id2 = 0;
			continue;
		}

		if (pos_[i].cluster_id >= 0)
		{
			flag_next = true;
			tmp_id = pos_[i].cluster_id;
			continue;
		}

		if(flag_next)
		{
			flag_next = false;
			for(int ii=i;ii<num_points;ii++)
			{
				tmp_id2 = pos_[ii].cluster_id;
				if(pos_[ii].cluster_id >= 0)
					break;
			}
		}

		if(tmp_id != tmp_id2)
			updateSector(sector, pos_[i], location, num_locations,
						 tmp_dir, tmp_dir_normal, tmp_norm,
						 num_location_intervals, num_sector_intervals,
						 tmp_id, tmp_id2, kernel_);
	}
}

void prepareSector(
	point_t *tmp_dir,
	point_t *tmp_dir_normal,
	double *tmp_norm,
	point_t *location,
	unsigned int num_locations)
{
	int c = 0;
	for(int i=0;i<num_locations;i++)
	{
		for(int ii=0;ii<num_locations;ii++)
		{
			if(i == ii)
			{
				tmp_dir[c].x =
				tmp_dir[c].y =
				tmp_dir[c].z = 0.0;
				tmp_norm[c] = 0;
				tmp_dir_normal[c].x =
				tmp_dir_normal[c].y =
				tmp_dir_normal[c].z = 0.0;
				++c;
				continue;
			}

			tmp_dir[c] = minusPoint(location[ii],location[i]);
			tmp_norm[c] = l2Norm(tmp_dir[c]);
			tmp_dir[c].x /= tmp_norm[c];
			tmp_dir[c].y /= tmp_norm[c];
			tmp_dir[c].z /= tmp_norm[c];
			vector<double> B(3);
			B[0] = 0; B[1] = 1; B[2] = 0;
			tmp_dir_normal[c] = vector2point(
									crossProduct(
										point2vector(tmp_dir[c]), B));

			++c;
		}
	}
}

void checkSectorConstraint(
	sector_t ***sector,
	double ***sector_constraint,
	unsigned int num_locations,
	unsigned int num_location_intervals,
	unsigned int num_sector_intervals)
{

	for(int i=0;i<Sqr(num_locations);i++)
		for(int ii=0;ii<num_location_intervals;ii++)
			for(int iii=0;iii<num_sector_intervals;iii++)
			{
				sector_constraint[i][ii][iii] =
					(sector[i][ii][iii].max - sector[i][ii][iii].min > 0) &&
					(0.05 - (sector[i][ii][iii].max - sector[i][ii][iii].min) > 0) ?
						sector[i][ii][iii].max - sector[i][ii][iii].min : 0;
				for(int iv=0;iv<5;iv++)
					for(int v=0;v<5;v++)
						if (sector[i]
						          [(iv-(int)(5/2)+ii+num_location_intervals)%num_location_intervals]
						          [(v-(int)(5/2)+iii+num_sector_intervals)%num_sector_intervals].max <= 0)
							sector_constraint[i][ii][iii] = 0;
//
//						if ((sector[i][ii][(iii+(int)(5/2)+num_sector_intervals)%num_sector_intervals].max <= 0) ||
//							(sector[i][ii][(iii-(int)(5/2)+num_sector_intervals)%num_sector_intervals].max <= 0))
//							sector_constraint[i][ii][iii] = 0;
			}
}

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
	vector<vector<double> >kernel_)
{
	double angle_tmp = 0.0, p_ = 0.0;
	int xx, yy, zz;
	xx = yy = zz = 0;
	point_t tmp_diff, p_dir, t_;

	xx       = tmp_id*num_locations+tmp_id2;

	tmp_diff = minusPoint(pos_, location[tmp_id]);

	p_       = dotProduct(point2vector(tmp_diff), point2vector(tmp_dir[xx]));

	p_dir    = tmp_dir[xx]; // along the direction vector
	p_dir.x *= p_;
	p_dir.y *= p_;
	p_dir.z *= p_;
	t_       = minusPoint(tmp_diff,p_dir);

	angle_tmp = atan2(l2Norm(crossProduct(point2vector(t_),
											  point2vector(tmp_dir_normal[xx]))),
							   dotProduct(point2vector(t_),
										  point2vector(tmp_dir_normal[xx])));
	//angle_tmp[ii] = angle_tmp[ii] / M_PI * 180;

	// the value of yy can be > or <  "num_location_intervals"
	// > object moved further away from the location area
	// < object is moving initially away from the intended goal location
	yy 		 = ceil(p_*num_location_intervals/tmp_norm[xx])  -1;
	zz 		 = ceil(angle_tmp*num_sector_intervals/M_PI) -1;

//	// THE use of gaussian kernel helps to smoothen can create a tube like structure.
//	// However it is still possible to have like bumps because the sampling is just not enough.
//	int kernel_size = 5;
//	double **kernel_ = Calloc(double*,kernel_size);
//	for(int i=0;i<kernel_size;i++)
//		kernel_[i] = Calloc(double,kernel_size);
//	gaussKernel(kernel_, kernel_size, kernel_size, 1.0);

	if(yy<num_location_intervals &&
	   zz<num_sector_intervals &&
	   yy>=0 &&
	   zz>=0)
	{
		sector[xx][yy][zz].max = max(l2Norm(t_),sector[xx][yy][zz].max);
		sector[xx][yy][zz].min = min(l2Norm(t_),sector[xx][yy][zz].min);
		double mid = (sector[xx][yy][zz].max + sector[xx][yy][zz].min)/2;
		double tmp_ratio1 = (sector[xx][yy][zz].max-mid) /
				            kernel_[(kernel_.size()-1)/2][(kernel_[0].size()-1)/2];

		for(int i=0;i<kernel_.size();i++)
		{
			for(int ii=0;ii<kernel_[0].size();ii++)
			{
				int tmp  = i -(kernel_.size()/2)+zz;
				int tmp2 = ii-(kernel_[0].size()/2)+yy;
				if (tmp<0)
					tmp += num_sector_intervals;
				else if (tmp>=num_sector_intervals)
					tmp %= num_sector_intervals;
				if (tmp2<0)
					continue;
				else if (tmp2>=num_location_intervals)
					continue;
				sector[xx][tmp2][tmp].max = max((kernel_[i][ii]*tmp_ratio1)+mid,sector[xx][tmp2][tmp].max);
				sector[xx][tmp2][tmp].min = min(mid-(kernel_[i][ii]*tmp_ratio1),sector[xx][tmp2][tmp].min);
			}
		}
	}
}

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
	bool learn)
{
	double angle_tmp = 0.0, p_ = 0.0;
	int xx, yy, zz;
	xx = yy = zz = 0;
	point_t tmp_diff, p_dir, t_;

	for(int i=0;i<num_locations;i++)
	{
		prediction[i] = 0;
		t_val[i]      = 0;

		if(tmp_id == i) continue;

		xx       = tmp_id*num_locations+i;

		tmp_diff = minusPoint(pos_, location[tmp_id]);
		p_       = dotProduct(point2vector(tmp_diff),
				              point2vector(tmp_dir[xx]));
		p_dir    = tmp_dir[xx]; // along the direction vector
		p_dir.x *= p_;
		p_dir.y *= p_;
		p_dir.z *= p_;
		t_       = minusPoint(tmp_diff,p_dir);

		angle_tmp = atan2(l2Norm(crossProduct(point2vector(t_),
											  point2vector(tmp_dir_normal[xx]))),
						  dotProduct(point2vector(t_),
									 point2vector(tmp_dir_normal[xx])));
		//angle_tmp[ii] = angle_tmp[ii] / M_PI * 180;

		yy 		 = ceil(p_*num_location_intervals/tmp_norm[xx])  -1;
		zz 		 = ceil(angle_tmp*num_sector_intervals/M_PI)     -1;

		if(yy<num_location_intervals && yy>=0 &&
		   zz<num_sector_intervals   && zz>=0)
		{
			if (l2Norm(t_)<=sector[xx][yy][zz].max &&
				l2Norm(t_)>=sector[xx][yy][zz].min)
			{
				prediction[i] = 1;
				t_val[i]      = l2Norm(t_);
			}
			else
			{
				if(learn)
				{
					prediction[i] = 3;
					if(l2Norm(t_) - sector[xx][yy][zz].max > 0)
						t_val[i] = l2Norm(t_) - sector[xx][yy][zz].max;
					else
						t_val[i] = sector[xx][yy][zz].min - l2Norm(t_);
					sector[xx][yy][zz].max = max(l2Norm(t_),sector[xx][yy][zz].max);
					sector[xx][yy][zz].min = min(l2Norm(t_),sector[xx][yy][zz].min);
				}
				else
				{
					prediction[i] = 2;
					t_val[i]      = l2Norm(t_);
				}
			}


//			if ( yy==0 )
//			{
//				if ((l2Norm(t_)<=sector[xx][yy][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy][zz].min)||
//					(l2Norm(t_)<=sector[xx][yy+1][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy+1][zz].min))
//				{
//					prediction[i] = 1;
//					t_val[i]      = l2Norm(t_);
//				}
//				else
//				{
//					if(learn)
//					{
//						//list[i] = max(l2Norm(t_),sector[xx][yy][zz].max) - min(l2Norm(t_),sector[xx][yy][zz].max);
//						prediction[i] = 3;
//						t_val[i]      = l2Norm(t_);
//						//list[i] = sector[xx][yy][zz].max;
//						//list[i] = -3;
//						sector[xx][yy][zz].max = max(l2Norm(t_),sector[xx][yy][zz].max);
//						sector[xx][yy][zz].min = min(l2Norm(t_),sector[xx][yy][zz].min);
//					}
//					else
//					{
//						prediction[i] = 2;
//						t_val[i]      = l2Norm(t_);
//					}
//				}
//			}
//			else if ( yy<num_location_intervals )
//			{
//				if ((l2Norm(t_)<=sector[xx][yy][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy][zz].min)||
//					(l2Norm(t_)<=sector[xx][yy-1][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy-1][zz].min))
//				{
//					prediction[i] = 1;
//					t_val[i]      = l2Norm(t_);
//				}
//				else
//				{
//					if(learn)
//					{
//						//list[i] = max(l2Norm(t_),sector[xx][yy][zz].max) - min(l2Norm(t_),sector[xx][yy][zz].max);
//						prediction[i] = 3;
//						t_val[i]      = l2Norm(t_);
//						//list[i] = sector[xx][yy][zz].max;
//						//list[i] = -3;
//						sector[xx][yy][zz].max = max(l2Norm(t_),sector[xx][yy][zz].max);
//						sector[xx][yy][zz].min = min(l2Norm(t_),sector[xx][yy][zz].min);
//					}
//					else
//					{
//						prediction[i] = 2;
//						t_val[i]      = l2Norm(t_);
//					}
//				}
//			}
//			else
//			{
//				if ((l2Norm(t_)<=sector[xx][yy][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy][zz].min)||
//					(l2Norm(t_)<=sector[xx][yy+1][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy+1][zz].min)||
//					(l2Norm(t_)<=sector[xx][yy-1][zz].max &&
//					 l2Norm(t_)>=sector[xx][yy-1][zz].min))
//				{
//					prediction[i] = 1;
//					t_val[i]      = l2Norm(t_);
//				}
//				else
//				{
//					if(learn)
//					{
//						//list[i] = max(l2Norm(t_),sector[xx][yy][zz].max) - min(l2Norm(t_),sector[xx][yy][zz].max);
//						prediction[i] = 3;
//						t_val[i]      = l2Norm(t_);
//						//list[i] = sector[xx][yy][zz].max;
//						//list[i] = -3;
//						sector[xx][yy][zz].max = max(l2Norm(t_),sector[xx][yy][zz].max);
//						sector[xx][yy][zz].min = min(l2Norm(t_),sector[xx][yy][zz].min);
//					}
//					else
//					{
//						prediction[i] = 2;
//						t_val[i]      = l2Norm(t_);
//					}
//				}
//			}

		}
	}
}

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

void writeLocLabelFile(
	vector<string> label_,
	unsigned int obj_,
	point_t *location_,
	unsigned int num_locations_)
{
	std::ofstream write_file("./Location/loc_data.txt", std::ios::app);
	write_file << obj_;
	for(unsigned int i=0;i<num_locations_;i++)
	{
		write_file << ",";
		write_file << label_[i+1]    << ","
				   << location_[i].x << ","
				   << location_[i].y << ","
				   << location_[i].z ;
	}
	write_file << "\n";
}

void writeObjLabelFile(
	vector<string> label_,
	unsigned int obj_)
{
	std::ofstream write_file("./Object/obj_mov_data.txt", std::ios::app);
	write_file << obj_;
	for(unsigned int i=0;i<label_.size();i++)
	{
		write_file << ",";
		write_file << label_[i];
	}
	write_file << "\n";
}

void readFile(
	char *name,
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

void rewriteFileObj(
	char *name_,
	int line_,
	vector<string> new_,
	char delimiter_)
{
	char name_2[] = "./Object/obj_data2.txt";
	fstream src_file( name_ );
	ofstream out_file( name_2 );
	while (src_file)
	{
		string file_line;
		if (!getline( src_file, file_line )) break;
		istringstream lines_( file_line );
		vector <string> data_line_;
		int w = 0;
		int tmp = file_line[0]-'0';
		while (lines_)
		{
			string word;
			if (!getline( lines_, word, delimiter_)) break;
			if (w == 0)
				if(tmp == line_)
					out_file << line_;
				else
					out_file << word;
			else
			{
				out_file << ",";
				if(tmp == line_)
					out_file << new_[w-1];
				else
					out_file << word;
			}
			w++;
		}
		out_file << "\n";
	}

	if (!src_file.eof())
		cerr << "FILE ERROR!!!\n";

	remove(name_);
	rename(name_2, name_);
}

void rewriteFileLoc(
	char *name_,
	point_t *location_,
	unsigned int num_locations,
	int line_,
	vector<string> new_,
	char delimiter_)
{
	char name_2[] = "./Location/loc_data2.txt";
	fstream src_file( name_ );
	ofstream out_file( name_2 );
	while (src_file)
	{
		string file_line;
		if (!getline( src_file, file_line )) break;
		istringstream lines_( file_line );
		vector <string> data_line_;
		int c = 1;
		int j = 0;
		int k = 0;
		int tmp = file_line[0]-'0';
		while (lines_)
		{
			string word;
			if (!getline( lines_, word, delimiter_)) break;
			if (j == 0)
				if(tmp == line_)
					out_file << line_;
				else
					out_file << word;
			else
			{
				out_file << ",";
				if(tmp == line_ && j==c)
				{
					out_file << new_[k+1];
					k++;
					c+=4;
				}
				else
					out_file << word;
			}
			j++;
		}
		out_file << "\n";
	}

	if (!src_file.eof())
		cerr << "FILE ERROR!!!\n";

	remove(name_);
	rename(name_2, name_);
}

void parseData2Point(
	vector<vector<string> > data_full,
	point_t *p,
	unsigned int num_points,
	bool SVM)
{
	// SVM file
	if(SVM)
	{
		for(unsigned int i=0;i<num_points;i++)
			p[i].cluster_id = atof(data_full[i][0].c_str())-1;
	}
	// 1:frame number, 2-5:table normal vector, 6-11:2 locations, 12-14:face, 15:contact, 16-18:object3D
	else
	{
		for(unsigned int i=0;i<num_points;i++)
		{
			//if(atof(data_full[i][14].c_str())>0)
			{
				p[i].x = atof(data_full[i][15].c_str());
				p[i].y = atof(data_full[i][16].c_str());
				p[i].z = atof(data_full[i][17].c_str());
				p[i].cluster_id = UNCLASSIFIED;
			}
		}
	}
}

void preprocessData(
	point_t *p1,
	point_t **pos_vel_acc,
	unsigned int num_points,
	unsigned int *file_eof,
	unsigned int window)
{
	int data_boundary = 0, cluster_ = 0, tmp_id = 0, file_num = 0;
	point_t pos, vel, acc;
	point_t pos_avg, pos_avg_, vel_avg, vel_avg_, acc_avg;
	vector<vector<double> > pos_tmp(3), vel_tmp(3), acc_tmp(3);

	int c1 = 0, c2 = -1, c3 = -2;
	for(unsigned int i=0;i<num_points;i++)
	{
		if(i==file_eof[file_num])
		{
			pos_tmp.clear();
			vel_tmp.clear();
			acc_tmp.clear();
			pos_tmp.resize(3);
			vel_tmp.resize(3);
			acc_tmp.resize(3);
			data_boundary = 0;
			c1 = 0, c2 = -1, c3 = -2;
			file_num++;
		}

		if(data_boundary != window+2)
		{
			data_boundary+=1;

			if(c1<window)
			{
				averagePointIncrement( p1[i], pos_tmp, pos_avg, c1 );
				memcpy(&pos_vel_acc[0][i], &pos_avg, sizeof(pos_avg));
			}
			else
			{
				averagePoint( p1[i], pos_tmp, pos_avg, window );
				memcpy(&pos_vel_acc[0][i], &pos_avg, sizeof(pos_avg));
			}

			if(c2<0)
			{
				pos_vel_acc[1][i].x
				= pos_vel_acc[1][i].y
				= pos_vel_acc[1][i].z
				= 0;
			}
			else if (c2<window)
			{
				vel = minusPoint( pos_avg, pos_avg_ );
				averagePointIncrement( vel, vel_tmp, vel_avg, c2 );
				memcpy(&pos_vel_acc[1][i], &vel_avg, sizeof(vel_avg));
			}
			else
			{
				vel = minusPoint( pos_avg, pos_avg_ );
				averagePoint( vel, vel_tmp, vel_avg, window );
				memcpy(&pos_vel_acc[1][i], &vel_avg, sizeof(vel_avg));
			}

			if(c3<0)
			{
				pos_vel_acc[2][i].x
				= pos_vel_acc[2][i].y
				= pos_vel_acc[2][i].z
				= 0;
			}
			else if(c3<window)
			{
				acc = minusPoint( pos_avg, pos_avg_ );
				averagePointIncrement( acc, acc_tmp, acc_avg, c3 );
				memcpy(&pos_vel_acc[2][i], &acc_avg, sizeof(acc_avg));
			}
			else
			{
				acc = minusPoint( pos_avg, pos_avg_ );
				averagePoint( acc, acc_tmp, acc_avg, window );
				memcpy(&pos_vel_acc[2][i], &acc_avg, sizeof(acc_avg));
			}

			c1+=1; c2+=1; c3+=1;
		}
		else
		{
			pos = p1[i];
			averagePoint( pos, pos_tmp, pos_avg, window );
			vel = minusPoint( pos_avg, pos_avg_ );
			averagePoint( vel, vel_tmp, vel_avg, window );
			acc = minusPoint( pos_avg, pos_avg_ );
			averagePoint( acc, acc_tmp, acc_avg, window );
			memcpy(&pos_vel_acc[0][i], &pos_avg, sizeof(pos_avg));
			memcpy(&pos_vel_acc[1][i], &vel_avg, sizeof(vel_avg));
			memcpy(&pos_vel_acc[2][i], &acc_avg, sizeof(acc_avg));
		}

		pos_vel_acc[0][i].cluster_id = p1[i].cluster_id;
		pos_vel_acc[1][i].cluster_id = p1[i].cluster_id;
		pos_vel_acc[2][i].cluster_id = p1[i].cluster_id;
		cluster_ = p1[i].cluster_id;
		pos_avg_ = pos_avg;
		vel_avg_ = vel_avg;
	}
}

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

point_t* combineNearCluster(
	point_t *p,
	unsigned int num_points,
	unsigned int &num_locations)
{
	for(unsigned int i=0;i<num_points;i++)
		num_locations = max(p[i].cluster_id,(int&)num_locations);
	num_locations += 1;

	// calculating the centroid of cluster
	point_t *p_tmp1 = Calloc(point_t, num_locations);
	point_t *tmp    = Calloc(point_t, num_locations);
	bool limit = false;

	for(unsigned int i=0;i<num_points;i++)
	{
		if(p[i].cluster_id >= 0)
		{
			p_tmp1[p[i].cluster_id].cluster_id += 1;
			tmp[p[i].cluster_id].x += p[i].x;
			tmp[p[i].cluster_id].y += p[i].y;
			tmp[p[i].cluster_id].z += p[i].z;
		}
	}

	for(unsigned int i=0;i<num_locations;i++)
	{
		p_tmp1[i].x = tmp[i].x/p_tmp1[i].cluster_id;
		p_tmp1[i].y = tmp[i].y/p_tmp1[i].cluster_id;
		p_tmp1[i].z = tmp[i].z/p_tmp1[i].cluster_id;
		p_tmp1[i].cluster_id = UNCLASSIFIED;
	}

	// combine cluster if it is less than 0.1m
	for(unsigned int i=0;i<num_locations;i++)
	{
		for(unsigned int j=0;j<num_locations;j++)
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
					for(unsigned int ii=0;ii<num_locations;ii++)
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
	for(unsigned int i=1;i<num_locations;i++)
	{
		if(p_tmp1[i].cluster_id > p_tmp1[i-1].cluster_id &&
		   p_tmp1[i].cluster_id == (int)i)
		{
			p_tmp1[i].cluster_id = c;
			for(unsigned int ii=i+1;ii<num_locations;ii++)
				if(p_tmp1[ii].cluster_id == (int)i)
				   p_tmp1[ii].cluster_id = c;
			c++;
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].cluster_id );
	}
	//int num_locations2 = c;

	// updating cluster label
	for(unsigned int i=0;i<num_points;i++)
	{
		if (p[i].cluster_id >= 0)
			p[i].cluster_id = p_tmp1[p[i].cluster_id].cluster_id;
		//printf("Location %02d: %02d\n", line_, p[i].cluster_id );
	}

	// calculate the centroid of combined clusters
	point_t *p_center = Calloc(point_t,c);
	point_t *tmp2     = Calloc(point_t,c);

	for(int i=0;i<num_points;i++)
	{
		if(p[i].cluster_id >= 0)
		{
			p_center[p[i].cluster_id].cluster_id += 1;
			tmp2[p[i].cluster_id].x += p[i].x;
	   		tmp2[p[i].cluster_id].y += p[i].y;
	   		tmp2[p[i].cluster_id].z += p[i].z;
		}
		//printf("Location %02d: %02d %02d\n", i, p[i].cluster_id, p_center  [p[i].cluster_id].cluster_id );
	}

	for(int i=0;i<c;i++)
	{
		p_center[i].x = tmp2[i].x/p_center[i].cluster_id;
		p_center[i].y = tmp2[i].y/p_center[i].cluster_id;
		p_center[i].z = tmp2[i].z/p_center[i].cluster_id;
		p_center[i].cluster_id = UNCLASSIFIED;
		//printf("Location %02d: %+.4f %+.4f %+.4f\n", i, p_center[i].x, p_center[i].y, p_center[i].z );
	}

	//cout << num_locations << c << endl;
	num_locations = c;

	free(p_tmp1);
	//free(p_center); done outside when function is called

	return p_center;
}

void decideBoundary(
	point_t &p,
	point_t *location,
	double *location_boundary,
	unsigned int num_locations)
{
	double location_contact = 0.0;
	point_t tmp_diff;
	location_contact = 0.0;
	p.cluster_id = UNCLASSIFIED;
	for(int ii=0;ii<(int)num_locations;ii++)
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
//	cout << p.cluster_id << "" << endl;
}

void contactBoundary(
	point_t *p,
	point_t *location,
	double *location_boundary,
	unsigned int num_points,
	unsigned int num_locations,
	bool learn)
{
	//if (learn)
		for(unsigned int i=0;i<num_locations;i++)
			location_boundary[i] = 0.95; //1.0 is the max

	for (unsigned int i=0;i<num_points;i++)
	{
		if (learn)
		{
			if (p[i].cluster_id<0) continue;
//			point_t tmp_diff = minusPoint(point2point3D(p[i]), location[p[i].cluster_id]);
//			location_boundary[p[i].cluster_id] =
//					min( pdfExp( 0.05, 0.0, l2Norm(tmp_diff) ),
//					     location_boundary[p[i].cluster_id]);
			continue;
		}
		else
			decideBoundary(p[i], location, location_boundary, num_locations);
	}
}


bool checkSurfaceRange(
	point_t pos_,
	point_t pos_surface_,
	double *surface_,
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
	double *surface_,
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




















//void classifierFeature(data_t motionData, point3D_t location,
//				 	   double &location_distance, double &location_angle)
//{
//	point3D_t direction, distance;
//	direction = minusVector(motionData.pos, location);
//	distance  = minusVector(motionData.pos, location);
//	location_distance = min(pdfExp( 0.05, 0.0, l2Norm(distance)), 1.0);
//	location_angle    = atan2(
//							  l2Norm(
//								crossProduct(point3D2vector(direction),
//											 point3D2vector(motionData.vel))),
//							  dotProduct(point3D2vector(direction),
//										 point3D2vector(motionData.vel)));
//	location_angle    = pdfExp(0.10, 0.0, fabs(location_angle)/M_PI);//1 - fabs(location_angle[ii]/M_PI);
//	//printf("Diff %.4f %.4f %.4f %.4f\n", tmp_diff[0], tmp_diff[1], tmp_diff[2], l2Norm(tmp_diff));
//	//printf("Distance to location %02d: %.4f\n", i, location_distance[ii]);
//	//printf("Angle to location %02d: %.4f\n", i, location_angle[ii]);
//}
//
//
//void classifierSVM(vector<data_t> motionData,
//		           int *label, unsigned int num_points,
//		           point3D_t *location, unsigned int num_locations,
//		           bool train)
//{
//	vector<double> location_distance(num_locations);
//	vector<double> location_angle(num_locations);
//	double vel_container = 0.0, acc_container = 0.0;
//	int data_boundary = 0, cluster_ = 0, tmp_id = 0;
//
//	if(train)
//		remove("data/data_svm.svm");
//	else
//		remove("data/data_svm2.svm");
//
//	ofstream write_file("data/data_svm2.svm", ofstream::app);
//
//	if(train)
//	{
//		write_file.close();
//		write_file.open("data/data_svm.svm", ofstream::app);
//	}
//
//	for(unsigned int i=0;i<num_points;i++)
//	{
//
//		if(motionData[i].vel.x==0 && motionData[i].vel.y==0 && motionData[i].vel.z==0) continue;
//
//		//vel_container = max( vel_container, l2Norm(vel_avg) );
//		//acc_container = max( acc_container, l2Norm(acc_avg) );
//		if(label[i]>=0)
//		{
//			write_file << label[i]+1 << " " ;
//			tmp_id = label[i]+1;
//		}
//		else
//			write_file << tmp_id+1 << " " ;
//
//		for(unsigned int ii=0;ii<num_locations;ii++)
//		{
//			classifierFeature(motionData[i], location[ii], location_distance[ii], location_angle[ii]);
//			write_file << ii*2+1 << ":" << location_distance[ii] << " "
//					   << ii*2+2 << ":" << location_angle[ii] << " ";
//
//		//	write_file << l2Norm(motionData[i].vel) << " ";
//		}
//		write_file << "\n";
//
//		if(!train)
//		{
//			svm_model *model_tmp = svm_load_model("model");
//			svm_node *x_tmp;
//			double *prob_estimates_tmp = NULL;
//			double predict_label_tmp = 0;
//			x_tmp = (struct svm_node *) malloc(8*sizeof(struct svm_node));
//			prob_estimates_tmp = (double *) malloc(3*sizeof(double));
//
//			for(unsigned int ii=0;ii<num_locations;ii++)
//			{
//				x_tmp[ii*2].index   = ii*2+1;
//				x_tmp[ii*2].value   = location_distance[ii];
//				x_tmp[ii*2+1].index = ii*2+2;
//				x_tmp[ii*2+1].value = location_angle[ii];
//			}
//			predict_label_tmp = svm_predict_probability(model_tmp,x_tmp,prob_estimates_tmp);
//
//			if(i<5)
//				cout << predict_label_tmp << " "<< prob_estimates_tmp[0]
//				                          << " "<< prob_estimates_tmp[1]
//				                          << " "<< prob_estimates_tmp[2] << endl;
//		}
//	}
//	write_file.close();
//}




// VTK ========================================================================
void colorCode(vector<unsigned char*> &container)
{
	// Setup colors
	unsigned char cw[]   = {255, 255, 255};
	unsigned char cy[]   = {255, 255, 0};
	unsigned char co[]   = {255, 127, 0};
	unsigned char cr[]   = {255, 0, 0};
	unsigned char clg[]  = {127, 255, 0};
	unsigned char cg[]   = {0, 255, 0};
	unsigned char cgb[]  = {0, 255, 127};
	unsigned char cc[]   = {0, 255, 255};
	unsigned char clb[]  = {0, 127, 255};
	unsigned char cb[]   = {0, 0, 255};
	unsigned char cpb[]  = {127, 0, 255};
	unsigned char cpr[]  = {255, 0, 127};
	copy(cw, 	cw+3, 	container[0]);
	copy(cy, 	cy+3, 	container[1]);
	copy(co, 	co+3, 	container[2]);
	copy(cr, 	cr+3, 	container[3]);
	copy(clg, 	clg+3,	container[4]);
	copy(cg, 	cg+3, 	container[5]);
	copy(cgb, 	cgb+3, 	container[6]);
	copy(cc, 	cc+3, 	container[7]);
	copy(clb, 	clb+3, 	container[8]);
	copy(cb, 	cb+3, 	container[9]);
	copy(cpb, 	cpb+3, 	container[10]);
	copy(cpr, 	cpr+3, 	container[11]);
	//	// Setup colors
	//	unsigned char cw[3]   = {255, 255, 255};
	//	unsigned char cy[3]   = {255, 255, 0};
	//	unsigned char co[3]   = {255, 127, 0};
	//	unsigned char cr[3]   = {255, 0, 0};
	//	unsigned char clg[3]  = {127, 255, 0};
	//	unsigned char cg[3]   = {0, 255, 0};
	//	unsigned char cgb[3]  = {0, 255, 127};
	//	unsigned char cc[3]   = {0, 255, 255};
	//	unsigned char clb[3]  = {0, 127, 255};
	//	unsigned char cb[3]   = {0, 0, 255};
	//	unsigned char cpb[3]  = {127, 0, 255};
	//	unsigned char cpr[3]  = {255, 0, 127};
	//	vector<unsigned char*> container(12);
	//	container[0] = cw;
	//	container[1] = cy;
	//	container[2] = co;
	//	container[3] = cr;
	//	container[4] = clg;
	//	container[5] = cg;
	//	container[6] = cgb;
	//	container[7] = cc;
	//	container[8] = clb;
	//	container[9] = cb;
	//	container[10] = cpb;
	//	container[11] = cpr;
}




// Define interaction style
class customMouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
	public:
		static customMouseInteractorStyle* New();
		vtkTypeMacro(customMouseInteractorStyle,
				     vtkInteractorStyleTrackballCamera);

		void setLeftButton(bool pick){pick_ = pick;}

		void setColors(vector<unsigned char*> color){color_ = color;}

		void setLabels(vector<string> label){LABEL = label;}

		vector<string> getLabels(){return LABEL;}

		void setNumberOfLabels(int x){num_locations = x;}

		virtual void OnLeftButtonDown();

/*
    virtual void OnMiddleButtonDown()
    {
      std::cout << "Pressed middle mouse button." << std::endl;
      // Forward events
      vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
    }

    virtual void OnRightButtonDown()
    {
      std::cout << "Pressed right mouse button." << std::endl;
      // Forward events
      vtkInteractorStyleTrackballCamera::OnRightButtonDown();
    }
*/
	private:
		int num_locations;
		bool pick_;
		vector<string> LABEL;
		vector<unsigned char*> color_;
		void writeText(const char* text, double *rgb, int x, int y);
};

// Define interaction style
void customMouseInteractorStyle::OnLeftButtonDown()
{
	if(pick_)
	{
		//std::cout << "Pressed left mouse button." << std::endl;
		int* clickPos = this->GetInteractor()->GetEventPosition();

		// Pick from this location.
		vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();
		picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

		double rgb[3];
		if (picker->GetActor()!=0)
		{
			double* pos = picker->GetPickPosition();
			std::cout << ">>>>> Pick position (world coordinates) is: "
					  << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;

			picker->GetActor()
				  ->GetMapper()->GetInput()
				  ->GetPointData()->GetScalars()
				  ->GetTuple(picker->GetPointId(),rgb);

			//std::cout << "Pick position (color) is: "
			//		  << rgb[0] << " " << rgb[1] << " " << rgb[2] << std::endl;
			//std::cout << "Picked actor: " << picker->GetActor() << std::endl;
		}

		for(int i = 0;i<num_locations;i++)
			if (rgb[0]==color_[i+1][0] &&
				rgb[1]==color_[i+1][1] &&
				rgb[2]==color_[i+1][2])
			{
				cout << ">>>>> Enter label : ";
				string mystr; getline (cin, mystr);
				printf(">>>>> Label %02d : %s\n", i+1, mystr.c_str());

				if (!LABEL[i+1].empty())
				{
					cout << ">>>>> [WARNING] : Label has been given. Do you wish to overwrite? [Y/N]" << endl;
					while(1)
					{
						string mystr2; getline (cin, mystr2);
						if(!strcmp(mystr2.c_str(),"Y"))
						{
							cout << ">>>>> [WARNING] : Label has been overwritten. New label : " << mystr << endl;
							LABEL[i+1] = mystr;
							writeText(mystr.c_str(), rgb, 10, 470-10*(i+1));
							printf(">>>>> Pick a location...\n");
							break;
						}
						if(!strcmp(mystr2.c_str(),"N"))
						{
							cout << ">>>>> [WARNING] : Label has not been overwritten." << endl;
							printf(">>>>> Pick a location...\n");
							break;
						}
					}
				}
				else
				{
					LABEL[i+1] = mystr;
					writeText(mystr.c_str(), rgb, 10, 470-10*(i+1));
					printf(">>>>> Pick a location...\n");
					break;
				}
			}

		for(int i = 1;i<num_locations+1;i++)
		{
			// when there is no labeling at all
			if (LABEL.empty()) break;
			// check for completeness of labelling
			if (LABEL[i].empty()) break;
			// else
			if (i==num_locations)
			{
				cout << ">>>>> [WARNING] : Label has been fully labeled. Proceed? [Y/N]" << endl;
				string mystr3; getline (cin, mystr3);
				if(!strcmp(mystr3.c_str(),"Y"))
					this->GetInteractor()->TerminateApp();
				else
					printf(">>>>> Pick a location...\n");
			}
		}
		// Forward events, camera manipulation
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	}
	else
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void customMouseInteractorStyle::writeText(const char* text, double *rgb, int x, int y)
{
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkTextActor> textActor;
	textActor = vtkSmartPointer<vtkTextActor>::New();
	textActor->SetInput ( text );
	textActor->SetPosition ( x, y );
	textActor->GetTextProperty()->SetFontSize ( 10 );
	textActor->GetTextProperty()->SetColor ( rgb[0]/255, rgb[1]/255, rgb[2]/255 );
	this->GetDefaultRenderer()->AddActor2D ( textActor );
	this->GetInteractor()->Render();
}

vtkStandardNewMacro(customMouseInteractorStyle);

void showData(
	point_t *p,
	unsigned int num_points,
	vector<string> &label,
	vector<unsigned char*> color_,
	bool cluster,
	bool labeling)
{

	int line_ = 0;
	int num_locations = 0;
	while(line_<num_points)
	{
		num_locations = max(p[line_].cluster_id,num_locations);
		++line_;
	}
	++num_locations;
	//cout << "Number of locations : " << num_locations << endl;

	// Create the geometry of a point (the coordinate)
	// add point to polydata to create the vertices for glyph
	// creating the vertices with a small cube glyph
	// add points and vertices to polydata
	// giving the points color
	// Create a mapper and actor
	// Create a renderer, render window, and interactor
	// custom mouse
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkPoints> 					points;
	vtkSmartPointer<vtkPolyData> 				pointsPolydata;
	vtkSmartPointer<vtkVertexGlyphFilter>		vertexFilter;
	vtkSmartPointer<vtkPolyData> 				polydata;
	vtkSmartPointer<vtkUnsignedCharArray> 		colors;
	vtkSmartPointer<vtkPolyDataMapper>			mapper;
	vtkSmartPointer<vtkActor> 					actor;
	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;
	vtkSmartPointer<vtkTextActor> 				textActor;

	points 					= vtkSmartPointer<vtkPoints>::New();
	pointsPolydata 			= vtkSmartPointer<vtkPolyData>::New();
	vertexFilter			= vtkSmartPointer<vtkVertexGlyphFilter>::New();
	polydata 				= vtkSmartPointer<vtkPolyData>::New();
	colors 					= vtkSmartPointer<vtkUnsignedCharArray>::New();
	mapper 					= vtkSmartPointer<vtkPolyDataMapper>::New();
	actor 					= vtkSmartPointer<vtkActor>::New();
	renderer 				= vtkSmartPointer<vtkRenderer>::New();
	renderWindow 			= vtkSmartPointer<vtkRenderWindow>::New();
	renderWindowInteractor 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	style 					= vtkSmartPointer<customMouseInteractorStyle>::New();
//	textActor 				= vtkSmartPointer<vtkTextActor>::New();

	for(int i=0;i<num_points;i++)
		points->InsertNextPoint(p[i].x, p[i].y, p[i].z);

	pointsPolydata->SetPoints(points);

	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	polydata->ShallowCopy(vertexFilter->GetOutput());

//#if VTK_MAJOR_VERSION <= 5
//	vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
//#else
//	vertexFilter->SetInputData(pointsPolydata);
//#endif

	if(cluster)
	{
		colors->SetNumberOfComponents(3);
		colors->SetName ("Colors");
		for(int i=0;i<num_points;i++)
		{
			if(p[i].cluster_id < num_locations && p[i].cluster_id >= 0)
				colors->InsertNextTypedTuple(color_[p[i].cluster_id+1]);
			else
				colors->InsertNextTypedTuple(color_[0]);
		}
		polydata->GetPointData()->SetScalars(colors);
	}

//	#if VTK_MAJOR_VERSION <= 5
//	  mapper->SetInput(polydata);
//	#else
//	  mapper->SetInputData(polydata);
//	#endif

	mapper->SetInputData(polydata);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(3);
//	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	renderWindow->SetSize(640,480); //(width, height)
	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);

	if(labeling)
	{
		style->setLeftButton(true);
		textActor = vtkSmartPointer<vtkTextActor>::New();
		printf(">>>>> Pick a location...\n");
		if(!label.empty()) textActor->SetInput ( label[0].c_str() );
		textActor->SetPosition ( 10, 470 );
		textActor->GetTextProperty()->SetFontSize ( 10 );
		textActor->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
		renderer->AddActor2D ( textActor );
	}
	else
	{
		style->setLeftButton(false);
		for(int i=0;i<num_locations+1;i++)
		{
			if(label.empty()) continue;
			if(label[i].empty()) continue;
			textActor = vtkSmartPointer<vtkTextActor>::New();
			textActor->SetInput(label[i].c_str());
			textActor->SetPosition(10, 470-i*10);
			textActor->GetTextProperty()->SetFontSize(10);
			textActor->GetTextProperty()
					 ->SetColor((double)color_[i][0]/255,
								(double)color_[i][1]/255,
								(double)color_[i][2]/255);
			renderer->AddActor2D(textActor);
		}
	}

	// Add the actor to the scene
	renderer->AddActor(actor);
	//renderer->SetBackground(nc->GetColor3d("MidnightBlue").GetData());
	style->SetDefaultRenderer(renderer);
	style->setNumberOfLabels(num_locations);
	style->setLabels(label);
	style->setColors(color_);
	renderWindowInteractor->SetInteractorStyle( style );
	renderWindow->Render();
	renderWindowInteractor->Start();

	label = style->getLabels();
}

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
	vector<unsigned char*> color_)
{
	vtkSmartPointer<vtkLineSource> 			lineSource	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyDataMapper> 		lineMapper	[Sqr(num_locations)];
	vtkSmartPointer<vtkActor> 				lineActor 	[Sqr(num_locations)];
	vtkSmartPointer<vtkPoints> 				points		[Sqr(num_locations)];
	vtkSmartPointer<vtkCellArray> 			lines 	   	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyData> 			polyData   	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkActor> 				tubeActor  	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors     	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkPolyData> 			polyData2  	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius2	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter2	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper2	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkActor> 				tubeActor2 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors2    	[Sqr(num_locations)][num_sector_intervals];

	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;

	for(int i=0;i<Sqr(num_locations);i++)
//	for(int i=0;i<2;i++)
	{
		lineSource[i] = vtkSmartPointer<vtkLineSource>::New();
		lineMapper[i] = vtkSmartPointer<vtkPolyDataMapper>::New();
		lineActor [i] = vtkSmartPointer<vtkActor>::New();

		points[i] = vtkSmartPointer<vtkPoints>::New();
		lines [i] = vtkSmartPointer<vtkCellArray>::New();

		int c = i/num_locations;

		if(i == c*num_locations+c) continue;

		// Create a line between each location
		lineSource[i]->SetPoint1(location[c].x,
				          	     location[c].y,
				          	     location[c].z);
		lineSource[i]->SetPoint2(location[c].x + (loc_loc_vec[i].x * loc_loc_norm[i]),
							     location[c].y + (loc_loc_vec[i].y * loc_loc_norm[i]),
							     location[c].z + (loc_loc_vec[i].z * loc_loc_norm[i]));

		lineMapper[i]->SetInputConnection(lineSource[i]->GetOutputPort());

		lineActor[i]->GetProperty()->SetLineWidth(5); // Give some color to the line
		lineActor[i]->GetProperty()->SetColor(0.0,0.0,1.0); // Give some color to the line
		lineActor[i]->SetMapper(lineMapper[i]);

		// Create points between each location
		lines[i]->InsertNextCell(num_location_intervals*2);

		for(int ii=0;ii<num_location_intervals;ii++)
		{
			points[i]->InsertPoint(ii*2+0,
				location[c].x + loc_loc_vec[i].x * loc_loc_norm[i] * (ii+0)/num_location_intervals,
				location[c].y + loc_loc_vec[i].y * loc_loc_norm[i] * (ii+0)/num_location_intervals,
				location[c].z + loc_loc_vec[i].z * loc_loc_norm[i] * (ii+0)/num_location_intervals);
			points[i]->InsertPoint(ii*2+1,
				location[c].x + loc_loc_vec[i].x * loc_loc_norm[i] * (ii+1)/num_location_intervals * 0.99,
				location[c].y + loc_loc_vec[i].y * loc_loc_norm[i] * (ii+1)/num_location_intervals * 0.99,
				location[c].z + loc_loc_vec[i].z * loc_loc_norm[i] * (ii+1)/num_location_intervals * 0.99);
			lines[i]->InsertCellPoint(ii*2+0);
			lines[i]->InsertCellPoint(ii*2+1);
		}

		for(int ii=0;ii<num_sector_intervals;ii++)
		{
			colors     [i][ii] = vtkSmartPointer<vtkUnsignedCharArray>::New();
			polyData   [i][ii] = vtkSmartPointer<vtkPolyData>::New();
			tubeRadius [i][ii] = vtkSmartPointer<vtkDoubleArray>::New();
			tubeFilter [i][ii] = vtkSmartPointer<vtkTubeFilter>::New();
			colors2    [i][ii] = vtkSmartPointer<vtkUnsignedCharArray>::New();
			polyData2  [i][ii] = vtkSmartPointer<vtkPolyData>::New();
			tubeRadius2[i][ii] = vtkSmartPointer<vtkDoubleArray>::New();
			tubeFilter2[i][ii] = vtkSmartPointer<vtkTubeFilter>::New();

			colors[i][ii]->SetNumberOfComponents(3);
			colors[i][ii]->SetName ("Colors");
			colors2[i][ii]->SetNumberOfComponents(3);
			colors2[i][ii]->SetName ("Colors");

			polyData[i][ii]->SetPoints(points[i]);
			polyData[i][ii]->SetLines(lines[i]);
			polyData2[i][ii]->SetPoints(points[i]);
			polyData2[i][ii]->SetLines(lines[i]);

			tubeRadius[i][ii]->SetName("TubeRadius");
			tubeRadius[i][ii]->SetNumberOfTuples(num_location_intervals*2); //to create the small gap to line between sectors
			tubeRadius2[i][ii]->SetName("TubeRadius");
			tubeRadius2[i][ii]->SetNumberOfTuples(num_location_intervals*2); //to create the small gap to line between sectors

			for (int iii=0;iii<num_location_intervals;iii++)
			{
				if (sector_constraint[i][iii][ii]==0)
				{
					colors[i][ii]->InsertNextTypedTuple(color_[5]);
					colors[i][ii]->InsertNextTypedTuple(color_[5]);
				}
				else
				{
					colors[i][ii]->InsertNextTypedTuple(color_[1]);
					colors[i][ii]->InsertNextTypedTuple(color_[1]);
				}

				colors2[i][ii]->InsertNextTypedTuple(color_[3]);
				colors2[i][ii]->InsertNextTypedTuple(color_[3]);

				if(sector[i][iii][ii].min<0)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else if(sector[i][iii][ii].min==INFINITY)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, sector[i][iii][ii].min);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, sector[i][iii][ii].min);
				}

				if(sector[i][iii][ii].max<=0)
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, sector[i][iii][ii].max);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, sector[i][iii][ii].max);
				}
			}

			polyData[i][ii]->GetPointData()->AddArray(tubeRadius[i][ii]);
			polyData[i][ii]->GetPointData()->SetActiveScalars("TubeRadius");
			polyData[i][ii]->GetPointData()->AddArray(colors[i][ii]);
			polyData2[i][ii]->GetPointData()->AddArray(tubeRadius2[i][ii]);
			polyData2[i][ii]->GetPointData()->SetActiveScalars("TubeRadius");
			polyData2[i][ii]->GetPointData()->AddArray(colors2[i][ii]);

//#if VTK_MAJOR_VERSION <= 5
//			tubeFilter[i][ii]->SetInputConnection(polyData[i][ii]->GetProducerPort());
//#else
//			tubeFilter[i][ii]->SetInputData(polyData[i][ii]);
//#endif

			tubeFilter[i][ii]->SetInputData(polyData[i][ii]);
			tubeFilter[i][ii]->SetNumberOfSides(num_sector_intervals);
			tubeFilter[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter[i][ii]->SidesShareVerticesOff();
			tubeFilter[i][ii]->SetOnRatio(num_sector_intervals);
			tubeFilter[i][ii]->SetOffset(ii);
			tubeFilter[i][ii]->Update();
			tubeFilter2[i][ii]->SetInputData(polyData2[i][ii]);
			tubeFilter2[i][ii]->SetNumberOfSides(num_sector_intervals);
			tubeFilter2[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter2[i][ii]->SidesShareVerticesOff();
			tubeFilter2[i][ii]->SetOnRatio(num_sector_intervals);
			tubeFilter2[i][ii]->SetOffset(ii);
			//cout << num_sector_intervals << ii <<endl;
			tubeFilter2[i][ii]->Update();

			tubeMapper[i][ii] = vtkSmartPointer<vtkPolyDataMapper>::New();
			tubeMapper[i][ii]->SetInputConnection(tubeFilter[i][ii]->GetOutputPort());
			tubeMapper[i][ii]->ScalarVisibilityOn();
			tubeMapper[i][ii]->SetScalarModeToUsePointFieldData();
			tubeMapper[i][ii]->SelectColorArray("Colors");
			tubeMapper2[i][ii] = vtkSmartPointer<vtkPolyDataMapper>::New();
			tubeMapper2[i][ii]->SetInputConnection(tubeFilter2[i][ii]->GetOutputPort());
			tubeMapper2[i][ii]->ScalarVisibilityOn();
			tubeMapper2[i][ii]->SetScalarModeToUsePointFieldData();
			tubeMapper2[i][ii]->SelectColorArray("Colors");

			tubeActor[i][ii] = vtkSmartPointer<vtkActor>::New();
			tubeActor[i][ii]->GetProperty()->SetOpacity(0.75); //Make the tube have some transparency.
			tubeActor[i][ii]->SetMapper(tubeMapper[i][ii]);
			tubeActor2[i][ii] = vtkSmartPointer<vtkActor>::New();
			tubeActor2[i][ii]->GetProperty()->SetOpacity(1.0); //Make the tube have some transparency.
			tubeActor2[i][ii]->SetMapper(tubeMapper2[i][ii]);
		}
	}

	style 				   	= vtkSmartPointer<customMouseInteractorStyle>::New();
	renderer               	= vtkSmartPointer<vtkRenderer>::New();
	renderWindow           	= vtkSmartPointer<vtkRenderWindow>::New();
	renderWindowInteractor 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();

	for(int i=0;i<Sqr(num_locations);i++)
//	for(int i=1;i<2;i++)
	{
//		int i = 6;
		renderer->AddActor(lineActor[i]);

		for(int ii=0;ii<num_sector_intervals;ii++)
		{
			renderer->AddActor(tubeActor[i][ii]);
			renderer->AddActor(tubeActor2[i][ii]);
		}
	}

	style->SetDefaultRenderer(renderer);
	style->setLeftButton(false);
	renderer->SetBackground(1.0,1.0,1.0);
	renderWindow->SetSize(1280,800); //(width, height)
	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindowInteractor->SetInteractorStyle(style);
	renderWindow->Render();
	renderWindowInteractor->Start();
}














