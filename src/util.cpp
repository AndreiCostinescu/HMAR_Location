/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#include "util.h"
#include "vtkExtra.h"

// For backward compatibility with new VTK generic data arrays
//#define InsertNextTypedTuple InsertNextTupleValue

// ============================================================================
// Labels
// ============================================================================

void labelMovement(
	string scene_,
	string object_,
	vector<string> &LABEL_MOV,
	int num_mov)
{
	reshapeVector(LABEL_MOV,num_mov);

	vector<vector<string> > data;
	string path;
	path =  "./Scene/" + scene_ + "/" + object_ + "/mov_data.txt";
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		LABEL_MOV[0] = {"MOVE"};
		LABEL_MOV[1] = {"SLIDE"};
		writeMovLabelFile(path,LABEL_MOV);
	}
	else
	{
		for(int ii=0;ii<data[0].size();ii++)
			LABEL_MOV[ii] = data[0][ii];
	}
	printf("Reviewing movement labels for OBJ...\n");
	for(int i=0;i<num_mov;i++)
		printf("MLabel %d : %s\n", i, LABEL_MOV[i].c_str());

	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<num_mov;i++)
			{
				printf("Enter new label for MLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					LABEL_MOV[i] = mystr;
				}
			}
			remove(path.c_str());
			writeMovLabelFile(path,LABEL_MOV);
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
}

void labelLocation(
	string scene_,
	string object_,
	vector<point_t> &points_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_,
	vector<string> &LABEL_LOC_,
	double epsilon_,
	int minpts_)
{
	int num_points = points_.size();
	int num_locations = locations_.size();

	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	point_t *points_array = Calloc(point_t,points_.size());
	vector2array(points_, points_array);

	vector<vector<string> > data;
	string path;
	path =  "./Scene/" + scene_ + "/" + object_ + "/loc_data.txt";
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
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
		reshapeVector(LABEL_LOC_, num_locations + 1);
		LABEL_LOC_[0] = {"CONNECTION"};
		showData(points_, LABEL_LOC_, color_code, true, true);
		writeLocLabelFile(path, LABEL_LOC_, locations_, location_boundary_);
	}
	else
	{
		int data_tmp  = 5;
		num_locations = data[0].size()/data_tmp;
		reshapeVector(locations_,		  num_locations);
		reshapeVector(location_boundary_, num_locations);
		reshapeVector(LABEL_LOC_,		  num_locations+1);
		LABEL_LOC_[0]  = {"CONNECTION"};
		for(int ii=0;ii<num_locations;ii++)
		{
			LABEL_LOC_[ii+1] 		  =      data[0][ii*data_tmp+0];
			locations_[ii].x 		  = atof(data[0][ii*data_tmp+1].c_str());
			locations_[ii].y 		  = atof(data[0][ii*data_tmp+2].c_str());
			locations_[ii].z 		  = atof(data[0][ii*data_tmp+3].c_str());
			locations_[ii].cluster_id = UNCLASSIFIED;
			location_boundary_[ii] 	  = atof(data[0][ii*data_tmp+4].c_str());
		}
		contactBoundary(points_, locations_, location_boundary_, false);
		showData(points_, LABEL_LOC_, color_code, true, false);
	}

	printf("Reviewing location labels...\n");
	for(int i=0;i<num_locations;i++)
		printf("LLabel %d : %s\n", i+1, LABEL_LOC_[i+1].c_str());

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
					LABEL_LOC_[i+1] = mystr;
				}
			}
			remove(path.c_str());
			writeLocLabelFile(path, LABEL_LOC_, locations_, location_boundary_);
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
}

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
	vector<unsigned char*> color_code_)
{
	int num_locations = locations_.size();

	vector<point_t> norm_location_dir   (Sqr(num_locations));
	vector<point_t> norm_location_normal(Sqr(num_locations));
	vector<double>  distance_location   (Sqr(num_locations));

	for(int i=0;i<Sqr(num_locations);i++)
	{
		norm_location_dir   [i].cluster_id = UNCLASSIFIED;
		norm_location_normal[i].cluster_id = UNCLASSIFIED;
	}

	// prepare the vectors from locations
	prepareSector(norm_location_dir, norm_location_normal,
				  distance_location, locations_);
	printf("Preparing sectors......Complete\n");

	// THE use of gaussian kernel helps to smoothen can create a tube like structure.
	// However it is still possible to have like bumps because the sampling is just not enough.
	int kernel_size = 5;
	vector<vector<double> > kernel(kernel_size);
	for(int i=0;i<kernel_size;i++) kernel[i].resize(kernel_size);
	gaussKernel(kernel, kernel_size, kernel_size, 1.0);

	generateSector(Graph_, sector_, pos_vel_acc_avg_, locations_,
			   	   norm_location_dir, norm_location_normal, distance_location,
			       num_location_intervals_, num_sector_intervals_,
			       file_eof_, kernel);
	printf("Generating sectors......Complete\n");

	checkSectorConstraint(sector_, sector_constraint_, num_locations,
						  num_location_intervals_, num_sector_intervals_);
	printf("Checking sector constraints......Complete\n");

	showConnection(sector_, sector_constraint_,
				   norm_location_dir, norm_location_normal, distance_location,
				   locations_,
				   num_location_intervals_, num_sector_intervals_,
				   color_code_);
	printf("Viewing sector......Complete\n");

	string path;
	path = 	"./Scene/" + scene_ + "/" + object_ + "/sec_data_max.txt";
	writeSectorFile(path, sector_, 0, num_locations,
			        num_location_intervals_, num_sector_intervals_);

	path = 	"./Scene/" + scene_ + "/" + object_ + "/sec_data_min.txt";
	writeSectorFile(path, sector_, 1, num_locations,
			        num_location_intervals_, num_sector_intervals_);

	path = 	"./Scene/" + scene_ + "/" + object_ + "/sec_data_const.txt";
	writeSectorConstraintFile(path, sector_constraint_, num_locations,
			        		  num_location_intervals_, num_sector_intervals_);
}

// ============================================================================
// Files
// ============================================================================

void writeSurfaceFile(
	string scene_,
	vector<vector<double> > surface_)
{
	string path = "./Scene/" + scene_ + "/surface.txt";
	if (!ifstream(path))
	{
		ofstream write_file(path, std::ios::app);
		for(int i=0;i<surface_.size();i++)
		{
			write_file << i;
			for(int ii=0;ii<4;ii++)
			{
				write_file << ",";
				write_file << surface_[i][ii];
			}
		}
		write_file << "\n";
	}
}

void writeLocLabelFile(
	string path_,
	vector<string> label_,
	vector<point_t> locations_,
	vector<double> boundary_)
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
					   << boundary_[i];
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
	string path_,
	vector<vector<vector<sector_t> > > sector_,
	int maxmin_,
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
					if (maxmin_)
						write_file << sector_[i][ii][iii].min;
					else
						write_file << sector_[i][ii][iii].max;

					if (i<sector_[i][ii].size()-1)
						write_file << ",";
				}
				write_file << "\n";
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
	string scene_,
	vector<vector<double> > &surface_)
{
	string path;
	vector<vector<string> > data;

	path = "./Scene/" + scene_ + "/surface.txt";
	readFile(path.c_str(), data, ',');

	reshapeVector(surface_, data.size());
	for(int i=0;i<data.size();i++)
	{
		int tmp = atoi(data[i][0].c_str());
		for(int ii=0;ii<4;ii++)
			surface_[tmp].push_back(atof(data[i][ii+1].c_str()));
	}
}

void readLocation(
	string path_,
	vector<string> &LABEL_LOC_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_)
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
		int data_tmp  = 5;
		num_locations = data[0].size()/data_tmp;
		reshapeVector(locations_,		  num_locations);
		reshapeVector(location_boundary_, num_locations);
		reshapeVector(LABEL_LOC_,		  num_locations+1);
		LABEL_LOC_[0]  = {"CONNECTION"};
		for(int ii=0;ii<num_locations;ii++)
		{
			LABEL_LOC_[ii+1] 		  =      data[0][ii*data_tmp+0];
			locations_[ii].x 		  = atof(data[0][ii*data_tmp+1].c_str());
			locations_[ii].y 		  = atof(data[0][ii*data_tmp+2].c_str());
			locations_[ii].z 		  = atof(data[0][ii*data_tmp+3].c_str());
			locations_[ii].cluster_id = UNCLASSIFIED;
			location_boundary_[ii] 	  = atof(data[0][ii*data_tmp+4].c_str());
		}
	}

	printf("Reviewing location labels...\n");
	for(int i=0;i<num_locations;i++)
		printf("LLabel %d : %s\n", i+1, LABEL_LOC_[i+1].c_str());

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
					LABEL_LOC_[i+1] = mystr;
				}
			}
			writeLocLabelFile(path_, LABEL_LOC_, locations_, location_boundary_);
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
}

void readMovement(
	string path_,
	vector<string> &LABEL_MOV_,
	int num_mov_)
{
	vector<vector<string> > data;
	readFile(path_.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Movement data is empty.");
	}
	else
	{
		reshapeVector(LABEL_MOV_, num_mov_);
		for(int ii=0;ii<data[0].size();ii++)
			LABEL_MOV_[ii] = data[0][ii];
	}
	printf("Reviewing movement labels for OBJ...\n");
	for(int i=0;i<num_mov_;i++)
		printf("MLabel %d : %s\n", i, LABEL_MOV_[i].c_str());

	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<num_mov_;i++)
			{
				printf("Enter new label for MLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					LABEL_MOV_[i] = mystr;
				}
			}
			writeMovLabelFile(path_,LABEL_MOV_);
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
}

void readSectorFile(
	string path_,
	vector<vector<vector<sector_t> > > &sector_,
	int maxmin_)
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

		if (sector_.empty())
		{
			reshapeVector(sector_,Sqr(num_locations));
			vector<vector<sector_t> > 	sector1(num_location_intervals);
				   vector<sector_t> 	sector2(num_sector_intervals);

			for(int i=0;i<Sqr(num_locations);i++)
			{
				sector_[i] = sector1;
				for(int ii=0;ii<num_location_intervals;ii++)
				{
					sector_[i][ii] = sector2;
					for(int iii=0;iii<num_sector_intervals;iii++)
					{
						sector_[i][ii][iii].min = INFINITY;
						sector_[i][ii][iii].max = 0;
					}
				}
			}
		}

		for(int i=0;i<Sqr(num_locations);i++)
		{
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				int tmp = i*num_location_intervals+ii+3;
				for(int iii=0;iii<num_sector_intervals;iii++)
				{
					if (maxmin_)
						sector_[i][ii][iii].min = atof(data[tmp][iii].c_str());
					else
						sector_[i][ii][iii].max = atof(data[tmp][iii].c_str());
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
	point_t pos,
	vector< vector< vector<double> > > &pos_vel_acc_mem, // motion,xyz,length
	vector<point_t> &pos_vel_acc_avg,
	unsigned int window)
{
	point_t vel = minusPoint(pos,pos_vel_acc_avg[0]);
	point_t acc = minusPoint(vel,pos_vel_acc_avg[1]);

	if (pos_vel_acc_mem[2][0].size() == window)
	{
		averagePoint(pos, pos_vel_acc_mem[0], pos_vel_acc_avg[0]);
		averagePoint(vel, pos_vel_acc_mem[1], pos_vel_acc_avg[1]);
		averagePoint(acc, pos_vel_acc_mem[2], pos_vel_acc_avg[2]);
	}
	else if (pos_vel_acc_mem[1][0].size() == window)
	{
		averagePoint         (pos, pos_vel_acc_mem[0], pos_vel_acc_avg[0]);
		averagePoint         (vel, pos_vel_acc_mem[1], pos_vel_acc_avg[1]);
		averagePointIncrement(acc, pos_vel_acc_mem[2], pos_vel_acc_avg[2]);
	}
	else if (pos_vel_acc_mem[0][0].size() == window)
	{
		averagePoint         (pos, pos_vel_acc_mem[0], pos_vel_acc_avg[0]);
		averagePointIncrement(vel, pos_vel_acc_mem[1], pos_vel_acc_avg[1]);
		averagePointIncrement(acc, pos_vel_acc_mem[2], pos_vel_acc_avg[2]);
	}
	else if (pos_vel_acc_mem[0][0].size() >= 3)
	{
		averagePointIncrement(pos, pos_vel_acc_mem[0], pos_vel_acc_avg[0]);
		averagePointIncrement(vel, pos_vel_acc_mem[1], pos_vel_acc_avg[1]);
		averagePointIncrement(acc, pos_vel_acc_mem[2], pos_vel_acc_avg[2]);
	}
	else if (pos_vel_acc_mem[0][0].size() == 2)
	{
		averagePointIncrement(pos, pos_vel_acc_mem[0], pos_vel_acc_avg[0]);
		averagePointIncrement(vel, pos_vel_acc_mem[1], pos_vel_acc_avg[1]);
		pos_vel_acc_mem[2][0].push_back(acc.x);
		pos_vel_acc_mem[2][1].push_back(acc.y);
		pos_vel_acc_mem[2][2].push_back(acc.z);
		pos_vel_acc_avg[2] = acc;
	}
	else if (pos_vel_acc_mem[0][0].size() == 1)
	{
		averagePointIncrement(pos, pos_vel_acc_mem[0], pos_vel_acc_avg[0]);
		pos_vel_acc_mem[1][0].push_back(vel.x);
		pos_vel_acc_mem[1][1].push_back(vel.y);
		pos_vel_acc_mem[1][2].push_back(vel.z);
		pos_vel_acc_avg[1] = vel;
		pos_vel_acc_avg[2].x = pos_vel_acc_avg[2].y = pos_vel_acc_avg[2].z = 0;
	}
	else
	{
		pos_vel_acc_mem[0][0].push_back(pos.x);
		pos_vel_acc_mem[0][1].push_back(pos.y);
		pos_vel_acc_mem[0][2].push_back(pos.z);
		pos_vel_acc_avg[0] = pos;
		pos_vel_acc_avg[1].x = pos_vel_acc_avg[1].y = pos_vel_acc_avg[1].z = 0;
		pos_vel_acc_avg[2].x = pos_vel_acc_avg[2].y = pos_vel_acc_avg[2].z = 0;
	}

	pos_vel_acc_avg[0].cluster_id = UNCLASSIFIED;
	pos_vel_acc_avg[1].cluster_id = UNCLASSIFIED;
	pos_vel_acc_avg[2].cluster_id = UNCLASSIFIED;
}

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
	vector<vector<double> > kernel_)
{
	int tmp_id1, tmp_id2, file_num;
	tmp_id1 = tmp_id2 = file_num = 0;
	bool flag_next = true;
	data_t motion_data;
	vector<data_t> edge_data;

	for(int i=0;i<pos_vel_acc_avg_.size();i++)
	{
		if(i==file_eof_[file_num]-1)
		{
			flag_next = true;
			tmp_id1  = 0;
			tmp_id2 = 0;
			continue;
		}

		if (pos_vel_acc_avg_[i][0].cluster_id >= 0)
		{
			Graph_.addEdge(edge_data,tmp_id1,tmp_id2,
							 num_location_intervals_,num_sector_intervals_,
							 sector_[tmp_id1*locations_.size()+tmp_id2]);
			flag_next = true;
			tmp_id1 = pos_vel_acc_avg_[i][0].cluster_id;
			edge_data.clear();
			continue;
		}

		if(flag_next)
		{
			flag_next = false;
			for(int ii=i;ii<pos_vel_acc_avg_.size();ii++)
			{
				tmp_id2 = pos_vel_acc_avg_[ii][0].cluster_id;
				if(pos_vel_acc_avg_[ii][0].cluster_id >= 0)
					break;
			}
		}

		if(tmp_id1 != tmp_id2)
		{
			updateSector(sector_, pos_vel_acc_avg_[i][0], locations_,
						 tmp_dir_, tmp_dir_normal_, tmp_norm_,
						 num_location_intervals_, num_sector_intervals_,
						 tmp_id1, tmp_id2, kernel_);
			motion_data.pos = pos_vel_acc_avg_[i][0];
			motion_data.vel = pos_vel_acc_avg_[i][1];
			motion_data.acc = pos_vel_acc_avg_[i][2];
			edge_data.push_back(motion_data);
		}
	}
}

void prepareSector(
	vector<point_t> &tmp_dir,
	vector<point_t> &tmp_dir_normal,
	vector<double> &tmp_norm,
	vector<point_t> location)
{
	int c = 0;
	int num_locations = location.size();
	for(int i=0;i<num_locations;i++)
	{
		for(int ii=0;ii<num_locations;ii++)
		{
			if(i == ii)
			{
				tmp_dir[c].x =
				tmp_dir[c].y =
				tmp_dir[c].z = 0.0;
				tmp_norm[c] = 0.0;
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
			B[0] = 0.0; B[1] = 1.0; B[2] = 0.0;
			tmp_dir_normal[c] = vector2point(
									crossProduct(
										point2vector(tmp_dir[c]), B));
			++c;
		}
	}
}

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
	vector<vector<double> > kernel_)
{
	double angle_tmp = 0.0, p_ = 0.0;
	int xx = 0, yy = 0, zz = 0;
	point_t tmp_diff, p_dir, t_;

	xx       = tmp_id1_*locations_.size()+tmp_id2_;

	tmp_diff = minusPoint(pos_, locations_[tmp_id1_]);

	p_       = dotProduct(point2vector(tmp_diff), point2vector(tmp_dir_[xx]));

	p_dir    = tmp_dir_[xx]; // along the direction vector
	p_dir.x *= p_;
	p_dir.y *= p_;
	p_dir.z *= p_;
	t_       = minusPoint(tmp_diff,p_dir);

	angle_tmp = atan2(l2Norm(crossProduct(point2vector(t_),
											  point2vector(tmp_dir_normal_[xx]))),
							   dotProduct(point2vector(t_),
										  point2vector(tmp_dir_normal_[xx])));
	//angle_tmp[ii] = angle_tmp[ii] / M_PI * 180;

	// the value of yy can be > or <  "num_location_intervals"
	// > object moved further away from the location area
	// < object is moving initially away from the intended goal location
	yy 		 = ceil(p_*num_location_intervals_/tmp_norm_[xx])  -1;
	zz 		 = ceil(angle_tmp*num_sector_intervals_/M_PI) -1;

	if(yy<num_location_intervals_ &&
	   zz<num_sector_intervals_ &&
	   yy>=0 &&
	   zz>=0)
	{
		sector_[xx][yy][zz].max = max(l2Norm(t_), sector_[xx][yy][zz].max);
		sector_[xx][yy][zz].min = min(l2Norm(t_), sector_[xx][yy][zz].min);
		double mid = (sector_[xx][yy][zz].max + sector_[xx][yy][zz].min)/2;
		double tmp_ratio1 = (sector_[xx][yy][zz].max-mid) /
				             kernel_[(kernel_.size()-1)/2]
				                    [(kernel_[0].size()-1)/2];

		for(int i=0;i<kernel_.size();i++)
		{
			for(int ii=0;ii<kernel_[0].size();ii++)
			{
				int tmp1 = i -(kernel_.size()/2)+zz;
				int tmp2 = ii-(kernel_[0].size()/2)+yy;
				if (tmp1<0)
					tmp1 += num_sector_intervals_;
				else if (tmp1>=num_sector_intervals_)
					tmp1 %= num_sector_intervals_;
				if (tmp2<0)
					continue;
				else if (tmp2>=num_location_intervals_)
					continue;
				sector_[xx][tmp2][tmp1].max =
						max((kernel_[i][ii]*tmp_ratio1)+mid ,
							sector_[xx][tmp2][tmp1].max);
				sector_[xx][tmp2][tmp1].min =
						min(mid-(kernel_[i][ii]*tmp_ratio1) ,
							sector_[xx][tmp2][tmp1].min);
			}
		}
	}
}

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
	bool learn)
{
	int num_locations = location.size();
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

		//cout << yy << " " << zz << " ";

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
					//printf(" %.4f %.4f %.4f %.4f   ", l2Norm(t_), t_val[i], sector[xx][yy][zz].max, sector[xx][yy][zz].min);
					sector[xx][yy][zz].max = max(l2Norm(t_),sector[xx][yy][zz].max);
					sector[xx][yy][zz].min = min(l2Norm(t_),sector[xx][yy][zz].min);
				}
				else
				{
					prediction[i] = 2;
					t_val[i]      = l2Norm(t_);
				}
			}
		}
	}
}

void checkSectorConstraint(
	vector<vector<vector<sector_t> > > 	sector,
	vector<vector<vector<double> > > &sector_constraint,
	int num_locations,
	int num_location_intervals,
	int num_sector_intervals)
{

	for(int i=0;i<Sqr(num_locations);i++)
		for(int ii=0;ii<num_location_intervals;ii++)
			for(int iii=0;iii<num_sector_intervals;iii++)
			{
				sector_constraint[i][ii][iii] =
					(sector[i][ii][iii].max - sector[i][ii][iii].min > 0) &&
					(0.06 - (sector[i][ii][iii].max - sector[i][ii][iii].min) > 0) ?
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

















