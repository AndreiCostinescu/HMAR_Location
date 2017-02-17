//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#define SAVEDDATA
//#define DBSCAN

#include "dataDeclaration.h"
#include "algo.h"
#include "util.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"


//=============================================================================
// Global
//=============================================================================
unsigned int window = 5;

vector<string> LABEL_MOV;
vector<string> LABEL_LOC_MOV;
vector<string> LABEL_LOC;




//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{
	//[VARIABLES]**************************************************************

	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	int num_points 		= 0;
	int num_locations	= 0;
	int file_num 		= 0;

	int num_surfaces 	= 1;
	int num_mov 		= 2;
	int obj 			= 3;
	int num_objs 		= 4;

	double epsilon 		= 0.015; 	// if datasets are merged these 2 values can be increased.
	int minpts 			= 10;

	double vel_limit 	= 0.01; //##### still need to validate

	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;

	DIR *dir;
	struct dirent *dent;
	char *name;

	vector<int> file_eof;

	vector<vector<string> > data_full;
	vector<vector<string> > data_test;

	point_t *points_array;
	vector<point_t> points;
	vector<point_t> points_test;
	vector<point_t> locations;

	vector<double> location_boundary;

	vector<double> 			surface_const(4);
	vector<vector<double> > surface(num_surfaces);
	for(int i=0;i<num_surfaces;i++) surface[i] = surface_const;

	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<vector<point_t> > 	pos_vel_acc_avg;

	Graph Graph[2];

	vector<vector<data_t> > node_data;
	vector<vector<double> > node_data_tmp;
	//**************************************************************[VARIABLES]

	//[READ FILE]**************************************************************
	string dir_name = "./../KINECT/data/";
	dir = opendir(dir_name.c_str());
	if(dir!=NULL)
	{
		while((dent=readdir(dir))!=NULL)
		{
			if (strcmp(dent->d_name,".")==0 ||
				strcmp(dent->d_name,"..")==0 ||
				(*dent->d_name) == '.' )
				continue;

			string fn = string(dent->d_name);
			size_t found_extension = fn.find(".txt");
			if(found_extension==std::string::npos)
				continue;

			name = new char[256];
			sprintf(name, "%s%s",dir_name.c_str(),dent->d_name);
			readFile(name, data_full , ',');
			file_eof.push_back(data_full.size());
			delete name;

			//table
			vector<string> last_line = data_full[data_full.size()-1];
			surface[0][0] = atof(last_line[5].c_str());
			surface[0][1] = atof(last_line[6].c_str());
			surface[0][2] = atof(last_line[7].c_str());
			surface[0][3] = atof(last_line[8].c_str());
		}
	}
	closedir(dir);
	//**************************************************************[READ FILE]
	printf("Reading training data......Complete\n");


	//[PARSE DATA]*************************************************************
	num_points = data_full.size();
	points.clear();
	points.resize(num_points);
	parseData2Point(data_full, points);
	//*************************************************************[PARSE DATA]
	printf("Parsing training data......Complete\n");

	// LEARNING
	//[PREPROCESS DATA]********************************************************
	vector<vector<double> > 		  	pos_vel_acc_mem1(3);
	vector<vector< vector<double> > > 	pos_vel_acc_mem;
	for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);

	pos_vel_acc_avg.clear();
	pos_vel_acc_avg.resize(num_points);
	for(int i=0;i<num_points;i++)
	{
		if(i == file_eof[file_num])
		{
			file_num++;
			pos_vel_acc_mem.clear();
			for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
		}
		if(i==0)
			pos_vel_acc_avg[i] = pos_vel_acc_avg1;
		else
			pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1];
		preprocessDataLive(points[i], pos_vel_acc_mem,
						   pos_vel_acc_avg[i], window);
	}
	//********************************************************[PREPROCESS DATA]
	printf("Preprocessing data......Complete\n");

	//[LABELLING MOVEMENT]*****************************************************
	labelingMovement(LABEL_MOV, num_mov, obj, num_objs);
	//*****************************************************[LABELLING MOVEMENT]

	//[LABELLING LOCATION]*****************************************************
	labelingLocation(points, locations, location_boundary, LABEL_LOC,
					 obj, epsilon, minpts);
	num_locations = locations.size();
	//*****************************************************[LABELLING LOCATION]
	printf("Labeling of clusters (action locations)......Complete\n");



//[GRAPH]**********************************************************************
	//[ADD NODES]**************************************************************
	// Collecting data based on locations/nodes
	node_data.clear();
	node_data.resize(num_locations);
	data_t motion_data;
	for(int i=0;i<num_points;i++)
		if (points[i].cluster_id >= 0)
		{
			motion_data.pos = pos_vel_acc_avg[i][0];
			motion_data.vel = pos_vel_acc_avg[i][1];
			motion_data.acc = pos_vel_acc_avg[i][2];
			node_data[points[i].cluster_id].push_back(motion_data);
		}

	// Adding nodes to the graph : nodes corresponds to the locations
	for(int i=0;i<num_locations;i++)
	{
		vector<double> tmp_vec(3);
		int surface_num  =    -1;
		double dist_tmp  = 	 0.0;
		double dist_tmp2 =  10.0; //### need improvement
		double tmp_spd 	 =   0.0;
		double tmp_dir 	 =   0.0;
		for(int ii=0;ii<node_data[i].size();ii++)
		{
			for(int iii=0;iii<num_surfaces;iii++)
			{
				dist_tmp = surfaceDistance(node_data[i][ii].pos,surface[iii]);
				if (dist_tmp < 0.1 && !max_(dist_tmp,dist_tmp2))
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

		if (surface_num >= 0 &&
			tmp_spd/node_data[i].size() > vel_limit &&
			tmp_dir/node_data[i].size() > 0.96) // sind(75)
			Graph[0].addNode(LABEL_LOC[i],1,surface_num,location_boundary[i],node_data[i]);
		else
			Graph[0].addNode(LABEL_LOC[i],0,surface_num,location_boundary[i],node_data[i]);
	}
	//**************************************************************[ADD NODES]
	//[VISUALIZE NODES]********************************************************
	bool pos_flag = true;
	bool vel_flag = false;
	bool acc_flag = false;

	node_data_tmp = Graph[0].getNodeDataLabel(pos_flag,vel_flag,acc_flag);

	vector<point_t> pos_node(node_data_tmp.size());
	vector<point_t> vel_node(node_data_tmp.size());
	vector<point_t> acc_node(node_data_tmp.size());
	for(int i=0; i<node_data_tmp.size();i++)
	{
		vector<double> tmp_data = node_data_tmp[i];
		int c = 0;
		if(pos_flag)
		{
			pos_node[i].x = tmp_data[c]; ++c;
			pos_node[i].y = tmp_data[c]; ++c;
			pos_node[i].z = tmp_data[c]; ++c;
			pos_node[i].cluster_id = tmp_data[c]; ++c;
		}
		if(vel_flag)
		{
			vel_node[i].x = tmp_data[c]; ++c;
			vel_node[i].y = tmp_data[c]; ++c;
			vel_node[i].z = tmp_data[c]; ++c;
			vel_node[i].cluster_id = tmp_data[c]; ++c;
		}
		if(acc_flag)
		{
			acc_node[i].x = tmp_data[c]; ++c;
			acc_node[i].y = tmp_data[c]; ++c;
			acc_node[i].z = tmp_data[c]; ++c;
			acc_node[i].cluster_id = tmp_data[c]; ++c;
		}
	}
	//********************************************************[VISUALIZE NODES]
	printf("Creating nodes for clusters (action locations)......Complete\n");

	//[EDGES]******************************************************************
	// Initialize sector

	vector<vector<vector<sector_t> > > 	sector (Sqr(num_locations));
           vector<vector<sector_t> > 	sector1(num_location_intervals);
	              vector<sector_t> 		sector2(num_sector_intervals);

	vector<vector<vector<double> > > sector_constraint (Sqr(num_locations));
	       vector<vector<double> > 	 sector_constraint1(num_location_intervals);
	              vector<double>  	 sector_constraint2(num_sector_intervals);

	for(int i=0;i<Sqr(num_locations);i++)
	{
		sector[i] 				= sector1;
		sector_constraint[i] 	= sector_constraint1;
		for(int ii=0;ii<num_location_intervals;ii++)
		{
			sector[i][ii] 				= sector2;
			sector_constraint[i][ii] 	= sector_constraint2;
			for(int iii=0;iii<num_sector_intervals;iii++)
			{
				sector[i][ii][iii].max = 0;
				sector[i][ii][iii].min = INFINITY;
			}
		}
	}

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
				  distance_location, locations);

	// THE use of gaussian kernel helps to smoothen can create a tube like structure.
	// However it is still possible to have like bumps because the sampling is just not enough.
	int kernel_size = 5;
	vector<vector<double> > kernel(kernel_size);
	for(int i=0;i<kernel_size;i++) kernel[i].resize(kernel_size);
	gaussKernel(kernel, kernel_size, kernel_size, 1.0);

	generateSector(sector, points, locations,
				   norm_location_dir, norm_location_normal, distance_location,
			       num_location_intervals, num_sector_intervals,
			       file_eof, kernel);

	checkSectorConstraint(sector, sector_constraint, num_locations,
						  num_location_intervals, num_sector_intervals);

	showConnection(sector, sector_constraint,
				   norm_location_dir, norm_location_normal, distance_location,
				   locations,
				   num_location_intervals, num_sector_intervals, color_code);
	//******************************************************************[EDGES]
	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");

//**********************************************************************[GRAPH]
printf("Creating a graph to represent the clusters (action locations)......Complete\n");






//NEED : surface, sectormap, boundary, locations
//[TESTING]********************************************************************
	//Read file will be replaced later on
	//[READ FILE]**************************************************************
	file_eof.clear();
	dir_name = "../KINECT/data/test/";
	dir = opendir(dir_name.c_str());

	if(dir!=NULL)
	{
		while((dent=readdir(dir))!=NULL)
		{
			if (strcmp(dent->d_name,".")==0 ||
				strcmp(dent->d_name,"..")==0 ||
				(*dent->d_name) == '.' )
				continue;

			string fn = string(dent->d_name);
			size_t found_extension = fn.find(".txt");
			if(found_extension==std::string::npos)
				continue;

			name = new char[256];
			sprintf(name, "%s%s",dir_name.c_str(),dent->d_name);
			readFile( name, data_test , ',');
			file_eof.push_back(data_test.size());
			delete name;

			//table
			vector<string> last_line = data_full[data_full.size()-1];
			surface[0][0] = atof(last_line[5].c_str());
			surface[0][1] = atof(last_line[6].c_str());
			surface[0][2] = atof(last_line[7].c_str());
			surface[0][3] = atof(last_line[8].c_str());
		}
	}
	closedir(dir);
	//**************************************************************[READ FILE]
	printf("Reading test data......Complete\n");

	//[PARSE DATA]*************************************************************
	num_points = data_test.size();
	points_test.clear();
	points_test.resize(num_points);
	parseData2Point(data_test, points_test);
	//*************************************************************[PARSE DATA]
	printf("Parsing test data......Complete\n");

//	//[PREPROCESS DATA]********************************************************
//	pos_vel_acc_mem.clear();
//	for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
//
//	pos_vel_acc_avg.clear();
//	pos_vel_acc_avg.resize(num_points);
//	for(int i=0;i<num_points;i++)
//	{
//		if(i == file_eof[file_num])
//		{
//			file_num++;
//			pos_vel_acc_mem.clear();
//			for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
//		}
//		pos_vel_acc_avg[i] = pos_vel_acc_avg1;
//		preprocessDataLive(points_test[i], pos_vel_acc_mem,
//						   pos_vel_acc_avg[i], window);
//	}
//	//********************************************************[PREPROCESS DATA]
//	printf("Preprocessing data......Complete\n");

	//[PREPROCESS DATA]********************************************************
	pos_vel_acc_mem.clear();
	for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
	pos_vel_acc_avg.clear();
	pos_vel_acc_avg.resize(num_points);
	//********************************************************[PREPROCESS DATA]

	//[GRAPH TEMPORARY]********************************************************
	//[ADD NODES]**************************************************************
	// Initialize sector backup
	vector<vector<vector<sector_t> > > 	sector_mem(Sqr(num_locations));

	for(int i=0;i<Sqr(num_locations);i++)
	{
		sector_mem[i] = sector1;
		for(int ii=0;ii<num_location_intervals;ii++)
		{
			sector_mem[i][ii] = sector2;
			for(int iii=0;iii<num_sector_intervals;iii++)
			{
				sector_mem[i][ii][iii].max = sector[i][ii][iii].max;
				sector_mem[i][ii][iii].min = sector[i][ii][iii].min;
			}
		}
	}

	bool flag_predict = false;
	double prediction1[num_locations];
	double prediction2[num_locations];
	vector<double> prediction1_(num_locations);

	double prediction_curr[num_locations];
	int last_prediction_flag = -1;
	double cc = 1;

	vector<double> prediction(num_locations);
	vector<double> t_val(num_locations);
	vector<data_t> tmp_data;
	bool learn = false;
	bool slide = false;
	int last_location = 0;
	int surface_num_tmp = 0;

	node_data.clear();
	node_data.resize(num_locations);
	motion_data = {};
	for(int i=0;i<num_points;i++)
		if (points_test[i].cluster_id >= 0)
		{
			motion_data.pos = pos_vel_acc_avg[i][0];
			motion_data.vel = pos_vel_acc_avg[i][1];
			motion_data.acc = pos_vel_acc_avg[i][2];
			node_data[points_test[i].cluster_id].push_back(motion_data);
		}

	for(int i=0;i<num_points;i++)
	{
		//[PREPROCESS DATA]********************************************************
		if(i == file_eof[file_num])
		{
			file_num++;
			pos_vel_acc_mem.clear();
			for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
		}
		pos_vel_acc_avg[i] = pos_vel_acc_avg1;
		preprocessDataLive(points_test[i], pos_vel_acc_mem,
						   pos_vel_acc_avg[i], window);
		//********************************************************[PREPROCESS DATA]

		slide = false;

		decideBoundary(pos_vel_acc_avg[i][0], locations, location_boundary);

		cout << i << " " << pos_vel_acc_avg[i][0].cluster_id << " ";

		if (pos_vel_acc_avg[i][0].cluster_id < 0)
		{
//			learn = true;

			checkSector(prediction, t_val, sector,
						pos_vel_acc_avg[i][0],
						locations,
						norm_location_dir, norm_location_normal,
						distance_location,
						num_location_intervals, num_sector_intervals,
						last_location, learn);

			if (l2Norm(pos_vel_acc_avg[i][1])> vel_limit)
			{
				for(int ii=0;ii<num_surfaces;ii++)
				{
					if (!slide)
					{
						slide =	checkMoveSlide(pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1], surface[ii], 0.1, 0.97); //####need to add
						surface_num_tmp = ii;
					}
				}

				if (slide)
					cout << " LABEL: "<< LABEL_MOV[1] << " surface " << surface_num_tmp;
				else
					cout << " LABEL: "<< LABEL_MOV[0];
			}
			else
				cout << " LABEL: "<< "NULL";



			for(int ii=0;ii<num_locations;ii++)
			{
				if ((int)prediction[ii]==1)
				{
					prediction1[ii]  = 1.0;
					prediction2[ii]  = 0.0;
				}
				else if ((int)prediction[ii]>1)
				{
					prediction1[ii]  = 0.0;
					prediction2[ii]  = t_val[ii];
				}
				else
				{
					prediction1[ii]  = 0.0;
					prediction2[ii]  = 0.0;
				}
			}

			double tmp_t = 0.0;
			for(int ii=0;ii<num_locations;ii++)
				tmp_t += prediction1[ii];
			for(int ii=0;ii<num_locations;ii++)
				if (tmp_t>0)
					prediction1[ii] /= tmp_t;

			if (tmp_t>0) flag_predict = false;

			double tmp_t2 = 0.0;
			for(int ii=0;ii<num_locations;ii++)
				tmp_t2 += prediction2[ii];
			for(int ii=0;ii<num_locations;ii++)
				if (prediction2[ii] != 0)
					if (prediction2[ii] != tmp_t2)
						prediction2[ii] = 1.0 - (prediction2[ii]/tmp_t2);
					else
						prediction2[ii] = 1.0;

			double tmp_t3 = 0.0;
			for(int ii=0;ii<num_locations;ii++)
				tmp_t3 += prediction2[ii];
			for(int ii=0;ii<num_locations;ii++)
				if (tmp_t3>0)
					prediction2[ii] /= tmp_t3;

			if (flag_predict)
			{
				for(int ii=0;ii<num_locations;ii++)
				{
					if (prediction1_[ii]>0 && prediction[ii]>0)
						prediction2[ii] = prediction2[ii] * (1-pow(0.5,cc)) + pow(0.5,cc);
					else if (prediction2[ii]!=1.0)
						prediction2[ii] = prediction2[ii] * (1-pow(0.5,cc));
				}
				cc += 0.01;
			}
			else
			{
				for(int ii=0;ii<num_locations;ii++)
					prediction1_[ii] = prediction1[ii];
				cc = 1;
			}

			if (tmp_t>0) flag_predict = true;

			for(int ii=0;ii<num_locations;ii++)
				//printf(" %d %.4f ", (int)prediction[ii], prediction2[ii]);
				printf(" %.4f ", prediction2[ii]);
			for(int ii=0;ii<num_locations;ii++)
				if((int)prediction[ii]==1)
					printf(" %s %.4f ", LABEL_LOC[ii+1].c_str(), prediction1[ii]);

		}
		else
		{
			flag_predict = false;

			if (LABEL_LOC[pos_vel_acc_avg[i][0].cluster_id+1].empty())
			{
				cout << " LABEL: "<< "Empty location Label.";
				if (l2Norm(pos_vel_acc_avg[i][1])> vel_limit)
				{
					for(int ii=0;ii<num_surfaces;ii++)
					{
						if (!slide)
						{
							slide =	checkMoveSlide(pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1], surface[ii], 0.1, 0.96); //####needto add
							surface_num_tmp = ii;
						}
					}

					if (slide)
						cout << " LABEL: "<< LABEL_MOV[1] << " surface " << surface_num_tmp;
					else
						cout << " LABEL: "<< LABEL_MOV[0];
				}
				else
					cout << " LABEL: "<< "NULL";
			}
			else
				cout << " LABEL: "<< LABEL_LOC[pos_vel_acc_avg[i][0].cluster_id+1];

			// if it is moved back to the same location
			if (last_location != pos_vel_acc_avg[i][0].cluster_id)
			{
				if (learn)
				{
					// copy only the intended values for sectors
					// updating the values in memory
					int c = last_location * num_locations + pos_vel_acc_avg[i][0].cluster_id;
					for(int ii=0;ii<num_location_intervals;ii++)
					{
						for(int iii=0;iii<num_sector_intervals;iii++)
						{
							sector_mem[c][ii][iii].max = sector[c][ii][iii].max;
							sector_mem[c][ii][iii].min = sector[c][ii][iii].min;
						}
					}
					// update the sector using values from memory
					for(int ii=0;ii<num_locations;ii++)
					{
						int cc = last_location * num_locations + ii;
						for(int iii=0;iii<num_location_intervals;iii++)
						{
							for(int iiii=0;iiii<num_sector_intervals;iiii++)
							{
								sector[cc][iii][iiii].max = sector_mem[cc][iii][iiii].max;
								sector[cc][iii][iiii].min = sector_mem[cc][iii][iiii].min;
							}
						}
					}
					learn = false;
				}
				else
				{
					// update the sector using values from memory
					for(int ii=0;ii<num_locations;ii++)
					{
						int cc = last_location * num_locations + ii;
						for(int iii=0;iii<num_location_intervals;iii++)
						{
							for(int iiii=0;iiii<num_sector_intervals;iiii++)
							{
								sector[cc][iii][iiii].max = sector_mem[cc][iii][iiii].max;
								sector[cc][iii][iiii].min = sector_mem[cc][iii][iiii].min;
							}
						}
					}
				}
			}
			else
			{
				if (learn)
					cout << " (same last location...)";
			}

			last_prediction_flag = -1;
			last_location = pos_vel_acc_avg[i][0].cluster_id;
		}
		cout << endl;
	}
	//********************************************************[GRAPH TEMPORARY]



	cout << "END" << endl;





	return 0;
}
