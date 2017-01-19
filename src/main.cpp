//============================================================================
// Name        : main.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#define SOURCEFILE
//#define DBSCAN

#include "dataDeclaration.h"
#include "util.h"
#include "dbscan.h"
#include "Graph.h"

using namespace std;

unsigned int window = 5;

vector<string> LABEL_MOV;
vector<string> LABEL_LOC_MOV;
vector<string> LABEL_LOC;

//-------------------------------------------------------------------------------------------------

#define Sqr(x) ((x)*(x))
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define Calloc(type,n) (type *)calloc( n, sizeof(type))

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{

	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++)
		color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	double ***sector_constraint;

	sector_t ***sector ;
	sector_t ***sector_mem ;

	point_t *points; 				// cluster
	point_t *points_centroid; 		// modified cluster centroid
	point_t *points_test; 				// test cluster
	point_t *pos_visualize;
	point_t *pos,*vel,*acc;
	point_t *pos_node,*vel_node,*acc_node;
	point_t *pos_vel_acc[3];
	point_t *pos_vel_acc_test[3];
	point_t *location;
	point_t *norm_location;
	point_t *norm_location_normal;

	vector<data_t> motion_data;
	vector<data_t> motionData_train;
	vector<data_t> motion_data_test;
	vector<data_t> motionData2_test;

	vector<vector<string> > obj_data;
	vector<vector<string> > loc_data;
	vector<vector<string> > data_full;
	vector<vector<string> > data_test;

	vector<vector<data_t> > node_data;
	vector<vector<double> > node_data_tmp;

	char *name;

	unsigned int *file_eof;


	int c, c1;
	int i,ii,iii,iiii;

	double *location_boundary;
	double  *distance_location;


	Graph Graph[2];

	double vel_limit 	= 0.01; //##### still need to validate
	double epsilon 		= 0.015; 	// if datasets are merged these 2 values can be increased.

	unsigned int minpts 				= 10;
	unsigned int num_points 			= 0;
	unsigned int num_locations 			= 0;
	unsigned int num_location_intervals	= 20;
	unsigned int num_sector_intervals 	= 36;

	int num_surfaces = 1;
	double **surface;
	surface = Calloc(double*, num_surfaces);
	for(int i=0;i<num_surfaces;i++)
		surface[i] = Calloc(double, 4);

	int obj = 0;
	int num_objs = 3;

	int num_filesi = 0;
	int num_files  = 5;


//[PROGRAM BEGIN]**************************************************************
#ifdef SOURCEFILE
	//[READ OBJ FILE]**********************************************************

	//**********************************************************[READ OBJ FILE]

	//[READ FILE]**************************************************************
	file_eof = new unsigned int[num_files-1];
	c = 0; c1 = 1;
	for(i=num_filesi;i<num_files;i++)
	{
		name = new char[256];
		if ( i == 10 )
		{
			c1++;
			continue;
		}
		if ( i == 4 || i == 9 || i == 14 )
		{
			c1 = 1;
			continue;
		}
		if ( i >= 10 )
		{
			sprintf(name, "../HMAR/3LocationData/traj_data_plyers%d.txt",c1);
			++c1;
		}
		else if ( i >= 5 )
		{
			sprintf(name, "../HMAR/3LocationData/traj_data_clean%d.txt",c1);
			++c1;
		}
		else
		{
			sprintf(name, "../HMAR/3LocationData/traj_data_drink%d.txt",c1);
			++c1;
		}
		readFile( name, data_full , ',');
		file_eof[c] = data_full.size(); ++c;
		delete name;

		//table
		vector<string> last_line = data_full[data_full.size()-1];
		surface[0][0] = atof(last_line[1].c_str());
		surface[0][1] = atof(last_line[2].c_str());
		surface[0][2] = atof(last_line[3].c_str());
		surface[0][3] = atof(last_line[4].c_str());
	}
	//**************************************************************[READ FILE]
	printf("Reading training data......Complete\n");


	//[PARSE DATA]*************************************************************
	num_points = data_full.size();
	points     = Calloc(point_t, num_points);
	parseData2Point(data_full, points , num_points, false);
	//*************************************************************[PARSE DATA]
	printf("Parsing training data......Complete\n");


	//[PREPROCESS DATA]********************************************************
	pos = Calloc(point_t, num_points);
	vel = Calloc(point_t, num_points);
	acc = Calloc(point_t, num_points);
	for(i=0;i<num_points;i++)
	{
		pos[i].cluster_id = UNCLASSIFIED;
		vel[i].cluster_id = UNCLASSIFIED;
		acc[i].cluster_id = UNCLASSIFIED;
	}
	pos_vel_acc[0] = pos;
	pos_vel_acc[1] = vel;
	pos_vel_acc[2] = acc;
	preprocessData( points, pos_vel_acc, num_points, file_eof, window);

	motion_data.resize(num_points);
	for(i=0;i<num_points;i++)
	{
		motion_data[i].pos = pos[i];
		motion_data[i].vel = vel[i];
		motion_data[i].acc = acc[i];
	}
	//********************************************************[PREPROCESS DATA]
	printf("Preprocessing data......Complete\n");
#endif


//[LEARNING]*******************************************************************

//	//[CLUSTERING]*************************************************************
//	dbscanCluster(epsilon, minpts, num_points, pos);
//	printf("Clustering training data......Complete\n");
//	points_centroid = combineNearCluster(pos, num_points, num_locations);
//	printf("Combining nearby clusters......Complete\n");
//	location = Calloc(point_t, num_locations);
//	for(i=0;i<num_locations;i++)
//	{
//		location[i].x = points_centroid[i].x;
//		location[i].y = points_centroid[i].y;
//		location[i].z = points_centroid[i].z;
//		location[i].cluster_id = points_centroid[i].cluster_id;
//	}
//
//	location_boundary = new double[num_locations];
//	contactBoundary(pos, location, location_boundary,
//					num_points, num_locations, true);
//	contactBoundary(pos, location, location_boundary,
//					num_points, num_locations, false);
//	//*************************************************************[CLUSTERING]
	printf("Learning boundary of clusters (action locations)......Complete\n");


	//[LABELLING MOVEMENT]*****************************************************
	int mov_tmp = 2;
	LABEL_MOV.resize(mov_tmp);

	name = new char[256];
	sprintf(name, "./Object/obj_mov_data.txt");
	readFile(name, obj_data , ',');

	if(obj_data.empty())
	{
		LABEL_MOV[0] = {"MOVE"};
		LABEL_MOV[1] = {"SLIDE"};
		writeObjLabelFile(LABEL_MOV, obj);
	}
	else
	{
		bool flag = false;
		for(i=0;i<num_objs;i++)
		{
			if(atoi(obj_data[i][0].c_str())==obj)
			{
				for(ii=1;ii<obj_data[i].size();ii++)
					LABEL_MOV[ii-1] = obj_data[i][ii];
				flag = true;
			}
		}
		if(!flag)
		{
			LABEL_MOV[0] = {"MOVE"};
			LABEL_MOV[1] = {"SLIDE"};
			writeObjLabelFile(LABEL_MOV, obj);
		}
	}
	printf("Reviewing movement labels for OBJ...\n");
	for(i=0;i<mov_tmp;i++) printf("MLabel %d : %s\n", i, LABEL_MOV[i].c_str());
	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(i=0;i<mov_tmp;i++)
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
			rewriteFileObj(name,obj,LABEL_MOV,',');
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
	delete name;
	//*****************************************************[LABELLING MOVEMENT]

	//[LABELLING LOCATION]*****************************************************
	name = new char[256];
	sprintf(name, "./Location/loc_data.txt");
	readFile(name, loc_data , ',');
	bool flag = false;
	if(loc_data.empty())
	{
		//[CLUSTERING]*********************************************************
		dbscanCluster(epsilon, minpts, num_points, pos);
		printf("Clustering training data......Complete\n");
		points_centroid = combineNearCluster(pos, num_points, num_locations);
		printf("Combining nearby clusters......Complete\n");
		location = Calloc(point_t, num_locations);
		for(i=0;i<num_locations;i++)
		{
			location[i].x = points_centroid[i].x;
			location[i].y = points_centroid[i].y;
			location[i].z = points_centroid[i].z;
			location[i].cluster_id = points_centroid[i].cluster_id;
		}

		location_boundary = new double[num_locations];
		contactBoundary(pos, location, location_boundary,
						num_points, num_locations, true);
		contactBoundary(pos, location, location_boundary,
						num_points, num_locations, false);
		//*********************************************************[CLUSTERING]
		LABEL_LOC.resize(num_locations + 1);
		LABEL_LOC[0] = {"CONNECTION"};
		showData(pos, num_points, LABEL_LOC, color_code, true, true);
		writeLocLabelFile(LABEL_LOC, obj, location, num_locations);
	}
	else
	{
		bool flag = false;
		for(i = 0;i<loc_data.size();i++)
		{
			if (atoi(loc_data[i][0].c_str())==obj)
			{
				num_locations = (loc_data[i].size()-1)/4;
				location_boundary = new double[num_locations];
				location = Calloc(point_t,num_locations);
				LABEL_LOC.resize(num_locations + 1);
				LABEL_LOC[0] = {"CONNECTION"};
				for(int ii=0;ii<num_locations;ii++)
				{
					LABEL_LOC[ii+1] = loc_data[i][ii*4+1];
					location[ii].x = atof(loc_data[i][ii*4+2].c_str());
					location[ii].y = atof(loc_data[i][ii*4+3].c_str());
					location[ii].z = atof(loc_data[i][ii*4+4].c_str());
					location[ii].cluster_id = UNCLASSIFIED;
				}
				flag = true;
				contactBoundary(pos, location, location_boundary,
								num_points, num_locations, false);
				showData(pos, num_points, LABEL_LOC, color_code, true, false);
			}
		}
		if(!flag)
		{
			//[CLUSTERING]*****************************************************
			dbscanCluster(epsilon, minpts, num_points, pos);
			printf("Clustering training data......Complete\n");
			points_centroid = combineNearCluster(pos, num_points, num_locations);
			printf("Combining nearby clusters......Complete\n");
			location = Calloc(point_t, num_locations);
			for(i=0;i<num_locations;i++)
			{
				location[i].x = points_centroid[i].x;
				location[i].y = points_centroid[i].y;
				location[i].z = points_centroid[i].z;
				location[i].cluster_id = points_centroid[i].cluster_id;
			}
			location_boundary = new double[num_locations];
			contactBoundary(pos, location, location_boundary,
							num_points, num_locations, true);
			contactBoundary(pos, location, location_boundary,
							num_points, num_locations, false);
			//*****************************************************[CLUSTERING]
			LABEL_LOC.resize(num_locations + 1);
			LABEL_LOC[0] = {"CONNECTION"};
			showData(pos, num_points, LABEL_LOC, color_code, true, true);
			writeLocLabelFile(LABEL_LOC, obj, location, num_locations);
		}
	}

	printf("Reviewing location labels...\n");
	for(i=0;i<num_locations;i++)
	{
		printf("LLabel %d : %s\n", i+1, LABEL_LOC[i+1].c_str());
	}
	printf("Replace default labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(i=0;i<num_locations;i++)
			{
				printf("Enter new label for LLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
					cout << "[WARNING] : Label has not been overwritten."<< endl;
				else
				{
					cout << "[WARNING] : Label has been overwritten. New label : " << mystr << endl;
					LABEL_LOC[i+1] = mystr;
				}
			}
			rewriteFileLoc(name,location,num_locations,obj,LABEL_LOC,',');
			break;
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
	delete name;
	//**************************************************************[LABELLING]
	printf("Labeling of clusters (action locations)......Complete\n");


	//[GRAPH]******************************************************************
	//[ADD NODES]**************************************************************
	// Collecting data based on locations/nodes
	node_data.resize(num_locations);
	for(i=0;i<num_points;i++)
		if (pos[i].cluster_id >= 0)
			node_data[pos[i].cluster_id].push_back(motion_data[i]);

	// Adding nodes to the graph
	for(i=0;i<num_locations;i++)
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
			Graph[0].addNode(LABEL_LOC[i],node_data[i],1,surface_num);
		else
			Graph[0].addNode(LABEL_LOC[i],node_data[i],0,surface_num);
	}
	//**************************************************************[ADD NODES]
	//[VISUALIZE NODES]********************************************************
	node_data_tmp = Graph[0].getNodeDataLabel(true,false,false);

	pos_node = Calloc(point_t,node_data_tmp.size());
	vel_node = Calloc(point_t,node_data_tmp.size());
	acc_node = Calloc(point_t,node_data_tmp.size());
	for(i=0; i<node_data_tmp.size();i++)
	{
		vector<double> tmp_data = node_data_tmp[i];
		c = 0;
		pos_node[i].x = tmp_data[c]; ++c;
		pos_node[i].y = tmp_data[c]; ++c;
		pos_node[i].z = tmp_data[c]; ++c;
		pos_node[i].cluster_id = tmp_data[c]; ++c;
		vel_node[i].x = tmp_data[c]; ++c;
		vel_node[i].y = tmp_data[c]; ++c;
		vel_node[i].z = tmp_data[c]; ++c;
		vel_node[i].cluster_id = tmp_data[c]; ++c;
		acc_node[i].x = tmp_data[c]; ++c;
		acc_node[i].y = tmp_data[c]; ++c;
		acc_node[i].z = tmp_data[c]; ++c;
		acc_node[i].cluster_id = tmp_data[c]; ++c;
	}
	//showData(pos_node, node_data_tmp.size(), LABEL_LOC, true, false);
	//********************************************************[VISUALIZE NODES]
	printf("Creating nodes for clusters (action locations)......Complete\n");


	//[EDGES]******************************************************************
	// Initialize sector
	sector            = Calloc(sector_t**, Sqr(num_locations));
	sector_constraint = Calloc(double**,   Sqr(num_locations));
	for(i=0;i<Sqr(num_locations);i++)
	{
		sector[i]            = Calloc(sector_t*,num_location_intervals);
		sector_constraint[i] = Calloc(double*,  num_location_intervals);
		for(ii=0;ii<num_location_intervals;ii++)
		{
			sector[i][ii]            = Calloc(sector_t,num_sector_intervals);
			sector_constraint[i][ii] = Calloc(double,  num_sector_intervals);
			for(iii=0;iii<num_sector_intervals;iii++)
			{
				sector[i][ii][iii].max = 0;
				sector[i][ii][iii].min = INFINITY;
			}
		}
	}

	norm_location        = Calloc(point_t, Sqr(num_locations));
	norm_location_normal = Calloc(point_t, Sqr(num_locations));
	distance_location    = Calloc(double , Sqr(num_locations));

	for(i=0;i<Sqr(num_locations);i++)
	{
		norm_location[i].cluster_id        = UNCLASSIFIED;
		norm_location_normal[i].cluster_id = UNCLASSIFIED;
	}

	// prepare the vectors from locations
	prepareSector(norm_location, norm_location_normal, distance_location,
			      location, num_locations);

	// THE use of gaussian kernel helps to smoothen can create a tube like structure.
	// However it is still possible to have like bumps because the sampling is just not enough.
	int kernel_size = 5;
	vector<vector<double> > kernel(kernel_size);
	for(int i=0;i<kernel_size;i++) kernel[i].resize(kernel_size);
	gaussKernel(kernel, kernel_size, kernel_size, 1.0);

	generateSector(sector,
			       norm_location, norm_location_normal, distance_location,
			       pos, num_points,
			       location, num_locations,
			       num_location_intervals, num_sector_intervals, file_eof, kernel);

	checkSectorConstraint(sector, sector_constraint, num_locations,
						  num_location_intervals, num_sector_intervals);

	showConnection(sector, sector_constraint,
				   norm_location, norm_location_normal, distance_location,
				   location, num_locations,
				   num_location_intervals, num_sector_intervals, color_code);
	//******************************************************************[EDGES]
	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");

	//******************************************************************[GRAPH]
	printf("Creating a graph to represent the clusters (action locations)......Complete\n");

//*******************************************************************[LEARNING]

//[TESTING]********************************************************************
	//Read file will be replaced later on
	//[READ FILE]**************************************************************
	{
		int i = obj;
		delete[]file_eof;
		file_eof = new unsigned int[1];
		//for(i=1;i<num_files;i++)
		{
			name = new char[256];
			if ( i == 1 )
				sprintf(name, "../HMAR/3LocationData/traj_data_clean%d.txt",5);
			else if ( i == 2 )
				sprintf(name, "../HMAR/3LocationData/traj_data_plyers%d.txt",5);
			else
				sprintf(name, "../HMAR/3LocationData/traj_data_drink%d.txt",5);
			readFile( name, data_test , ',');
			file_eof[i] = data_test.size();
			delete name;

			//table
			vector<string> last_line = data_test[data_test.size()-1];
			surface[0][0] = atof(last_line[1].c_str());
			surface[0][1] = atof(last_line[2].c_str());
			surface[0][2] = atof(last_line[3].c_str());
			surface[0][3] = atof(last_line[4].c_str());
		}
	}
	//**************************************************************[READ FILE]
	printf("Reading test data......Complete\n");


	//[PARSE DATA]*************************************************************
	num_points = data_test.size();
	points_test = Calloc(point_t, num_points);
	parseData2Point(data_test, points_test, num_points, false);
	//*************************************************************[PARSE DATA]
	printf("Parsing test data......Complete\n");


	//[PREPROCESSING DATA]*****************************************************
	point_t *pos_test = Calloc(point_t, num_points);
	point_t *vel_test = Calloc(point_t, num_points);
	point_t *acc_test = Calloc(point_t, num_points);
	pos_vel_acc_test[0] = pos_test;
	pos_vel_acc_test[1] = vel_test;
	pos_vel_acc_test[2] = acc_test;
	preprocessData(points_test, pos_vel_acc_test, num_points, file_eof, window);
	motion_data_test.resize(num_points);
	for (int i=0;i<num_points;i++)
	{
		motion_data_test[i].pos = pos_test[i];
		motion_data_test[i].vel = vel_test[i];
		motion_data_test[i].acc = acc_test[i];
	}
	//*****************************************************[PREPROCESSING DATA]
	printf("Preprocessing data......Complete\n");

	//[GRAPH TEMPORARY]********************************************************
	//[ADD NODES]**************************************************************
	// Initialize sector backup
	sector_mem = Calloc(sector_t**, Sqr(num_locations));
	for(i=0;i<Sqr(num_locations);i++)
	{
		sector_mem[i] = Calloc(sector_t*,num_location_intervals);
		for(ii=0;ii<num_location_intervals;ii++)
		{
			sector_mem[i][ii] = Calloc(sector_t,num_sector_intervals);
			for(iii=0;iii<num_sector_intervals;iii++)
			{
				sector_mem[i][ii][iii].max = sector[i][ii][iii].max;
				sector_mem[i][ii][iii].min = sector[i][ii][iii].min;
			}
		}
	}

	bool flag_predict = false;
	double prediction1[num_locations];
	double prediction2[num_locations];
	double *prediction1_;
	prediction1_ = Calloc(double,num_locations);

	double prediction_curr[num_locations];
	//double prediction_last[num_locations];
	int last_prediction_flag = -1;
	double cc = 1;

	double prediction[num_locations];
	double t_val[num_locations];
	vector<data_t> tmp_data;
	bool flag_learned = false;
	bool slide = false;
	int last_location = 0;
	int surface_num_tmp = 0;
	for(i=0;i<num_points;i++)
	{
		slide = false;

		decideBoundary(pos_test[i], location, location_boundary, num_locations);

		cout << i << " " << pos_test[i].cluster_id << " ";

//		for(ii=0;ii<num_surfaces;ii++)
//			cout << checkSurfaceRange(pos_test[i], pos_surface[ii], surface, 0.1, 0.02);

		if (pos_test[i].cluster_id < 0)
		{
			flag_learned = true;

			checkSector(sector, prediction, t_val,
						pos_test[i],
						location, num_locations,
						norm_location, norm_location_normal,
						distance_location,
						num_location_intervals, num_sector_intervals,
						last_location, flag_learned);

			if (l2Norm(vel_test[i])> vel_limit)
			{
				for(ii=0;ii<num_surfaces;ii++)
				{
					if (!slide)
					{
						slide =	checkMoveSlide(pos_test[i], vel_test[i], surface[ii], 0.1, 0.97); //####needto add
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

//			for(int ii=0;ii<num_locations;ii++)
//			{
//				if((int)prediction[ii]==1)
//					printf(" %d %.4f ", (int)prediction[ii], prediction1[ii]);
//				else
//					printf(" %d %.4f ", (int)prediction[ii], prediction2[ii]);
//			}

			for(int ii=0;ii<num_locations;ii++)
				printf(" %d %.4f ", (int)prediction[ii], prediction2[ii]);
			for(int ii=0;ii<num_locations;ii++)
				if((int)prediction[ii]==1)
					printf(" %s %.4f ", LABEL_LOC[ii+1].c_str(), prediction1[ii]);

		}
		else
		{
			flag_predict = false;

			if (!Graph[1].checkNode(pos_test[i].cluster_id))
			{
				tmp_data.push_back(motion_data_test[i]);
				Graph[1].addNode(LABEL_LOC[pos_test[i].cluster_id], tmp_data,
						         Graph[0].getNode(pos_test[i].cluster_id).category,
						         Graph[0].getNode(pos_test[i].cluster_id).surface_num);
				tmp_data.clear();
			}
			else
			{
				tmp_data.push_back(motion_data_test[i]);
				Graph[1].extendNode(tmp_data, pos_test[i].cluster_id);
				tmp_data.clear();
			}

			if (LABEL_LOC[pos_test[i].cluster_id+1].empty())
			{
				cout << " LABEL: "<< "Empty location Label.";
				if (l2Norm(vel_test[i])> vel_limit)
				{
					for(ii=0;ii<num_surfaces;ii++)
					{
						if (!slide)
						{
							slide =	checkMoveSlide(pos_test[i], vel_test[i], surface[ii], 0.1, 0.96); //####needto add
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
				cout << " LABEL: "<< LABEL_LOC[pos_test[i].cluster_id+1];

			// if it is moved back to the same location
			if (last_location != pos_test[i].cluster_id)
			{
				if (flag_learned)
				{
					// copy only the intended values for sectors
					// updating the values in memory
					int c = last_location * num_locations + pos_test[i].cluster_id;
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
					flag_learned = false;
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
				if (flag_learned)
					cout << " (same last location...)";
			}


//			for(int ii=0;ii<num_locations;ii++)
//				prediction_last[ii] = 0.0;

			last_prediction_flag = -1;
			last_location = pos_test[i].cluster_id;
		}
		cout << endl;
	}

//############################################################################################################### EVAL START
//Step 00: 	Object identified, contact detected, start analysis
//Step 01:	Object moving ?
//			Yes : Step 02
//			No 	: Step 05
//Step 02:	Within any known location ?
//			Yes	: Step 03
//			No	: Step 04
//Step 03:	Check category -> 0 : Label = OBJECT** (given at location)
//	 					   -> 1 : surface contact based labeling -> Step 04 + Label = OBJECT** (given at location)
//Step 04:  Transport/sliding ?
//			Transport	: Label = MOVE
//			Sliding		: Label = SLIDE/PUSHING
//Step 05:	Within any known location ?
//			Yes	: Label = OBJECT**
//			No	: Label = STOP

//	bool flag_learned = false;
//	vector<data_t> tmp_data;
//	int last_location = 0;
//	unsigned int output[num_locations];
//	bool slide = false;
//	for(int i=0;i<num_points;i++)
//	{
//		decideBoundary(pos_test[i], location, location_boundary, num_locations);
//
//		cout << i << " " << pos_test[i].cluster_id << " ";
//
//		if (pos_test[i].cluster_id >= 0)
//		{
//			if (!Graph[1].checkNode(pos_test[i].cluster_id))
//			{
//				tmp_data.push_back(motion_data_test[i]);
//				Graph[1].addNode(LABEL[pos_test[i].cluster_id], tmp_data,
//						         Graph[0].getNode(pos_test[i].cluster_id).category,
//						         Graph[0].getNode(pos_test[i].cluster_id).surface_num);
//				tmp_data.clear();
//			}
//			else
//			{
//				tmp_data.push_back(motion_data_test[i]);
//				Graph[1].extendNode(tmp_data, pos_test[i].cluster_id);
//				tmp_data.clear();
//			}
//			if (flag_learned)
//			{
//				// if it is moved back to the same location
//				if (last_location != pos_test[i].cluster_id)
//				{
//					// copy only the intended values for sectors
//					// updating the values in memory
//					int c = last_location * num_locations + pos_test[i].cluster_id;
//					for(int ii=0;ii<num_location_intervals;ii++)
//					{
//						for(int iii=0;iii<num_sector_intervals;iii++)
//						{
//							sector_mem[c][ii][iii].max = sector[c][ii][iii].max;
//							sector_mem[c][ii][iii].min = sector[c][ii][iii].min;
//						}
//					}
//				}
//				else
//					cout << "same last location...";
//
//				// update the sector using values from memory
//				for(int ii=0;ii<num_locations;ii++)
//				{
//					int cc = last_location * num_locations + ii;
//					for(int iii=0;iii<num_location_intervals;iii++)
//					{
//						for(int iiii=0;iiii<num_sector_intervals;iiii++)
//						{
//							sector[cc][iii][iiii].max = sector_mem[cc][iii][iiii].max;
//							sector[cc][iii][iiii].min = sector_mem[cc][iii][iiii].min;
//						}
//					}
//				}
//				flag_learned = false;
//			}
//
//			for(int ii=0;ii<num_locations;ii++)
//			{
//				output[ii] = 0.0;
//			}
//
//			last_location = pos_test[i].cluster_id;
//
//// #########Now that we know the object is in location, we can check whether it is in contact with surface.
////			If not in contact with any surface, then check object for function label.
////			If in contact, check if it is sliding.
////			Vel = 0 means place, moving without sliding means normal movement.
////			Sliding and movement gives more info, to see label have to check object.
//
////			double  tmp1 = checkMoveSlideOutside(pos_test[i], vel_test[i], surface, num_surfaces);
////			cout << l2Norm(vel_test[i]) << " ";
//
//			//category switch
//			switch(Graph[1].getNode(pos_test[i].cluster_id).category)
//			{
//				case 0 : // Check Object Label
//					cout << " Cat 0 ";
//					if (LABEL[pos_test[i].cluster_id+1].empty())
//					{
//						cout << " LABEL: "<< "Empty category 0 Label.";
//						if (l2Norm(vel_test[i])> vel_limit)
//							cout << "...MOVE"<< endl;
//						else
//							cout << "...NULL"<< endl;
//					}
//					else
//						cout << " LABEL: "<< LABEL[pos_test[i].cluster_id+1] << endl;
//					break;
//				case 1 :
//					cout << " Cat 1 ";
//					slide =	checkMoveSlide(pos_test[i],
//							surface[Graph[1].getNode(pos_test[i].cluster_id).surface_num],
//							0.1);
//					if (l2Norm(vel_test[i])> vel_limit)
//					{
//						if (slide)
//						{
//							if (LABEL[pos_test[i].cluster_id+1].empty())
//								cout << " LABEL: "<< "Empty category 1 Label." << endl;
//							else
//								cout << " LABEL: "<< LABEL[pos_test[i].cluster_id+1] << endl;
//							break;
//						}
//						else
//							cout << " LABEL: "<< "MOVE" << endl;
//					}
//					else
//					{
//						cout << " "<< l2Norm(vel_test[i]);
//						cout << " LABEL: "<< "NULL" << endl;
//					}
//					break;
////				case  2 :
////					...
////					break;
//				default:
//					cout << " LABEL: "<< "Unknown category..." << endl;
//					break;
//			}
//		}
//		else
//		{
//			checkSector(sector, output,
//						pos_test[i],
//						location, num_locations,
//						norm_location, norm_location_normal,
//						distance_location,
//						num_location_intervals, num_sector_intervals,
//						last_location, true);
//			flag_learned = true;
//
//			if (l2Norm(vel_test[i])> vel_limit)
//			{
//				if (Graph[1].getNode(last_location).category==1)
//					slide =	checkMoveSlide(pos_test[i],
//							surface[Graph[1].getNode(last_location).surface_num],
//							0.1);
//				else
//					slide = false;
//				if (slide)
//					if (LABEL[pos_test[i].cluster_id+1].empty())
//						cout << " LABEL: "<< "Empty category 1 Label.";
//					else
//						//cout << " LABEL: "<< LABEL[last_location+1];
//						cout << " LABEL OUT: "<< " SLIDE ";
//				else
//					cout << " LABEL OUT: "<< " MOVE ";
//			}
//			else
//			{
//				cout << " "<< l2Norm(vel_test[i]);
//				cout << " LABEL OUT: "<< " NULL ";
//			}
//
//
//			//cout << l2Norm(vel_test[i]) << " ";
//
//			for(int ii=0;ii<num_locations;ii++)
//			{
//
//				switch(output[ii])
//				{
//					case 1 :
//						cout << " Target: "<< ii << " possible...";
//						break;
//					case 2 :
//						cout << " Target: "<< ii << " detected...";
//						break;
//					case 3 :
//						cout << " Target: "<< ii << " learned...";
//						break;
//					default:
//						cout << " Target: "<< ii << " not possible...";
//						break;
//				}
//			}
//			cout << endl;
//		}
//
//
////		printf("OUTPUT : %02d %02d ", i, pos_test[i].cluster_id);
////		printf("%02d %02d %02d %02d\n", output[0], output[1], output[2], output[3]);
//	}
//
//	showConnection(sector,
//		 norm_location, norm_location_normal, distance_location,
//		 location, num_locations,
//		 num_location_intervals, num_sector_intervals);
















//    int argc;
//    char **argv;
//
//    point_t *points; 				// cluster
//    point_t *points_centroid; 		// modified cluster centroid
//    point_t *points_test; 				// test cluster
//    point_t *pos_visualize;
//
//
//
//
//	vector<vector<string> > data_test;
//
//
//	vector<data_t> motionData_train;
//	vector<data_t> motion_data_test;
//	vector<data_t> motionData2_test;
//
//	Graph Graph[2];
//
//	int *label;
//
//
//
//
//
//
//
//	int c,c1,c2,c3; // act as temporary counter
//
//    double epsilon = 0.015; 	// if datasets are merged these 2 values can be increased.
//    unsigned int minpts = 10;
//    unsigned int num_points = 0;
//	unsigned int num_locations = 0;
//	unsigned int num_location_intervals = 10;
//	unsigned int num_sector_intervals = 4;
//
//	double vel_limit = 0.01; //##### still need to validate
//
//	point_t *pos,*vel,*acc;
//	point_t *pos_node,*vel_node,*acc_node;
//	point_t *pos_vel_acc[3];
//
//	vector<data_t> motion_data;
//
//	point_t *location;
//	double *location_boundary;
//
//	int num_surfaces = 1;
//	double **surface;
//	surface = new double*[num_surfaces];
//	for(int i=0;i<num_surfaces;i++)
//		surface[i] = new double[4];
//
//
//





//
//
//
//


//
//// if the tube is lost or goes in new direction,
////	we can check the current direction vector to see
////	which location it is going and check the corresponding sectors
//

////	//**************************************************************[ADD NODES]
//////	//[ADD EDGES]**************************************************************
//////	mem_tmp = -1;
//////	edge_num = 0;
//////	edge_data.clear();
//////	for(int i=0;i<num_points;i++)
//////	{
//////		//decideBoundary(num_locations, pos_test[i], location, location_boundary);
//////		if (i < 1) // first data point is skipped because we need to check for the previous node.
//////			continue;
//////		if (pos_test[i].cluster_id < 0)
//////			if (pos_test[i-1].cluster_id < 0)
//////				edge_data.push_back(motion_data_test[i]);
//////			else
//////				mem_tmp = pos_test[i-1].cluster_id;
//////		else
//////			if (pos_test[i-1].cluster_id < 0)
//////				// check which node is assigned an edge, and check which node the edge is linked to
//////				if (!Graph[1].checkEdge((unsigned int)pos_test[i].cluster_id,
//////										(unsigned int)mem_tmp , edge_num))
//////				{
//////					// adding a new edge
//////					Graph[1].addEdge((unsigned int)pos_test[i].cluster_id,
//////									 (unsigned int)mem_tmp, edge_data);
//////					edge_data.clear();
//////				}
//////				else
//////				{
//////					// extending the edge list for each node
//////					Graph[1].extendEdge((unsigned int)pos_test[i].cluster_id,
//////										(unsigned int)mem_tmp, edge_data, edge_num);
//////					edge_data.clear();
//////				}
//////			else
//////				continue;
//////	}
//////	//**************************************************************[ADD EDGES]
////	//[VISUALIZE NODES]********************************************************
////	node_data_tmp = Graph[1].getNodeDataLabel(true,false,false);
////	free(pos_test); free(vel_test); free(acc_test);
////	pos_test = (point_t *)calloc(node_data_tmp.size(), sizeof(point_t));
////	vel_test = (point_t *)calloc(node_data_tmp.size(), sizeof(point_t));
////	acc_test = (point_t *)calloc(node_data_tmp.size(), sizeof(point_t));
////	for(int i=0; i<node_data_tmp.size();i++)
////	{
////		vector<double> tmp_data = node_data_tmp[i];
////		c = 0;
////		pos_test[i].cluster_id = tmp_data[c]; ++c;
////		pos_test[i].x = tmp_data[c]; ++c;
////		pos_test[i].y = tmp_data[c]; ++c;
////		pos_test[i].z = tmp_data[c]; ++c;
////		vel_test[i].cluster_id = tmp_data[c]; ++c;
////		vel_test[i].x = tmp_data[c]; ++c;
////		vel_test[i].y = tmp_data[c]; ++c;
////		vel_test[i].z = tmp_data[c]; ++c;
////		acc_test[i].cluster_id = tmp_data[c]; ++c;
////		acc_test[i].x = tmp_data[c]; ++c;
////		acc_test[i].y = tmp_data[c]; ++c;
////		acc_test[i].z = tmp_data[c]; ++c;
////	}
//////	showData(pos_test, node_data_tmp.size(), LABEL, true, false);
////	//********************************************************[VISUALIZE NODES]
//////	//[VISUALIZE EDGES]********************************************************
//////	edge_data_tmp = Graph[1].getEdgeDataLabel(true,true,true);
//////	free(pos_test); free(vel_test); free(acc_test);
//////	pos_test = (point_t *)calloc(edge_data_tmp.size(), sizeof(point_t));
//////	vel_test = (point_t *)calloc(edge_data_tmp.size(), sizeof(point_t));
//////	acc_test = (point_t *)calloc(edge_data_tmp.size(), sizeof(point_t));
//////	for(int i=0; i<edge_data_tmp.size();i++)
//////	{
//////		vector<double> tmp_data = edge_data_tmp[i];
//////		c = 0;
//////		pos_test[i].cluster_id = tmp_data[c]; ++c;
//////		pos_test[i].x = tmp_data[c]; ++c;
//////		pos_test[i].y = tmp_data[c]; ++c;
//////		pos_test[i].z = tmp_data[c]; ++c;
//////		vel_test[i].cluster_id = tmp_data[c]; ++c;
//////		vel_test[i].x = tmp_data[c]; ++c;
//////		vel_test[i].y = tmp_data[c]; ++c;
//////		vel_test[i].z = tmp_data[c]; ++c;
//////		acc_test[i].cluster_id = tmp_data[c]; ++c;
//////		acc_test[i].x = tmp_data[c]; ++c;
//////		acc_test[i].y = tmp_data[c]; ++c;
//////		acc_test[i].z = tmp_data[c]; ++c;
//////	}
//////	showData(pos_test, edge_data_tmp.size(), true, false);
//////	motionData2_test.resize(edge_data_tmp.size());
//////	delete[]label;
//////	label = new int[edge_data_tmp.size()];
//////	for(int i=0;i<edge_data_tmp.size();i++)
//////	{
//////		label[i] = pos_test[i].cluster_id;
//////		motionData2_test[i].pos = point2point3D(pos_test[i]);
//////		motionData2_test[i].vel = point2point3D(vel_test[i]);
//////		motionData2_test[i].acc = point2point3D(acc_test[i]);
//////	}
//////	//********************************************************[VISUALIZE EDGES]
////	//********************************************************[GRAPH TEMPORARY]
////	printf("Creating a graph to represent the clusters (action locations)......Complete\n");
////
////
////
////
//////********************************************************************[TESTING]*/
////
////
////
////
////
////
////
////
////
////
////
////
//////	//[CLASSIFIER TRAINNING]***************************************************
//////	num_points = edge_data_tmp.size();
//////	classifierSVM( motionData_train, label, num_points, location, num_locations, true );
//////    argc = 11;
//////    char* n_argv[] = {"svm-train", "-t", "0", "-b", "1", "-c", "0.5", "-g", "0.0078125","data/data_svm.svm", "model"};
//////    argv = n_argv;
//////    trainSVM(argc,argv);
//////    //***************************************************[CLASSIFIER TRAINNING]
//////	printf("Learning classifier from training data......Complete\n");
//////
//////
//////
//////
//////
//////
//////
//////
//////
////
////
//////	//[CLASSIFIER TESTING]*****************************************************
//////	num_points = edge_data_tmp.size();
//////	classifierSVM( motionData2_test, label, num_points, location, num_locations, false );
//////    argc = 6;
//////    char* n_argv2[] = {"svm-predict", "-b", "1", "data/data_svm2.svm", "model", "predict"};
//////    argv = n_argv2;
//////    predictSVM( argc, argv );
//////    //*****************************************************[CLASSIFIER TESTING]
//////	printf("Testing classifier using test data......Complete\n");
//////
//////	/*
//////	vector<vector<string> > data_predict;
//////	char *name = new char[256];
//////	sprintf(name, "predict");
//////	readFile( name, data_predict , ' ');
//////	delete name;
//////
//////	num_points = data_predict.size();
//////
//////	parseData2Point( num_points, data_predict, points_test , true);
//////
//////	//showData(points_test, num_points, true, false);
//////
//////	printf("Prediction from test data......Complete\n");
//////
//////	/*
////////	free(p1);
////////	free(p4);
////////	free(p5);
//////*/


	cout << "END" << endl;

	//**********************************************************[Program Begin]
	return 0;
}




