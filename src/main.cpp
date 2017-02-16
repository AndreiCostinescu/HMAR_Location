//============================================================================
// Name        : main.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#define SAVEDDATA
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
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
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

	int obj = 3;
	int num_objs = 4;

	int num_filesi = 0;
	int num_files  = 5;


//[PROGRAM BEGIN]**************************************************************

#ifdef SAVEDDATA
	//[READ OBJ FILE]**********************************************************
	//**********************************************************[READ OBJ FILE]

	//[READ FILE]**************************************************************
	string dir_name = "../../KINECT/data/";
	file_eof = new unsigned int[100];
	c = 0;

	DIR *dir;
	struct dirent *dent;
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
			file_eof[c] = data_full.size(); c++;
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

	//[LABELLING MOVEMENT]*****************************************************
	int mov_tmp = 2;
	LABEL_MOV.resize(mov_tmp);

	name = new char[256];
	sprintf(name, "../Object/obj_mov_data.txt");
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
			if(obj_data.size() < num_objs) continue;
			if(atoi(obj_data[i][0].c_str())==obj)
			{
				for(ii=0;ii<obj_data[i].size()-1;ii++)
					LABEL_MOV[ii] = obj_data[i][ii+1];
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
	while(true)
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
	sprintf(name, "../Location/loc_data.txt");
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
	while(true)
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
			       num_location_intervals, num_sector_intervals,
			       file_eof, kernel);

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
		
	dir_name = "../../KINECT/data/test/";
	delete[]file_eof;
	file_eof = new unsigned int[1];

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
			file_eof[0] = data_test.size();
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
	bool learn = false;
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
			learn = true;

			checkSector(sector, prediction, t_val,
						pos_test[i],
						location, num_locations,
						norm_location, norm_location_normal,
						distance_location,
						num_location_intervals, num_sector_intervals,
						last_location, learn);

			if (l2Norm(vel_test[i])> vel_limit)
			{
				for(ii=0;ii<num_surfaces;ii++)
				{
					if (!slide)
					{
						slide =	checkMoveSlide(pos_test[i], vel_test[i], surface[ii], 0.1, 0.97); //####need to add
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
				//printf(" %d %.4f ", (int)prediction[ii], prediction2[ii]);
				printf(" %.4f ", prediction2[ii]);
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
				if (learn)
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


//			for(int ii=0;ii<num_locations;ii++)
//				prediction_last[ii] = 0.0;

			last_prediction_flag = -1;
			last_location = pos_test[i].cluster_id;
		}
		cout << endl;
	}


	cout << "END" << endl;

	//**********************************************************[Program Begin]
	return 0;
}




