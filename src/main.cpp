//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

//#define LEARN_LOCATION
//#define LEARN
#define TESTING

#include "dataDeclaration.h"
#include "algo.h"
#include "util.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"

#ifdef PC
	string DATA_DIR_SINGLE 	= "../../KINECT/recording/";
	string DATA_DIR_ALL		= "../../KINECT/recording/";
	string TEST_DIR_ 		= "../../KINECT/recording/";
	string SCENE_ 			= "../Scene/";
#else
	string DATA_DIR_SINGLE_ = "../KINECT/recording/";
	string DATA_DIR_ALL_	= "../KINECT/recording/";
	string TEST_DIR_ 		= "../KINECT/recording/";
	string SCENE_ 			= "Scene/";
#endif

	string scene   			= "Kitchen";

#ifdef LEARN_LOCATION
	string object  			= "ALL";
	string DATA_DIR_ALL 	= DATA_DIR_ALL_ 	+ object	+ string("/");
#else
	string object  			= "04";
	string DATA_DIR_SINGLE 	= DATA_DIR_SINGLE_	+ object	+ string("/");
	string TEST_DIR 		= TEST_DIR_ 		+ object 	+ string("/test/");
#endif

//=============================================================================
// Global
//=============================================================================

stack<clock_t> tictoc_stack;
void tic() {tictoc_stack.push(clock());}
void toc()
{
	cout << "Time elapsed: "
       << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
       << std::endl;
	tictoc_stack.pop();
}

//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{

#ifdef LEARN_LOCATION

	// [PARAMETERS]************************************************************
	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;
	// if more data points are available these 2 values can be increased.
	int minpts 					= 20;
	double epsilon 				= 0.015;
	// ************************************************************[PARAMETERS]

	// [VARIABLES]*************************************************************
	int num_points 				= 0;
	int num_locations			= 0;
	int file_num 				= 0;
	vector<int> 				file_eof;
	vector<point_t> 			points;
	vector<vector<string> > 	data_full;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<vector<point_t> > 	pos_vel_acc_mem; // motion->length
	vector<unsigned char*> color_code(24);
	for(int j=0;j<24;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);
	Graph Graph(scene, object);
	printf("Initialization......Complete\n");
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	struct dirent **namelist;
	string name;
	string dir_name;
	dir_name = DATA_DIR_ALL;
	int n = scandir(dir_name.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dir_name + namelist[i]->d_name;
		readFile(name.c_str(), data_full , ',');
		file_eof.push_back(data_full.size());
	}
	readSurfaceFile(Graph);
	printf("Reading training data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_full.size();
	reshapeVector(points, num_points);
	parseData2Point(data_full, points);
	printf("Parsing training data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [PREPROCESS DATA]*******************************************************
	reshapeVector(pos_vel_acc_avg, num_points);
	reshapeVector(pos_vel_acc_mem, 3);
	for(int i=0;i<num_points;i++)
	{
		if (i == file_eof[file_num])
		{
			file_num++;
			reshapeVector(pos_vel_acc_mem, 3);
		}
		if (i == 0)	{ pos_vel_acc_avg[i] = pos_vel_acc_avg1; }
		else		{ pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1]; }
		preprocessDataLive(
				points[i], pos_vel_acc_mem, pos_vel_acc_avg[i], FILTER_WIN);
	}
	printf("Pre-processing data......Complete\n");
	// *******************************************************[PREPROCESS DATA]

	// [LABELLING MOVEMENT]****************************************************
	labelMovement(Graph);
	printf("Labeling of movements......Complete\n");
	// ****************************************************[LABELLING MOVEMENT]

	// [LABELLING LOCATION]****************************************************
	labelLocation_(Graph, pos_vel_acc_avg, epsilon, minpts);
	printf("Labeling of location areas......Complete\n");
	// ****************************************************[LABELLING LOCATION]

#endif


#ifdef LEARN

	// [PARAMETERS]************************************************************
	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;
	// if more data points are available these 2 values can be increased.
	int minpts 					= 20;
	double epsilon 				= 0.015;
	double max_range 			= 0.05;
	// ************************************************************[PARAMETERS]

	// [VARIABLES]*************************************************************
	int num_points 				= 0;
	int num_locations			= 0;
	int file_num 				= 0;
	vector<int> 				file_eof;
	vector<point_t> 			points;
	vector<vector<string> > 	data_full;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<vector<point_t> > 	pos_vel_acc_mem; // motion->length
	vector<unsigned char*> color_code(24);
	for(int j=0;j<24;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);
	Graph Graph(scene, object);
	printf("Initialization......Complete\n");
	// *************************************************************[VARIABLES]

	// [LEARNED DATA]**********************************************************
	directoryCheck(SCENE_ + scene + "/" + object);
	string path_tmp_src, path_tmp_dest;
	path_tmp_src 	= SCENE_ + scene + string("/ALL/data_loc.txt");
	path_tmp_dest 	= SCENE_ + scene + "/" + object + "/data_loc.txt";
	copyFile(path_tmp_src, path_tmp_dest);
	path_tmp_src 	= SCENE_ + scene + string("/ALL/data_mov.txt");
	path_tmp_dest 	= SCENE_ + scene + "/" + object + "/data_mov.txt";
	copyFile(path_tmp_src, path_tmp_dest);
	printf("Copying learned data......Complete\n");
	// **********************************************************[LEARNED DATA]

	// [READ FILE]*************************************************************
	struct dirent **namelist;
	string name;
	string dir_name;
	dir_name = DATA_DIR_SINGLE;
	int n = scandir(dir_name.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dir_name + namelist[i]->d_name;
		readFile(name.c_str(), data_full , ',');
		file_eof.push_back(data_full.size());
		vector<string> last_line = data_full[data_full.size()-1];
	}
	readSurfaceFile(Graph);
	printf("Reading training data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_full.size();
	reshapeVector(points, num_points);
	parseData2Point(data_full, points);
	printf("Parsing training data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [PREPROCESS DATA]*******************************************************
	reshapeVector(pos_vel_acc_avg, num_points);
	reshapeVector(pos_vel_acc_mem, 3);
	for(int i=0;i<num_points;i++)
	{
		if (i == file_eof[file_num])
		{
			file_num++;
			reshapeVector(pos_vel_acc_mem, 3);
		}
		if (i == 0)
			pos_vel_acc_avg[i] = pos_vel_acc_avg1;
		else
			pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1];
		preprocessDataLive(points[i], pos_vel_acc_mem, pos_vel_acc_avg[i],
						   FILTER_WIN);
	}
	printf("Pre-processing data......Complete\n");
	// *******************************************************[PREPROCESS DATA]

	// [NODES]*****************************************************************
	readLocation_(Graph);
	readMovement (Graph);
	labelLocation_(Graph, pos_vel_acc_avg, epsilon, minpts);
	printf("Creating nodes for the clusters (action locations)......Complete\n");
	// *****************************************************************[NODES]

	//[EDGES]******************************************************************
	Graph.initEdge(num_location_intervals, num_sector_intervals);
	labelSector(Graph, pos_vel_acc_avg,	max_range, file_eof, color_code);
	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");
	// *****************************************************************[EDGES]

#endif

#ifdef TESTING


// [TESTING]*******************************************************************

	// [VARIABLES]*************************************************************
	vector<unsigned char*> color_code(24);
	for(int j=0;j<24;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	Graph Graph_(scene, object);

	int num_points 			= 0;
	int num_locations		= 0;
	int num_surfaces		= 0;
	int file_num 			= 0;
	int minpts 				= 10;
	double epsilon 			= 0.015; 	// if datasets are merged these 2 values can be increased.

	limit_t LIMIT;
	LIMIT.vel 				= 0.005;
	LIMIT.sur_dis 			= 0.075;
	LIMIT.sur_ang 			= 0.95;

	vector<int> 				file_eof;
	vector<point_t> 			points_test;
	vector<vector<string> > 	data_test;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion

	bool flag_motion      	= false;
	bool flag_predict      	= false;
	bool flag_predict_last 	= false;
	bool learn 				= false;
	bool slide 				= false;
	int loc_last 			= 0;
	int surface_num_tmp 	= 0;
	double pow_dec 			= 1;

	pred_t prediction;

	label_t LABEL;

	msg_t MSG;

	vector<int> 				loc_last_idxs;
	vector<double> 				t_val;
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<point_t > 		  	pos_vel_acc_mem1(3);
	vector<vector< point_t > > 	pos_vel_acc_mem; // motion->length
	vector<vector< point_t > > 	pva_avg;

	// JUST FOR VISUALIZING
	vector<point_t> p_data;
	vector<string> label_data; label_data.push_back(string("CONNECTION"));
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	struct dirent **namelist;
	string name, dir_name;
	dir_name = TEST_DIR;
	int n = scandir(dir_name.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dir_name + namelist[i]->d_name;
		readFile(name.c_str(), data_test , ',');
		file_eof.push_back(data_test.size());
	}
	readSurfaceFile(Graph_);
	printf("Reading test data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_test.size();
	reshapeVector(points_test, num_points);
	parseData2Point(data_test, points_test);
	printf("Parsing test data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [LEARNED DATA]**********************************************************
	readLocation_   (Graph_   );
	readMovement    (Graph_   );
	readSectorFile  (Graph_, 0);
	readSectorFile  (Graph_, 1);
	readLocationFile(Graph_, 0);
	readLocationFile(Graph_, 1);
	readLocationFile(Graph_, 2);
	readLocationFile(Graph_, 3);
	readLocationFile(Graph_, 4);
	readCounterFile (Graph_, 0);
	num_locations = Graph_.getNodeList().size();
	num_surfaces  = Graph_.getSurface ().size();

	for(int i=0;i<num_locations;i++)
		label_data.push_back(Graph_.getNode(i).name);
	// **********************************************************[LEARNED DATA]

	// [PREDICTION VARIABLES]**************************************************
	reshapeVector(prediction.pred, 			num_locations);
	reshapeVector(prediction.pred_in,		num_locations);
	reshapeVector(prediction.pred_in_last, 	num_locations);
	reshapeVector(prediction.pred_err,		num_locations);
	reshapeVector(pos_vel_acc_avg, 			num_points);
	reshapeVector(pos_vel_acc_mem, 			3);
	reshapeVector(loc_last_idxs, 			num_locations);
	reshapeVector(LABEL.loc, 				num_locations);
	reshapeVector(LABEL.sur, 				num_surfaces);
	LABEL.mov   = -1;
	MSG.num_loc = num_locations;
	MSG.num_sur = num_surfaces;
	// **************************************************[PREDICTION VARIABLES]

	printf("\n\n>>>>> SYSTEM START <<<<<\n\n");

	for(int i=0;i<num_points;i++)
	{
		//[PREPROCESS DATA]****************************************************
		if(i == file_eof[file_num])
		{
			file_num++;
			pos_vel_acc_mem.clear();
			for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
		}

		if (i>0) 	pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1];
		else 		pos_vel_acc_avg[i] = pos_vel_acc_avg1;

		preprocessDataLive(points_test[i], pos_vel_acc_mem, pos_vel_acc_avg[i],
						   FILTER_WIN);
		//****************************************************[PREPROCESS DATA]

// ============================================================================
// PREDICTION STARTS
// ============================================================================
		slide 	  = false;
		LABEL.mov = -1;
		MSG.idx   =  i;
		reshapeVector(LABEL.loc, num_locations);
		reshapeVector(LABEL.sur, num_surfaces);

		// 1. Contact trigger
		// 1.1 Check if the object is within a sphere volume of the location areas
		triggerContact(pos_vel_acc_avg[i][0], Graph_);
		p_data.push_back(pos_vel_acc_avg[i][0]);

		// 2. Prediction during motion
		if (pos_vel_acc_avg[i][0].cluster_id < 0)
		{
			pva_avg.push_back(pos_vel_acc_avg[i]);
			flag_motion = true;

			predictionEdge(prediction, Graph_,
					pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1],
					loc_last, loc_last_idxs, LIMIT, LABEL,
					flag_predict, flag_predict_last, pow_dec);

			MSG.msg 	= 1;
			MSG.label 	= LABEL;
			MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
			MSG.pred 	= prediction;
			outputMsg(MSG, Graph_);
		}

		// 3. Prediction within location area
		else
		{
			flag_predict      = false;
			flag_predict_last = false;
			reshapeVector(loc_last_idxs,num_locations);

			predictionNode(
					pva_avg, pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1],
					loc_last, pos_vel_acc_avg[i][0].cluster_id,
					Graph_, LIMIT, LABEL,
					flag_motion, learn);

			loc_last 	= pos_vel_acc_avg[i][0].cluster_id;
			flag_motion = false;

			pva_avg.clear();

			MSG.msg 	= 2;
			MSG.label 	= LABEL;
			MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
			MSG.pred 	= prediction;
			outputMsg(MSG, Graph_);
		}

		MSG.msg 	= 3;
		MSG.label 	= LABEL;
		MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
		MSG.pred 	= prediction;
		outputMsg(MSG, Graph_);

// ============================================================================
// PREDICTION ENDS
// ============================================================================

	} // points

	showConnection(p_data,label_data,Graph_,color_code,true);

#endif

//		vector<double> x,y1,y2,y0;
//		for(int i=0;i<num_points;i++)
//		{
//			x.push_back(i);
//			y0.push_back(l2Norm(pos_vel_acc_avg[i][0]));
//			y1.push_back(l2Norm(pos_vel_acc_avg[i][1]));
//			y2.push_back(l2Norm(pos_vel_acc_avg[i][2]));
//		}
//
//		plotData(x, y0);
//		plotData(x, y1);
//		plotData(x, y2);







	cout << "END" << endl;

	return 0;
}




