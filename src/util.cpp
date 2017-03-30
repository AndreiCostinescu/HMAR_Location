/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

//#define DBSCAN

#include "util.h"

#ifdef PC
	string SCENE_ = "../Scene/";
#else
	string SCENE_ = "Scene/";
#endif

// ============================================================================
// Data
// ============================================================================

void parseData2Point(
	vector<vector<string> > data_,
	vector<point_d> 		&points_,
	vector<int> 			&contact_)
{
	reshapeVector(points_, data_.size());
	reshapeVector(contact_,data_.size());
	for(int i=0;i<data_.size();i++)
	{
		contact_[i]   = atoi(data_[i][1].c_str());
		points_ [i].x = atof(data_[i][2].c_str());
		points_ [i].y = atof(data_[i][3].c_str());
		points_ [i].z = atof(data_[i][4].c_str());
		points_ [i].l = UNCLASSIFIED;
	}
}

void preprocessDataLive(
	point_d pos_,
	vector< vector< point_d > > &pva_mem_, // motion -> length(empty at beginning)
	vector<point_d> &pva_avg_, //motion
	unsigned int window_)
{
	point_d vel = minusPoint(pos_, pva_avg_[0]);
	point_d acc = minusPoint(vel , pva_avg_[1]);
	vector<point_d> tmp; tmp.resize(3);
	tmp[0] = pos_; tmp[1] = vel; tmp[2] = acc;
	for(int i=0;i<3;i++)
	{
		if(pva_mem_[i].size() == window_)
		{
			pva_avg_[i] = movingAverage(tmp[i], pva_mem_[i]);
		}
		else if (pva_mem_[i].size()>0)
		{
			pva_avg_[i] = averagePointIncrement(tmp[i], pva_mem_[i]);
		}
		else
		{
			pva_mem_[i].push_back(tmp[i]);
			pva_avg_[i] = tmp[i];
			for(int ii=i+1;ii<3;ii++)
			{
				pva_avg_[ii].x =
				pva_avg_[ii].y =
				pva_avg_[ii].z =
				pva_avg_[ii].l = 0.0;
			}
			break;
		}
		if(i==0)	{ pva_avg_[i].l = UNCLASSIFIED;	}
		else		{ pva_avg_[i].l = 0.0;			}
	}
}


// ============================================================================
// Modules
// ============================================================================

int learning(
	string dirname_,
	string scene,
	string object)
{
	// [VARIABLES]*************************************************************
	string path;
	vector<int> 				file_eof;
	vector<int> 				contact;
	vector<int> 				last_loc;
	map<string,pair<int,int> >	action_cat;
	vector<string> 				action_label;
	vector<vector<string> > 	data;
	vector<point_d> 			points;
	vector<point_d> 			curve_mem;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
	vector<int>					obj_mask;
	predict_t					predict;
	Graph 						Graph_main(scene, object);
	printer(1);
	// *************************************************************[VARIABLES]

	// [ACTION LABELS]*********************************************************
	path =  SCENE_ + "/action_label.txt";
	data.clear();
	readFile(path.c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=0;i<data.size();i++)
		{
			if (i<3)
			{
				pair<int,int> tmp_pair(
						atoi(data[i][1].c_str()),
						atoi(data[i][2].c_str()));
				action_cat[data[i][0]] = tmp_pair;
			}
			else
			{
				action_label.push_back(data[i][0]);
			}
		}
		Graph_main.setActionCategory(action_cat);
		Graph_main.setActionLabel(action_label);
		Graph_main.initFilter();
	}
	else
	{
		printer(3);
		return EXIT_FAILURE;
	}
	printer(2);
	// *********************************************************[ACTION LABELS]

	// [OBJ MASK]**************************************************************
	path =  SCENE_ + scene + "/" + object + "/obj_action_label.txt";
	data.clear();
	readFile(path.c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=3;i<data.size();i++)
		{
			obj_mask.push_back(atoi(data[i][0].c_str()));
		}
		Graph_main.setInitFilter(obj_mask);
	}
	else
	{
		printer(5);
		return EXIT_FAILURE;
	}
	printer(4);
	// **************************************************************[OBJ MASK]

//	// [FILTER MASK]***********************************************************
//	path =  SCENE_ + scene + "/" + object + "/filter_mask.txt";
//	data.clear();
//	readFile(path.c_str(), data , ',');
//	if (!data.empty())
//	{
//		filter_mask.resize(data[0].size());
//		for(int i=0;i<data.size();i++)
//		{
//			for(int ii=0;ii<data[i].size();ii++)
//			{
//				filter_mask[ii][i] = atoi(data[i][ii].c_str());
//			}
//		}
//		// options : c_cur, c_vel, c_sur, c_win
//		Graph_main.setFilter("c_cur", filter_mask[0]);
//		Graph_main.setFilter("c_vel", filter_mask[1]);
//		Graph_main.setFilter("c_sur", filter_mask[2]);
//		Graph_main.setFilter("c_win", filter_mask[3]);
//	}
//	// ***********************************************************[FILTER MASK]

	// [SAVED LOCATION]********************************************************
	path =  SCENE_ + scene + "/" + object + "/location_area.txt";
	readFileExt(Graph_main, path, 0);
	// ********************************************************[SAVED LOCATION]

	// [READ FILE]*************************************************************
	data.clear();
	struct dirent **namelist;
	string name;
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dirname_ + namelist[i]->d_name;
		readFile(name.c_str(), data , ',');
		file_eof.push_back(data.size());
	}
	printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	parseData2Point(data, points, contact);
	printer(9);
	// ************************************************************[PARSE DATA]

	// [LOOP THROUGH FILES]****************************************************
//	int tmp_id1 = file_eof[file_eof.size()-2]+1;
//	for(int i=file_eof.size()-1;i<file_eof.size();i++)
	int tmp_id1 = 0;
	for(int i=0;i<file_eof.size();i++)
	{
		// ====================================================================
		// TESTING (ONLINE PREDICTION)
		// ====================================================================
		if (i==file_eof.size()-1)
		{
			directoryCheck(SCENE_ + scene + "/" + object);
			path =  SCENE_ + scene + "/" + object + "/location_area.txt";
			writeFile(Graph_main, path, 0);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_begin.txt";
			writeFile(Graph_main, path, 10);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_mid.txt";
			writeFile(Graph_main, path, 11);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_end.txt";
			writeFile(Graph_main, path, 12);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_tangent.txt";
			writeFile(Graph_main, path, 13);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_normal.txt";
			writeFile(Graph_main, path, 14);
			path = 	SCENE_ + scene + "/" + object + "/counter.txt";
			writeFile(Graph_main, path, 15);
			path = 	SCENE_ + scene + "/" + object + "/sec_data_max.txt";
			writeFile(Graph_main, path, 16);
			path = 	SCENE_ + scene + "/" + object + "/sec_data_const.txt";
			writeFile(Graph_main, path, 17);

			path = 	SCENE_ + scene + "/" + object + "/loc_data_begin.txt";
			readFileExt(Graph_main, path, 10);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_mid.txt";
			readFileExt(Graph_main, path, 11);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_end.txt";
			readFileExt(Graph_main, path, 12);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_tangent.txt";
			readFileExt(Graph_main, path, 13);
			path = 	SCENE_ + scene + "/" + object + "/loc_data_normal.txt";
			readFileExt(Graph_main, path, 14);
			path = 	SCENE_ + scene + "/" + object + "/counter.txt";
			readFileExt(Graph_main, path, 15);
			path = 	SCENE_ + scene + "/" + object + "/sec_data_max.txt";
			readFileExt(Graph_main, path, 16);
			path = 	SCENE_ + scene + "/" + object + "/sec_data_const.txt";
			readFileExt(Graph_main, path, 17);

			vector<int> contact_tmp; contact_tmp.resize(file_eof[i]-1-tmp_id1);
			for(int iii=30;iii<contact_tmp.size();iii++)
			{
				contact_tmp[iii] =
						(int)(
								(float)accumulate(
										contact.begin()+tmp_id1-30+iii,
										contact.begin()+tmp_id1+iii, 0) / 30);
			}


			printf("\n******************************************************************************\n");
			printf("* TESTING START                                                              *\n");
			printf("******************************************************************************\n");

			vector<point_d> point_tmp(
					points.begin() + tmp_id1,
					points.begin() + file_eof[i]-1);
			int p1 = point_tmp.size();
			reshapeVector(pva_avg, p1);
			reshapeVector(pva_mem, 3);
			reshapeVector(last_loc, Graph_main.getNumberOfNodes());
			reshapePredict(predict, Graph_main.getNumberOfNodes());
			bool flag_last=false;
			double pow_dec = 1.0;
			int label1 = -1;
			Graph Graph_update = Graph_main;
			vector<double> x;
			vector<vector<double> > y; y.resize(10);
			for(int ii=0;ii<p1;ii++)
			{
				if (ii == 0) 	{ pva_avg[ii] = pva_avg1;		}
				else			{ pva_avg[ii] = pva_avg[ii-1];	}
				preprocessDataLive(
						points[ii+tmp_id1], pva_mem, pva_avg[ii], FILTER_WIN);
				predictAction(
						Graph_main,
						Graph_update,
						contact_tmp[ii],
						pva_avg[ii],
						curve_mem,
						predict,
						label1,
						last_loc,
						false);


				map<string,pair<int,int> > ac_tmp;
				vector<string> al_tmp;
				vector<map<string, double> > f_tmp;
				Graph_main.getActionCategory(ac_tmp);
				Graph_main.getActionLabel(al_tmp);
				Graph_main.getPrediction(f_tmp);


				if (f_tmp.size()>0)
				{
					x.push_back(ii);
					for(int iii=0;iii<Graph_main.getNumberOfNodes();iii++)
					{
//						y[iii].push_back(f_tmp[iii]["DRINK"]);
//						y[iii].push_back(f_tmp[iii]["THROW"]);
//						y[iii].push_back(f_tmp[iii]["CUT"]);
//						y[iii].push_back(f_tmp[iii]["CLEAN"]);
//						y[iii].push_back(f_tmp[iii]["SCAN"]);
						y[iii].push_back(f_tmp[iii]["WINDOW"]);
//						y[iii].push_back(f_tmp[iii]["SLIDE"]);
//						y[iii].push_back(f_tmp[iii]["CURVE"]);
//						y[iii].push_back(f_tmp[iii]["MOVE"]);
//						y[iii].push_back(f_tmp[iii]["STOP"]);
					}
				}


//				if (ii>800)
//				{
//					vector<point_d> point_zero; vector<string> label_zero;
//					for(int iii=0;iii<ii+1;iii++) point_zero.push_back(pva_avg[iii][0]);
//					vector<vector<unsigned char> > color_code; colorCode(color_code);
//					showConnection(Graph_update, point_zero, label_zero, color_code, true);
//				}


			}

			plotData(x,y[0]);
			plotData(x,y[1]);
//			plotData(x,y[2]);
//			plotData(x,y[3]);

			vector<point_d> point_zero; vector<string> label_zero;
			for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_update, point_zero, label_zero, color_code, true);

			tmp_id1 = file_eof[i];
			printf("******************************************************************************\n");
			printf("* TESTING END                                                                *\n");
			printf("******************************************************************************\n\n");
		}
		// ====================================================================
		// TRAINING
		// ====================================================================
		else
		{
			// [PREPROCESS DATA]***********************************************
			vector<point_d> point_tmp(
					points.begin() + tmp_id1,
					points.begin() + file_eof[i]-1);
			reshapeVector(pva_avg, point_tmp.size());
			reshapeVector(pva_mem, 3);
			for(int ii=0;ii<point_tmp.size();ii++)
			{
				if (ii == 0) 	{ pva_avg[ii] = pva_avg1;		}
				else			{ pva_avg[ii] = pva_avg[ii-1];	}
				preprocessDataLive(
						points[ii+tmp_id1], pva_mem, pva_avg[ii], FILTER_WIN);
			}
			printer(10);
			// ***********************************************[PREPROCESS DATA]

			vector<int> contact_tmp; contact_tmp.resize(file_eof[i]-1-tmp_id1);
			for(int iii=30;iii<contact_tmp.size();iii++)
			{
				contact_tmp[iii] =
						(int)(
								(float)accumulate(
										contact.begin()+tmp_id1-30+iii,
										contact.begin()+tmp_id1+iii, 0) / 30);
			}

			// [LOCATION AND SECTOR-MAP]***************************************
			buildLocationArea(Graph_main, pva_avg, contact_tmp);
			printer(11);
			buildSectorMap   (Graph_main, pva_avg, contact_tmp);
			printer(12);
			// ***************************************[LOCATION AND SECTOR-MAP]
			tmp_id1 = file_eof[i];
//
//			vector<point_d> point_zero; vector<string> label_zero;
//			for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
//			vector<vector<unsigned char> > color_code; colorCode(color_code);
//			showConnection(Graph_main, point_zero, label_zero, color_code, true);

		}
	}







//	directoryCheck(SCENE_ + scene + "/" + object);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_beg.txt";
//	writeLearnedDataFile(Graph, path, 0);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_mid.txt";
//	writeLearnedDataFile(Graph, path, 1);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_end.txt";
//	writeLearnedDataFile(Graph, path, 2);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_tangent.txt";
//	writeLearnedDataFile(Graph, path, 3);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_normal.txt";
//	writeLearnedDataFile(Graph, path, 4);
//	path = 	SCENE_ + scene + "/" + object + "/counter.txt";
//	writeLearnedDataFile(Graph, path, 5);
//	path = 	SCENE_ + scene + "/" + object + "/sec_data_max.txt";
//	writeLearnedDataFile(Graph, path, 6);
//	path = 	SCENE_ + scene + "/" + object + "/sec_data_const.txt";
//	writeLearnedDataFile(Graph, path, 7);
//	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");
	// *************************************************** [LOOP THROUGH FILES]
	return EXIT_SUCCESS;
}

