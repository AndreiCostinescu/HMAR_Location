/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

//#define DBSCAN

#include "util.h"

// ============================================================================
// Data
// ============================================================================

int parseData2Point(
	vector<vector<string> > data_,
	vector<point_d> 		&points_,
	vector<int> 			&contact_)
{
	reshapeVector(points_, data_.size());
	reshapeVector(contact_,data_.size());
	for(int i=0;i<data_.size();i++)
	{
		if (data_[i].size()<5) return EXIT_FAILURE;
		contact_[i]   = atoi(data_[i][1].c_str());
		points_ [i].x = atof(data_[i][2].c_str());
		points_ [i].y = atof(data_[i][3].c_str());
		points_ [i].z = atof(data_[i][4].c_str());
		points_ [i].l = UNCLASSIFIED;
	}
	return EXIT_SUCCESS;
}

int parseData2Point(
	vector<vector<string> > data_,
	vector<point_d> 		&points_,
	vector<int> 			&contact_,
	vector<string>			&labels_)
{
	reshapeVector(points_, data_.size());
	reshapeVector(contact_,data_.size());
	reshapeVector(labels_, data_.size());
	for(int i=0;i<data_.size();i++)
	{
		if (data_[i].size()<9) return EXIT_FAILURE;
		contact_[i]   = atoi(data_[i][1].c_str());
		points_ [i].x = atof(data_[i][2].c_str());
		points_ [i].y = atof(data_[i][3].c_str());
		points_ [i].z = atof(data_[i][4].c_str());
		points_ [i].l = UNCLASSIFIED;
		labels_[i]    = data_[i][8];
	}
	return EXIT_SUCCESS;
}

int preprocessDataLive(
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
			pva_avg_[i] 	= tmp[i];
			pva_avg_[i].l 	= UNCLASSIFIED;
			for(int ii=i+1;ii<3;ii++)
			{
				pva_avg_[ii].x =
				pva_avg_[ii].y =
				pva_avg_[ii].z =
				pva_avg_[ii].l = UNCLASSIFIED;
			}
			break;
		}
		pva_avg_[i].l = UNCLASSIFIED;
	}
	return EXIT_SUCCESS;
}

int preprocessContactLive(
	int &contact_,
	vector<int> &contact_mem_,
	unsigned int window_)
{
	if(contact_mem_.size() == window_)
	{
		contact_ = movingAverage(contact_, contact_mem_);
	}
	else if (contact_mem_.size()>0)
	{
		contact_ = movingAverageIncrement(contact_, contact_mem_);
	}
	else
	{
		contact_mem_.push_back(contact_);
	}
	return EXIT_SUCCESS;
}

// ============================================================================
// Modules
// ============================================================================

int learning(
	string filename_,
	Graph *Graph_,
	bool flag_)
{
	// [VARIABLES]*************************************************************
	vector<int> 				contact;
	vector<int> 				contact_mem;
	vector<int> 				last_loc;
	vector<vector<string> > 	data;
	vector<point_d> 			points;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
	point_d 					face;
	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	readFile(filename_.c_str(), data, ',');
	if (!data.empty()) 	{ printer(8);			}
	else 				{ return EXIT_FAILURE;	}
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	if (parseData2Point(data, points, contact)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	// use of 10 is just random
	face.x = atof(data[data.size()-10][5].c_str());
	face.y = atof(data[data.size()-10][6].c_str())-0.10;
	face.z = atof(data[data.size()-10][7].c_str())-0.05;
	face.l = UNCLASSIFIED;
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	for(int ii=0;ii<Graph_->getNumberOfNodes();ii++)
	{
		node_tt node_tmp = {};
		Graph_->getNode(ii, node_tmp);
		if (!strcmp(node_tmp.name.c_str(),"FACE"))
		{
			face.l = 0.10;
			node_tmp.centroid = face;
			Graph_->setNode(node_tmp);
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [PREPROCESS DATA] ******************************************************
	reshapeVector(pva_avg, points.size());
	reshapeVector(pva_mem, 3);
	for(int ii=0;ii<points.size();ii++)
	{
		if (ii==0)	{ pva_avg[ii] = pva_avg1;		}
		else		{ pva_avg[ii] = pva_avg[ii-1];	}
		preprocessDataLive(points[ii], pva_mem, pva_avg[ii], FILTER_WIN);
		preprocessContactLive(contact[ii], contact_mem, FILTER_WIN);
	}
	printer(10);
	// ****************************************************** [PREPROCESS DATA]

	// [LOCATION AND SECTOR-MAP]***********************************************
	vector<point_d> p;
	for(int ii=0;ii<pva_avg.size();ii++) p.push_back(pva_avg[ii][0]);

	buildLocationArea(Graph_, pva_avg, contact, flag_);
	printer(11);

//	for(int ii=0;ii<p.size();ii++)
//	decideBoundary___(p[ii], p[ii], Graph_->getCentroidList(), Graph_->getSurfaceFlagList(), Graph_->getSurfaceEq());
//	// Visualize
//	if (1)
//	{
//		vector<point_d> point_zero; vector<string> label_zero;
//		for(int i=0;i<Graph_->getNumberOfNodes();i++)
//		{
//			node_tt node_tmp = {};
//			Graph_->getNode(i, node_tmp);
//			label_zero.push_back(node_tmp.name);
//		}
//		vector<vector<unsigned char> > color_code; colorCode(color_code);
//		showConnection(Graph_, p, label_zero, color_code, true);
//	}


	buildSectorMap   (Graph_, pva_avg, contact);
	printer(12);
	// ***************************************[LOCATION AND SECTOR-MAP]

	// Visualize
	if (0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		for(int i=0;i<Graph_->getNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->getNode(i, node_tmp);
			label_zero.push_back(node_tmp.name);
		}
		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, point_zero, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}

int testing(
	string filename_,
	string resultdir_,
	int resultnum_,
	Graph *Graph_)
{
	// [VARIABLES]*************************************************************
	vector<bool> 				sm_init;
	int 						label1;
	vector<int> 				contact;
	vector<int> 				contact_mem;
	vector<int> 				last_loc;
	vector<vector<string> > 	data;
	vector<point_d> 			points;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_avg_mem;
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
	vector<predict_t>			predict_mem;
	point_d 					face;


	vector<double> 						px;
	vector<vector<double> > 			py;
	vector<vector<vector<double> > > 	pyy;

	vector<map<string, double> >	prediction;

	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	readFile(filename_.c_str(), data, ',');
	if (!data.empty()) 	{ printer(8);			}
	else 				{ return EXIT_FAILURE;	}
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	if (parseData2Point(data, points, contact)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
//	vector<string> labels;
//	if (parseData2Point(data, points, contact, labels)==EXIT_FAILURE)
//	{return EXIT_FAILURE;}
	// use of 10 is just random
	face.x = atof(data[data.size()-10][5].c_str());
	face.y = atof(data[data.size()-10][6].c_str())-0.10;
	face.z = atof(data[data.size()-10][7].c_str())-0.05;
	face.l = UNCLASSIFIED;
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	for(int ii=0;ii<Graph_->getNumberOfNodes();ii++)
	{
		node_tt node_tmp = {};
		Graph_->getNode(ii, node_tmp);
		if (!strcmp(node_tmp.name.c_str(),"FACE"))
		{
			face.l = 0.10;
			node_tmp.centroid = face;
			Graph_->setNode(node_tmp);
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [Initialization] *******************************************************
	reshapeVector(pva_avg, points.size());
	reshapeVector(pva_mem, 3);
	reshapeVector(last_loc, Graph_->getNumberOfNodes());
	for(int is=0;is<Graph_->getNumberOfNodes();is++) sm_init.push_back(true);

	Prediction P(3,Graph_->getActionCategory(),Graph_->getActionLabel());

	state_t s_tmp = Graph_->getState();
	s_tmp.grasp = false;
	s_tmp.label1 = -1;
	s_tmp.label2 = -1;
	s_tmp.con = -1;
	s_tmp.sur = -1;
	s_tmp.pct_err = -1;
	s_tmp.mov = 0;
	Graph_->setState(s_tmp);

	label1 = 0;

	char num[8]; sprintf(num, "%03d.txt", resultnum_);
	ofstream write_file(resultdir_ + num, ios::app);

	vector<double> x; vector<double> y;
	// ******************************************************* [Initialization]

	for(int i=0;i<points.size();i++)
	{
		if (i==0)	{ pva_avg[i] = pva_avg1;		}
		else		{ pva_avg[i] = pva_avg[i-1];	}
		preprocessDataLive(points[i], pva_mem, pva_avg[i], FILTER_WIN);
		preprocessContactLive(contact[i], contact_mem, FILTER_WIN);

		if (contact[i]==1)
		{
			state_t state = Graph_->getState();
			state.grasp = true;
			Graph_->setState(state);
			predictAction(
					Graph_, pva_avg[i], pva_avg_mem, predict_mem, label1,
					last_loc, sm_init, false);
		}
		else
		{
			state_t state = Graph_->getState();
			state.grasp = false;
			Graph_->setState(state);
		}

		x.push_back(i);
		y.push_back((double)Graph_->getState().label2);
//		cout << Graph_->getState().label2 << " " << (double)Graph_->getState().pct_err << endl;

		P.parse(Graph_->getState());
		P.display();

//		write_file << labels[i] << ",";
		if (!Graph_->getState().grasp)
		{
			write_file << "RELEASE";
		}
		else if (Graph_->getState().pct_err<0)
		{
			write_file << Graph_->getActionLabel()[Graph_->getState().label2];
		}
		else
		{
			write_file << "MOVE";
		}
		write_file << "\n";

	}

	plotData(x,y);

	// Visualize
	if (0)
	{
		vector<string>goal_action;
		for(int i=0;i<Graph_->getNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->getNode(i, node_tmp);
			goal_action.push_back(node_tmp.name);
		}
		vector<point_d> point_zero; vector<string> label_zero;
		for(int ii=0;ii<pva_avg.size();ii++) {point_zero.push_back(pva_avg[ii][0]);}
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		vector<int> 	loc_idx_zero;
		showData(
				point_zero, goal_action, Graph_->getActionLabel(),
				loc_idx_zero, color_code, true, false, false);
	}

	// Visualize
	if (0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		for(int i=0;i<Graph_->getNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->getNode(i, node_tmp);
			label_zero.push_back(node_tmp.name);
		}
		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, point_zero, label_zero, color_code, true);
	}


	return EXIT_SUCCESS;
}
