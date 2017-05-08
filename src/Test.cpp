/*
 * Test.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Test.h"

Test::Test() { }

Test::~Test() { }

int Test::Testing(
	string filename_,
	string resultdir_,
	Graph *Graph_)
{
	// [VARIABLES]*************************************************************
	vector<string> 				labels_predict;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<map<string,double> > goals;
	vector<map<string,double> > windows;

	vector<vector<double> > data_writeout;
	vector<double> x;
	vector<double> y;
	vector<double> 						px;
	vector<vector<double> > 			py; py.resize(7);
	vector<vector<vector<double> > > 	pyy; pyy.push_back(py);

	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	this->ClearRF();
	if (this->ReadFile_(filename_,',')==EXIT_FAILURE)
	{ return EXIT_FAILURE;	} printer(8);
	// *************************************************************[READ FILE]

//	if(strcmp(filename_.c_str(),"recording/04_Gregory/006/170419135904.txt"))
//		return 0;

	// [PARSE DATA]************************************************************
	this->ClearParser();
	this->SetDataParser(data_rf);
	if (this->ParseData()==EXIT_FAILURE) {return EXIT_FAILURE;}
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	if(0)
	{
		for(int ii=0;ii<Graph_->GetNumberOfNodes();ii++)
		{
			node_tt node_tmp = {};
			Graph_->GetNode(ii, node_tmp);
			if (!strcmp(node_tmp.name.c_str(),"FACE"))
			{
				face_parser.l = 0.10;
				node_tmp.centroid = face_parser;
				Graph_->SetNode(node_tmp);
			}
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [Initialization] *******************************************************
	reshapeVector(pva_avg, points_parser.size());
	labels_predict.resize(points_parser.size());
	goals.resize(points_parser.size());
	windows.resize(points_parser.size());

	this->ResetFilter();

	ActionPrediction AP(Graph_, false);
//	Prediction P(Graph_->GetActionCategory(), Graph_->GetActionLabel(),3);
	// ******************************************************* [Initialization]

	AP.PredictInit();

	for(int i=0;i<points_parser.size();i++)
	{
		if (i==0)	{ pva_avg[i] = pva_avg1;		}
		else		{ pva_avg[i] = pva_avg[i-1];	}
		this->PreprocessDataLive(points_parser[i], pva_avg[i], FILTER_WIN);
		this->PreprocessContactLive(contact_parser[i], FILTER_WIN);

		AP.PredictExt(pva_avg[i], contact_parser[i]);

//		if(Graph_->GetActionState().label1>=0)
		{
			vector<double> tmp;
			tmp.push_back(i);
			tmp.push_back(Graph_->GetActionState().grasp);
			tmp.push_back(Graph_->GetActionState().label1);
			tmp.push_back(Graph_->GetActionState().label2);
			tmp.push_back(Graph_->GetActionState().mov);
			tmp.push_back(Graph_->GetActionState().sur);
			tmp.push_back(fabs(checkSurfaceDistance(pva_avg[i][0],Graph_->GetSurfaceEq()[1])));
			tmp.push_back(pva_avg[i][0].x);
			tmp.push_back(pva_avg[i][0].y);
			tmp.push_back(pva_avg[i][0].z);
			data_writeout.push_back(tmp);
		}

		//decideBoundarySphere(pva_avg[i][0], pva_avg[i][0], Graph_->GetCentroidList());

		x.push_back(i);
		y.push_back((double)Graph_->GetActionState().label2);

		vector<string> al_tmp = Graph_->GetActionLabel();
		map<string,pair<int,int> > ac_tmp = Graph_->GetActionCategory();
		for(int i=ac_tmp["GEOMETRIC"].first;i<ac_tmp["GEOMETRIC"].second+1;i++)
		{
			pyy[0][i].push_back(Graph_->GetActionState().goal[al_tmp[i]]);
		}

		goals[i] = Graph_->GetActionState().goal;
		windows[i] = Graph_->GetActionState().window;

		if (Graph_->GetActionState().grasp==RELEASE)
		{
			labels_predict[i] = "RELEASE";
		}
		else if (Graph_->GetActionState().pct_err<0)
		{
			labels_predict[i] =
					Graph_->GetActionLabel()[Graph_->GetActionState().label2];
		}
		else
		{
			labels_predict[i] = "MOVE";
		}

//		P.Parse(Graph_->GetActionState());
//		P.Display();

		// Visualize
		if (0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			for(int ii=0;ii<i+1;ii++) point_zero.push_back(pva_avg[ii][0]);
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, point_zero, label_zero, color_code, true);
		}

	}

	// writing results
	{
		string name_tmp = filename_;
		reverse(name_tmp.begin(),name_tmp.end());
		name_tmp.erase(name_tmp.begin()+name_tmp.find("/"),name_tmp.end());
		reverse(name_tmp.begin(),name_tmp.end());
		this->WriteFilePrediction(
				Graph_, (resultdir_ + name_tmp), labels_parser, labels_predict,
				goals, windows);
		this->WriteFile_((resultdir_ + "_" + name_tmp), data_writeout);
	}

//	plotData(x,y);

//	vector<string> title; title.resize(1);
//	plotDatas(title, x, pyy);

	// Visualize
	if(0)
	{
		vector<string>goal_action;
		for(int i=0;i<Graph_->GetNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->GetNode(i, node_tmp);
			goal_action.push_back(node_tmp.name);
		}
		vector<point_d> point_zero; vector<string> label_zero;
		for(int ii=0;ii<pva_avg.size();ii++) {point_zero.push_back(pva_avg[ii][0]);}
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		vector<int> 	loc_idx_zero;
		showData(
				point_zero, goal_action, Graph_->GetActionLabel(),
				loc_idx_zero, color_code, true, false, false);

		for(int ii=0;ii<pva_avg.size();ii++)
		{
			if (!strcmp(labels_parser[ii].c_str(),"MOVE"))
				point_zero[ii].l = -1;
			else if (!strcmp(labels_parser[ii].c_str(),"RELEASE"))
				point_zero[ii].l = -1;
			else
				point_zero[ii].l = 1;
		}
		showData(
				point_zero, goal_action, Graph_->GetActionLabel(),
				loc_idx_zero, color_code, true, false, false);
	}

	// Visualize
	if (0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		for(int i=0;i<Graph_->GetNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->GetNode(i, node_tmp);
			label_zero.push_back(node_tmp.name);
		}
		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, point_zero, label_zero, color_code, true);
	}


	return EXIT_SUCCESS;
}

int Test::Deploying(
	string filename_,
	string resultdir_,
	Graph *Graph_)
{
	// [VARIABLES]*************************************************************
	vector<int> 				contact;
	vector<point_d> 			points;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	this->ClearRF();
	if (this->ReadFile_(filename_,',')==EXIT_FAILURE)
	{ return EXIT_FAILURE;	} printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	this->ClearParser();
	this->SetDataParser(data_rf);
	if (this->ParseData()==EXIT_FAILURE) {return EXIT_FAILURE;}
	points = this->GetPointParser();
	contact = this->GetContactParser();
	printer(9);
	// ************************************************************[PARSE DATA]

	// [Initialization] *******************************************************
	reshapeVector(pva_avg, points.size());
	this->ResetFilter();
	ActionPrediction AP(Graph_, false);
	Prediction P(Graph_->GetObject(), Graph_->GetActionCategory(),
					Graph_->GetActionLabel(), Graph_->GetObjectLabel(), 3);
	// ******************************************************* [Initialization]

	AP.PredictInit();

	for(int i=0;i<points.size();i++)
	{
		if (i==0)	{ pva_avg[i] = pva_avg1;		}
		else		{ pva_avg[i] = pva_avg[i-1];	}

		this->PreprocessDataLive(points[i], pva_avg[i], FILTER_WIN);
		this->PreprocessContactLive(contact[i], FILTER_WIN);

		AP.PredictExt(pva_avg[i],contact[i]);

		P.Parse(Graph_->GetActionState());
	}

	return EXIT_SUCCESS;
}
