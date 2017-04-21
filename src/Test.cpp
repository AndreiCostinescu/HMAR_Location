/*
 * Test.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Test.h"

Test::Test() {
	// TODO Auto-generated constructor stub

}

Test::~Test() {
	// TODO Auto-generated destructor stub
}

int Test::Testing(
	string filename_,
	string resultdir_,
	Graph *Graph_)
{
	// [VARIABLES]*************************************************************
	point_d 					face;
	vector<int> 				contact;
	vector<int> 				last_loc;
	vector<string> 				labels;
	vector<string> 				labels_predict;
	vector<point_d> 			points;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<map<string,double> > goals;
	vector<map<string,double> > windows;

	vector<double> x;
	vector<double> y;
	vector<double> 						px;
	vector<vector<double> > 			py;
	vector<vector<vector<double> > > 	pyy;

	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	if (this->ReadFile_(filename_,',')==EXIT_FAILURE)
	{ return EXIT_FAILURE;	} printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	this->SetDataParser(this->GetDataRF());
	if (this->ParseData()==EXIT_FAILURE) {return EXIT_FAILURE;}
	points = this->GetPointParser();
	contact = this->GetContactParser();
	face = this->GetFaceParser();
	labels = this->GetLabelParser();
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
//	for(int ii=0;ii<Graph_->getNumberOfNodes();ii++)
//	{
//		node_tt node_tmp = {};
//		Graph_->getNode(ii, node_tmp);
//		if (!strcmp(node_tmp.name.c_str(),"FACE"))
//		{
//			face.l = 0.10;
//			node_tmp.centroid = face;
//			Graph_->setNode(node_tmp);
//		}
//	}
	// ********************************************************** [FACE ADJUST]

	// [Initialization] *******************************************************
	reshapeVector(pva_avg, points.size());
	labels_predict.resize(points.size());
	goals.resize(points.size());
	windows.resize(points.size());

	this->ResetFilter();

	ActionPrediction AP(Graph_,false);
	Prediction P(3,Graph_->getActionCategory(),Graph_->getActionLabel());
	// ******************************************************* [Initialization]

	for(int i=0;i<points.size();i++)
	{

		if (i==0)	{ pva_avg[i] = pva_avg1;		}
		else		{ pva_avg[i] = pva_avg[i-1];	}
		this->PreprocessDataLive(points[i], pva_avg[i], FILTER_WIN);
		this->PreprocessContactLive(contact[i], FILTER_WIN);

		if (i==0) 	{ AP.PredictInit(points[i]); }
		else 		{ AP.PredictExt(pva_avg[i],contact[i]); }

		x.push_back(i);
		y.push_back((double)Graph_->getState().label2);

		goals[i] = Graph_->getState().goal;
		windows[i] = Graph_->getState().window;

		if (!Graph_->getState().grasp)
		{
			labels_predict[i] = "RELEASE";
		}
		else if (Graph_->getState().pct_err<0)
		{
			labels_predict[i] =
					Graph_->getActionLabel()[Graph_->getState().label2];
		}
		else
		{
			labels_predict[i] = "MOVE";
		}

		//		P.Parse(Graph_->getState());
		//		P.Display();
	}

	// writing results
	{
		string name_tmp = filename_;
		reverse(name_tmp.begin(),name_tmp.end());
		name_tmp.erase(name_tmp.begin()+name_tmp.find("/"),name_tmp.end());
		reverse(name_tmp.begin(),name_tmp.end());
		name_tmp = resultdir_ + name_tmp;
		this->WriteFilePrediction(
				Graph_, name_tmp, labels, labels_predict, goals, windows);
	}

	//	plotData(x,y);

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
