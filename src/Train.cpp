/*
 * Train.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Train.h"

Train::Train() {
	// TODO Auto-generated constructor stub

}

Train::~Train() {
	// TODO Auto-generated destructor stub
}

int Train::Learning(
	string filename_,
	Graph *Graph_,
	bool flag_)
{
	// [VARIABLES]*************************************************************
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
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
	if (this->ParseDataNoLabel()==EXIT_FAILURE) {return EXIT_FAILURE;}
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	if (0)
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

	// [PREPROCESS DATA] ******************************************************
	reshapeVector(pva_avg, points_parser.size());
	this->ResetFilter();
	for(int ii=0;ii<points_parser.size();ii++)
	{
		if (ii==0)	{ pva_avg[ii] = pva_avg1;		}
		else		{ pva_avg[ii] = pva_avg[ii-1];	}
		this->PreprocessDataLive(points_parser[ii], pva_avg[ii], FILTER_WIN);
		this->PreprocessContactLive(contact_parser[ii], FILTER_WIN);
	}
	printer(10);
	// ****************************************************** [PREPROCESS DATA]

	// [LOCATION AND SECTOR-MAP]***********************************************
	this->ClearLA();
	this->BuildLocationArea(Graph_, pva_avg, contact_parser, flag_);
	printer(11);
	this->ClearSM();
	this->BuildSectorMap   (Graph_, pva_avg, contact_parser);
	printer(12);
	// ***************************************[LOCATION AND SECTOR-MAP]

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
//		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnectionTest(Graph_, point_zero, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}
