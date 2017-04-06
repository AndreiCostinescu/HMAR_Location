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
		contact_[i]   = atoi(data_[i][1].c_str());
		points_ [i].x = atof(data_[i][2].c_str());
		points_ [i].y = atof(data_[i][3].c_str());
		points_ [i].z = atof(data_[i][4].c_str());
		points_ [i].l = UNCLASSIFIED;
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
	return EXIT_SUCCESS;
}


// ============================================================================
// Modules
// ============================================================================

int learning(
	string dirname_,
	Graph *Graph_)
{
	// [VARIABLES]*************************************************************
	struct dirent 				**namelist;
	int* 						file_eof;

	string						path;
	vector<int> 				contact;
	vector<int> 				last_loc;
	vector<vector<string> > 	data;
	vector<point_d> 			points;
	vector<point_d> 			curve_mem;
	vector<point_d> 			delta_t_mem;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
	vector<point_d> 			faces;
	predict_t					predict;
	printer(1);
	// *************************************************************[VARIABLES]

	// [ACTION LABELS]*********************************************************
	path = string(SCN) + "/action_label.txt";
	if (readFileExt(Graph_, path.c_str(), 2)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	// *********************************************************[ACTION LABELS]

	// [OBJ MASK]**************************************************************
	path =
			string(SCN) + Graph_->getScene() + "/" + Graph_->getObject() +
			"/obj_action_label.txt";
	if (readFileExt(Graph_, path.c_str(), 1)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	// **************************************************************[OBJ MASK]

	// [SAVED LOCATION]********************************************************
	path =
			string(SCN) + Graph_->getScene() + "/" + Graph_->getObject() +
			"/location_area.txt";
	readFileExt(Graph_, path.c_str(), 0);
	// ********************************************************[SAVED LOCATION]

	// [READ FILE]*************************************************************
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) {printer(24); return EXIT_FAILURE;}
	file_eof = new int[n];
	for(int i=0;i<n;i++)
	{
		string name = dirname_ + "/" + namelist[i]->d_name;
		readFile(name.c_str(), data , ',');
		file_eof[i] = data.size();
		point_d face; // use of 10 is just random
		face.x = atof(data[file_eof[i]-10][5].c_str());
		face.y = atof(data[file_eof[i]-10][6].c_str())-0.10;
		face.z = atof(data[file_eof[i]-10][7].c_str())-0.05;
		face.l = UNCLASSIFIED;
		faces.push_back(face);
	}
	printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	parseData2Point(data, points, contact);
	printer(9);
	// ************************************************************[PARSE DATA]

	// [LOOP THROUGH FILES]****************************************************
	int tmp_id1 = 0;
	for(int i=0;i<n;i++)
	{
		for(int ii=0;ii<Graph_->getNumberOfNodes();ii++)
		{
			node_tt node_tmp = {};
			Graph_->getNode(ii, node_tmp);
			if (!strcmp(node_tmp.name.c_str(),"DRINK") || !strcmp(node_tmp.name.c_str(),"EAT"))
			{
				faces[i]. l = 0.10;
				node_tmp.centroid = faces[i];
				Graph_->setNode(node_tmp);
			}
		}
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
		buildLocationArea(Graph_, pva_avg, contact_tmp);
		printer(11);
		buildSectorMap   (Graph_, pva_avg, contact_tmp);
		printer(12);
		// ***************************************[LOCATION AND SECTOR-MAP]
		tmp_id1 = file_eof[i];

		// Visualize
		if(1)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, point_zero, label_zero, color_code, true);
		}
	}
	// *************************************************** [LOOP THROUGH FILES]
	delete[] file_eof;

	return EXIT_SUCCESS;
}


int testing(
	string dirname_,
	Graph *Graph_)
{
	// [VARIABLES]*************************************************************
	struct dirent 				**namelist;
	int* 						file_eof;

	bool 						sm_init;
	int 						label1;
	string 						path;
	vector<int> 				contact;
	vector<int> 				last_loc;
	vector<vector<string> > 	data;
	vector<point_d> 			points;
	vector<point_d> 			curve_mem;
	vector<point_d> 			delta_t_mem;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_avg_mem;
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
	predict_t					predict;
	vector<predict_t>			predict_mem;


	vector<double> 						px;
	vector<vector<double> > 			py;
	vector<vector<vector<double> > > 	pyy;

	map<string,pair<int,int> > 		ac;
	vector<string> 					al;
	vector<map<string, double> >	prediction;

	printer(1);
	// *************************************************************[VARIABLES]

//	// [ACTION LABELS]*********************************************************
//	path = string(SCN) + "/action_label.txt";
//	if (readFileExt(Graph_, path.c_str(), 2)==EXIT_FAILURE)
//	{return EXIT_FAILURE;}
//	// *********************************************************[ACTION LABELS]
//
//	// [OBJ MASK]**************************************************************
//	path =
//			string(SCN) + Graph_->getScene() + "/" + Graph_->getObject() +
//			"/obj_action_label.txt";
//	if (readFileExt(Graph_, path.c_str(), 1)==EXIT_FAILURE)
//	{return EXIT_FAILURE;}
//	// **************************************************************[OBJ MASK]
//
//	// [SAVED LOCATION]********************************************************
//	path =
//			string(SCN) + Graph_->getScene() + "/" + Graph_->getObject() +
//			"/location_area.txt";
//	if (readFileExt(Graph_, path.c_str(), 0)==EXIT_FAILURE)
//	{return EXIT_FAILURE;}
//	// ********************************************************[SAVED LOCATION]

	// [READ FILE]*************************************************************
	string name;
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) {printer(24); return EXIT_FAILURE;}
	file_eof = new int[n];
	for(int i=0;i<n;i++)
	{
		name = dirname_ + "/" + namelist[i]->d_name;
		readFile(name.c_str(), data , ',');
		file_eof[i] = data.size();
	}
	printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	parseData2Point(data, points, contact);
	printer(9);
	// ************************************************************[PARSE DATA]

	// [LOOP THROUGH FILES]****************************************************
	int tmp_id1 = 0;
	int id1 = 0;
	int id2 = id1+1;
	for(int i=id1;i<id2;i++)
//	int tmp_id1 = 0;
//	for(int i=0;i<file_eof.size();i++)
	{
		// ====================================================================
		// TESTING (ONLINE PREDICTION)
		// ====================================================================
		vector<int> contact_tmp; contact_tmp.resize(file_eof[i]-1-tmp_id1);
		for(int iii=30;iii<contact_tmp.size();iii++)
		{
			contact_tmp[iii] =
					(int)(
							round(
									(float)accumulate(
										contact.begin()+tmp_id1-30+iii,
										contact.begin()+tmp_id1+iii, 0) / 30));
		}


		printf("\n******************************************************************************\n");
		printf("* TESTING START                                                              *\n");
		printf("******************************************************************************\n");

		// [Initialization] ***************************************************
		Graph_->getActionCategory(ac);
		Graph_->getActionLabel(al);

		vector<point_d> point_tmp(
				points.begin() + tmp_id1,
				points.begin() + file_eof[i]-1);
		reshapeVector(pva_avg, point_tmp.size());
		reshapeVector(pva_mem, 3);
		reshapeVector(last_loc, Graph_->getNumberOfNodes());
		reshapePredict(predict, Graph_->getNumberOfNodes());

		pva_avg_mem.clear();
		vector<bool> sm_init; for(int is=0;is<Graph_->getNumberOfNodes();is++) sm_init.push_back(true);
		label1 = 0; //###TODO:HACK ALWAYS START AT 0
		Graph* Graph_update = new Graph(*Graph_);

		px.clear();
		py.resize(ac["GEOMETRIC"].second-ac["GEOMETRIC"].first+1);
		for(int ii=0;ii<al.size();ii++) {pyy.push_back(py);}

		vector<map<string, double> > p_tmp;
		Graph_->getPredictionReset(p_tmp);
		Graph_->setPrediction(p_tmp);

		vector<double> prob_tmp;
		int decide = -1;

		string out1 = "";
		string out2 = "";
		// *************************************************** [Initialization]
		for(int ii=0;ii<point_tmp.size();ii++)
		{
			if (ii == 0) 	{ pva_avg[ii] = pva_avg1;		}
			else			{ pva_avg[ii] = pva_avg[ii-1];	}
			preprocessDataLive(
					points[ii+tmp_id1], pva_mem, pva_avg[ii], FILTER_WIN);

			if (contact_tmp[ii]==1)
			{
				predictAction(
						Graph_,
						Graph_update,
						pva_avg[ii],
						pva_avg_mem,
						curve_mem,
						delta_t_mem,
						predict_mem,
						label1,
						last_loc,
						sm_init,
						false);
			}
//			else
//			{
//				vector<map<string, double> > p_tmp;
//				Graph_->getPredictionReset(p_tmp);
//				Graph_->setPrediction(p_tmp);
//			}

			Graph_->getPrediction(prediction);

			px.push_back(ii);
			for(int iii=0;iii<(ac["GEOMETRIC"].second-ac["GEOMETRIC"].first+1);iii++)
			{
				int c = 0;
				pyy[c][iii].push_back(prediction[iii]["EAT"]); c++;
				pyy[c][iii].push_back(prediction[iii]["DRINK"]); c++;
				pyy[c][iii].push_back(prediction[iii]["WASH"]); c++;
				pyy[c][iii].push_back(prediction[iii]["THROW"]); c++;
				pyy[c][iii].push_back(prediction[iii]["KEEP"]); c++;
				pyy[c][iii].push_back(prediction[iii]["REST"]); c++;
				pyy[c][iii].push_back(prediction[iii]["CUT"]); c++;
				pyy[c][iii].push_back(prediction[iii]["CLEAN"]); c++;
				pyy[c][iii].push_back(prediction[iii]["SCAN"]); c++;
				pyy[c][iii].push_back(prediction[iii]["WINDOW"]); c++;
				pyy[c][iii].push_back(prediction[iii]["SLIDE"]); c++;
				pyy[c][iii].push_back(prediction[iii]["CURVE"]); c++;
				pyy[c][iii].push_back(prediction[iii]["MOVE"]); c++;
				pyy[c][iii].push_back(prediction[iii]["STOP"]);
			}

			if (contact_tmp[ii]==0)
			{
				out1 = "[---] NO CONTACT\n";
				if(strcmp(out1.c_str(),out2.c_str()))
					printf("%s", out1.c_str());
				out2 = out1;
				continue;
			}

			if (pva_avg[ii][0].l<0)
			{
				prob_tmp.clear();
				prob_tmp.resize(ac["GEOMETRIC"].second-ac["GEOMETRIC"].first+1);
				for(int iii=ac["GEOMETRIC"].first;
						iii<ac["GEOMETRIC"].second+1;
						iii++)
				{
					prob_tmp[iii] = prediction[iii][al[iii]];
				}
				if (*max_element(prob_tmp.begin(), prob_tmp.end()) <= 0)
				{
//					out1 = to_string(ii) + "[GLA] UNKNOWN GOAL ";
					out1 = "[GLA] UNKNOWN GOAL ";
				}
				else
				{
					decide =
							distance(
									prob_tmp.begin(),
									max_element(
										   prob_tmp.begin(),
										   prob_tmp.end()));
//					out1 = to_string(ii) + "[GLA] " + al[decide] + " ";
					out1 = "[GLA] " + al[decide] + " ";
				}

				prob_tmp.clear();
				prob_tmp.resize(ac["MOVEMENT"].second-ac["MOVEMENT"].first+1);
				for(int iii=ac["MOVEMENT"].first;
						iii<ac["MOVEMENT"].second+1;
						iii++)
				{
					prob_tmp[iii-ac["MOVEMENT"].first] = prediction[decide][al[iii]];
				}
				if (*max_element(prob_tmp.begin(), prob_tmp.end()) <= 0)
				{
					out1 += "[MOV] UNKNOWN MOV ";
				}
				else
				{
					out1 += "[MOV] " + al[ ac["MOVEMENT"].first +
										   distance(
												   prob_tmp.begin(),
												   max_element(
														   prob_tmp.begin(),
														   prob_tmp.end()))] + " ";
				}

				prob_tmp.clear();
				prob_tmp.resize(ac["GENERIC"].second-ac["GENERIC"].first+1);
				for(int iii=ac["GENERIC"].first;
						iii<ac["GENERIC"].second+1;
						iii++)
				{
					prob_tmp[iii-ac["GENERIC"].first] = prediction[decide][al[iii]];
				}
				if (*max_element(prob_tmp.begin(), prob_tmp.end()) <= 0)
				{
					out1 += "[GEN] UNKNOWN GEN\n";
					if(strcmp(out1.c_str(),out2.c_str()))
						printf("%s", out1.c_str());
					out2 = out1;
				}
				else
				{
					out1 += "[GEN] " + al[ ac["GENERIC"].first +
										   distance(
												   prob_tmp.begin(),
												   max_element(
														   prob_tmp.begin(),
														   prob_tmp.end()))] + "\n";
					if(strcmp(out1.c_str(),out2.c_str()))
						printf("%s", out1.c_str());
					out2 = out1;
				}

			}
			else
			{
				node_tt node_tmp = {};
				Graph_->getNode(pva_avg[ii][0].l, node_tmp);
//				out1 = to_string(ii) + "[CLA] " + node_tmp.name + "\n";
				out1 = "[CLA] " + node_tmp.name + "\n";
				if(strcmp(out1.c_str(),out2.c_str()))
					printf("%s", out1.c_str());
				out2 = out1;
			}
		}

//		{
//			vector<string> al_tmp(
//					al.begin() + ac["GEOMETRIC"].first,
//					al.begin() + ac["GEOMETRIC"].second+1);
//			vector<vector<vector<double> > > pyyy(
//					pyy.begin() + ac["GEOMETRIC"].first,
//					pyy.begin() + ac["GEOMETRIC"].second+1);
//			plotDatasGeo(al_tmp,px,pyyy);
//		}
//
//		{
//			vector<string> al_tmp(
//					al.begin() + ac["MOVEMENT"].first,
//					al.begin() + ac["MOVEMENT"].second+1);
//			vector<vector<vector<double> > > pyyy(
//					pyy.begin() + ac["MOVEMENT"].first,
//					pyy.begin() + ac["MOVEMENT"].second+1);
//			plotDatas(al_tmp,px,pyyy);
//		}

		{
			vector<string> al_tmp(
					al.begin() + ac["GENERIC"].first,
					al.begin() + ac["GENERIC"].second+1);
			vector<vector<vector<double> > > pyyy(
					pyy.begin() + ac["GENERIC"].first,
					pyy.begin() + ac["GENERIC"].second+1);
			plotDatas(al_tmp,px,pyyy);
		}

		vector<point_d> point_zero; vector<string> label_zero;
		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, point_zero, label_zero, color_code, true);

		tmp_id1 = file_eof[i];
		printf("******************************************************************************\n");
		printf("* TESTING END                                                                *\n");
		printf("******************************************************************************\n\n");
	}
	// *************************************************** [LOOP THROUGH FILES]
	return EXIT_SUCCESS;
}
