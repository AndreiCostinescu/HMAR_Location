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
	string resultdir_)
{
	// [VARIABLES]*************************************************************
	vector<string> 				labels_predict;
	vector<vector<Vector4d> >	pvas;

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

	// [PARSE DATA]************************************************************
	this->ClearParser();
	this->SetDataParser(data_rf);
	if (this->ParseData()==EXIT_FAILURE) {return EXIT_FAILURE;}
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	if (1)
	{
		face_parser -= Vector4d(0,0.15,0,1);
		// When a graph is present.
		if(G->GetNumberOfNodes()>0)
		{
			// Initialize.
			double scale = 1.0;
			Vector3d t;
			Matrix3d R;
			Matrix4d T = Matrix4d::Zero();
			node_tt old_node;
			edge_tt edge_tmp;

			// Check for the LA that we intend to change.
			for(int i=0;i<G->GetNumberOfNodes();i++)
			{
				old_node = G->GetNode(i);
				if (!strcmp(old_node.name.c_str(),"FACE"))
				{
					// 1. Change the SM that has the LA as goal.
					//    The start locations stay the same.
					//    Transform to new target goal.
					//    p' = [S]*[R]*p
					for(int ii=0;ii<G->GetNumberOfNodes();ii++)
					{
						edge_tmp = G->GetEdge(ii,i,0);

						if (edge_tmp.counter==0) { continue; }

						scale =
								V4d3d(face_parser		- G->GetNode(ii).centroid).norm() /
								V4d3d(old_node.centroid - G->GetNode(ii).centroid).norm();
						R =
								rodriguezRot(
										V4d3d(old_node.centroid - G->GetNode(ii).centroid),
										V4d3d(face_parser		- G->GetNode(ii).centroid));
						T = Matrix4d::Zero();
						T.block<3,3>(0,0) = R;
						T(3,3) = scale;

						for(int j=0;j<LOC_INT;j++)
						{
							edge_tmp.tan[j] = R*edge_tmp.tan[j];
							edge_tmp.nor[j] = R.inverse().transpose()*edge_tmp.nor[j];
							edge_tmp.loc_mid[j] = T*edge_tmp.loc_mid[j];
							edge_tmp.loc_len[j] = edge_tmp.loc_len[j]*scale;
						}

						G->SetEdge(ii,i,0,edge_tmp);
					}

					// 2. Change the SM that has the LA as start.
					//    The goal locations stay the same.
					//    Transform to new origin.
					//    p' = [S]*[R,t]*p
					for(int ii=0;ii<G->GetNumberOfNodes();ii++)
					{
						edge_tmp = G->GetEdge(i,ii,0);

						if (edge_tmp.counter==0) { continue; }

						t = V4d3d(face_parser - old_node.centroid);

						R =
								rodriguezRot(
										V4d3d(G->GetNode(ii).centroid - old_node.centroid),
										V4d3d(G->GetNode(ii).centroid - face_parser		 ));
						t += R*V4d3d(G->GetNode(ii).centroid - old_node.centroid);
						scale =
								V4d3d(G->GetNode(ii).centroid - face_parser).norm() /
								V4d3d(G->GetNode(ii).centroid - old_node.centroid).norm();

						T = Matrix4d::Zero();
						T.block<3,3>(0,0) = R;
						T(3,3) = scale;

						for(int j=0;j<edge_tmp.tan.size();j++)
						{
							edge_tmp.tan[j] = R*edge_tmp.tan[j];
							edge_tmp.nor[j] = R.inverse().transpose()*edge_tmp.nor[j];
							edge_tmp.loc_mid[j] = T*edge_tmp.loc_mid[j];
							edge_tmp.loc_len[j] = edge_tmp.loc_len[j]*scale;
						}

						G->SetEdge(i,ii,0,edge_tmp);
					}

					old_node.centroid.head(3) = V4d3d(face_parser);
					G->SetNode(old_node);
					break;
				}
			}

//			for(int i=0;i<G->GetNumberOfNodes();i++)
//			{
//				for(int ii=0;ii<G->GetNumberOfNodes();ii++)
//				{
//					if (i==ii) {continue;}
//					if (i==c)
//					{
//
//					}
//					if (ii==c)
//					{
//
//					}
//				}
//			}
//
//			// writing location areas data that is known
//			point_d center_tmp;
//			vector<string> line_tmp;
//			vector<vector<string> > data_tmp;
//			for(int i=0;i<KB.al.size();i++)
//			{
//				if (!strcmp(KB.al[i].c_str(),"FACE"))
//				{
//					line_tmp.push_back(to_string(i));
//					line_tmp.push_back(to_string(face_parser.x));
//					line_tmp.push_back(to_string(face_parser.y));
//					line_tmp.push_back(to_string(face_parser.z));
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					line_tmp.push_back("");
//					break;
//				}
//			}
//			this->ReadFileLA(data_tmp,path_LA_);
//			for(int i=0;i<data_tmp.size();i++)
//			{
//				if(!strcmp(data_tmp[i][0].c_str(),line_tmp[0].c_str()))
//				{
//					center_tmp.x = atof(data_tmp[i][1].c_str());
//					center_tmp.y = atof(data_tmp[i][2].c_str());
//					center_tmp.z = atof(data_tmp[i][3].c_str());
//				}
//			}
//			this->WriteFileLA(line_tmp,data_tmp,path_LA_);
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [Initialization] *******************************************************
	this->ResetFilter();
	this->PredictInit(false);

//	Prediction P(G->GetObject(), KB.ac, KB.al, KB.ol, 3);
	// ******************************************************* [Initialization]

	for(int i=0;i<points_parser.size();i++)
	{
		this->Deploy(points_parser[i], contact_parser[i]);

//		if(G->GetActionState().label1>=0)
		{
			vector<double> tmp;
			tmp.push_back(i);
			tmp.push_back(G->GetActionState().grasp);
			tmp.push_back(G->GetActionState().label1);
			tmp.push_back(G->GetActionState().label2);
			tmp.push_back(G->GetActionState().mov);
			tmp.push_back(G->GetActionState().sur);
			tmp.push_back((checkSurfaceDistance(pva[0],KB.surface_eq[1])));
			tmp.push_back((double)pva[0][0]);
			tmp.push_back((double)pva[0][1]);
			tmp.push_back((double)pva[0][2]);
			data_writeout.push_back(tmp);
		}

		//decideBoundarySphere(pva_avg[i][0], pva_avg[i][0], G->GetCentroidList());

		x.push_back(i);
		y.push_back((double)G->GetActionState().label2);

		for(int i=KB.ac["GEOMETRIC"].first;i<KB.ac["GEOMETRIC"].second+1;i++)
		{
			pyy[0][i].push_back(G->GetActionState().goal[KB.al[i]]);
		}

		pvas.push_back(pva);
		goals.push_back(G->GetActionState().goal);
		windows.push_back(G->GetActionState().window);

		if (G->GetActionState().grasp==RELEASE)
		{
			labels_predict.push_back("RELEASE");
		}
		else if (G->GetActionState().pct_err<0)
		{
			labels_predict.push_back(KB.al[G->GetActionState().label2]);
		}
		else
		{
			labels_predict.push_back("MOVE");
		}

//		P.Parse(G->GetActionState());
//		P.Display();

		// Visualize
		if (0)
		{
			vector<Vector4d> point_zero; vector<string> label_zero;
			for(int ii=0;ii<i+1;ii++) point_zero.push_back(pvas[ii][0]);
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnectionTest(G, point_zero, label_zero, color_code, true);
		}

	}

	// writing results
	{
		string name_tmp = filename_;
		reverse(name_tmp.begin(),name_tmp.end());
		name_tmp.erase(name_tmp.begin()+name_tmp.find("/"),name_tmp.end());
		reverse(name_tmp.begin(),name_tmp.end());
		this->WriteFilePrediction(
				G, KB, (resultdir_ + name_tmp), labels_parser, labels_predict,
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
		for(int i=0;i<G->GetNumberOfNodes();i++)
		{
			goal_action.push_back(G->GetNode(i).name);
		}
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int ii=0;ii<pvas.size();ii++) {point_zero.push_back(pvas[ii][0]);}
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		vector<int> 	loc_idx_zero;
		showData(
				point_zero, goal_action, KB.al,
				loc_idx_zero, color_code, true, false, false);

		for(int ii=0;ii<pvas.size();ii++)
		{
			if (!strcmp(labels_parser[ii].c_str(),"MOVE"))
				point_zero[ii][3] = -1;
			else if (!strcmp(labels_parser[ii].c_str(),"RELEASE"))
				point_zero[ii][3] = -1;
			else
				point_zero[ii][3] = 1;
		}
		showData(
				point_zero, goal_action, KB.al,
				loc_idx_zero, color_code, true, false, false);
	}

	// Visualize
	if (0)
	{
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int i=0;i<G->GetNumberOfNodes();i++)
		{
			label_zero.push_back(G->GetNode(i).name);
		}
		for(int ii=0;ii<pvas.size();ii++) point_zero.push_back(pvas[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(G, point_zero, label_zero, color_code, true);
	}


	return EXIT_SUCCESS;
}

