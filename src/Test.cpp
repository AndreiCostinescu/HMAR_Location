/*
 * Test.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Test.h"

Test::Test() : 	VTK(NULL),
				DF(NULL),
				APred(NULL),
				AParse(NULL),
				RF(NULL),
				WF(NULL),
				loc_int(-1),
				sec_int(-1),
				f_win(-1)
{
}

Test::~Test()
{
	delete DF;
	delete APred;
	delete AParse;
	delete RF;
	delete WF;
}

int Test::Init(int loc_int_, int sec_int_, int f_win_, string obj_)
{
	DF 		= new DataFilter;
	APred 	= new ActionPrediction;
	AParse 	= new ActionParser;
	RF 		= new ReadFile;
	WF 		= new WriteFile;
	loc_int = loc_int_;
	sec_int = sec_int_;
	f_win 	= f_win_;
	APred->G->SetObject(obj_);
	APred->G->SetLocInt(loc_int);
	APred->G->SetSecInt(sec_int);
	return EXIT_SUCCESS;
}

int Test::ReadKB(string path_)
{
	if (RF->ReadFileKB(path_, APred->KB)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	else
	{return EXIT_SUCCESS;}
}

int Test::ReadLA(string path_)
{
	if (RF->ReadFileLA(APred->G, APred->KB->al, path_)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	else
	{return EXIT_SUCCESS;}
}

int Test::ReadGraph(string path_)
{
	if (RF->ReadFileGraph(APred->G, path_)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	else
	{return EXIT_SUCCESS;}
}

int Test::SetMessage(vector<string> msg_)
{
	AParse->SetMsg(msg_);
	return EXIT_SUCCESS;
}

int Test::SetKB(CKB *kb_)
{
	APred->KB = kb_;
	return EXIT_SUCCESS;
}

int Test::WriteWindow(string path_)
{
	WF->WriteFileWindow(APred->G, path_);
	return EXIT_SUCCESS;
}

int Test::ApplyGauss(int num_x_, int num_y_)
{
	vector<vector<double> > k_xy; k_xy.resize(num_x_);
	for(int i=0;i<num_x_;i++) { k_xy[i].resize(num_x_); }
	gaussKernel(k_xy, num_x_, num_x_, 1);

	vector<double> sm_tmp1; reshapeVector(sm_tmp1, loc_int*sec_int);
	vector<double> sm_tmp2; reshapeVector(sm_tmp2, loc_int*sec_int);
	double sum_tmp = 0.0;

	// Visualize
	if (0)
	{
		VTK = new VTKExtra (loc_int, sec_int);
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int i=0;i<APred->G->GetNumberOfNodes();i++)
		{ label_zero.push_back(APred->G->GetNode(i).name); }
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(APred->G, point_zero, label_zero, color_code, true);
		delete VTK;
	}

	// Gauss
	if(1)
	{
		for(int i=0;i<APred->G->GetNumberOfNodes();i++)
		{
			for(int ii=0;ii<APred->G->GetNumberOfNodes();ii++)
			{
				if (i==ii) { continue; }

				sm_tmp2 = sm_tmp1 = APred->G->GetEdgeSectorMap(i, ii, 0);

				for(int l=0;l<loc_int;l++)
				{
					for(int s=0;s<sec_int;s++)
					{
						sum_tmp = 0.0;
						for(int gkx=0;gkx<num_x_;gkx++)
						{
							for(int gky=0;gky<num_y_;gky++)
							{
								int tmpl = l-(num_y_/2)+gky; if(tmpl < 0 || tmpl>=loc_int) continue;
								int tmps = (s-(num_x_/2)+gkx+sec_int)%sec_int;
								sum_tmp += sm_tmp2[tmpl*sec_int + tmps] * k_xy[gkx][gky];
							}
						}
						sm_tmp1[l*sec_int+s] = sum_tmp;
					}
				}
				APred->G->SetEdgeSectorMap(i,ii,0,sm_tmp1);
			}
		}
	}

	// Visualize
	if (0)
	{
		VTK = new VTKExtra (loc_int, sec_int);
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int i=0;i<APred->G->GetNumberOfNodes();i++)
		{ label_zero.push_back(APred->G->GetNode(i).name); }
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(APred->G, point_zero, label_zero, color_code, true);
		delete VTK;
	}

	return EXIT_SUCCESS;
}

int Test::Testing(
	string filename_,
	string resultdir_)
{
	// [VARIABLES]**************************************************************
	vector<string> 				labels_predict;
	vector<vector<Vector4d> >	pvas;

	vector<map<string,double> > goals;
	vector<map<string,double> > windows;

	vector<vector<double> > data_writeout;
	vector<double> x;
	vector<double> y;
	vector<double> px;
	vector<vector<double> > py; py.resize(7);
	vector<vector<vector<double> > > pyy; pyy.push_back(py);
	// **************************************************************[VARIABLES]

	// [Initialization] ********************************************************
	DF->ResetFilter();
	APred->Init(false);
	AParse->Init(
			APred->G->GetObject(), APred->KB->ac, APred->KB->al, APred->KB->ol,
			3);

	string tmpname = filename_;
	replace(tmpname.begin(), tmpname.end(), '/', '_');

	printer(1);
	// ******************************************************** [Initialization]

	// [READ FILE]**************************************************************
	if (RF->ReadFile_(filename_,',')==EXIT_FAILURE)
	{ return EXIT_FAILURE;	} printer(8);
	// **************************************************************[READ FILE]

	// [PARSE DATA]*************************************************************
	this->ClearParser();
	this->SetDataParser(RF->GetDataRF());
	if (this->ParseData()==EXIT_FAILURE) {return EXIT_FAILURE;}
	printer(9);
	// *************************************************************[PARSE DATA]

	// [FACE ADJUST]************************************************************
	if (1)
	{
		face_parser -= Vector4d(0,0.15,0,1);
		// When a graph is present.
		if(APred->G->GetNumberOfNodes()>0)
		{
			// Initialize.
			double scale = 1.0;
			Vector3d t;
			Matrix3d R;
			Matrix4d T = Matrix4d::Zero();
			CGraph::node_t old_node;
			CGraph::edge_t edge_tmp;

			// Check for the LA that we intend to change.
			for(int i=0;i<APred->G->GetNumberOfNodes();i++)
			{
				old_node = APred->G->GetNode(i);
				if (!strcmp(old_node.name.c_str(),"FACE"))
				{
					// 1. Change the SM that has the LA as goal.
					//    The start locations stay the same.
					//    Transform to new target goal.
					//    p' = [S]*[R]*p
					for(int ii=0;ii<APred->G->GetNumberOfNodes();ii++)
					{
						edge_tmp = APred->G->GetEdge(ii,i,0);

						if (edge_tmp.counter==0) { continue; }

						scale =
								V4d3d(face_parser		- APred->G->GetNode(ii).centroid).norm() /
								V4d3d(old_node.centroid - APred->G->GetNode(ii).centroid).norm();
						R =
								rodriguezRot(
										V4d3d(old_node.centroid - APred->G->GetNode(ii).centroid),
										V4d3d(face_parser		- APred->G->GetNode(ii).centroid));
						T = Matrix4d::Zero();
						T.block<3,3>(0,0) = R;
						T(3,3) = scale;

						for(int j=0;j<APred->G->GetLocInt();j++)
						{
							edge_tmp.tan[j] = R*edge_tmp.tan[j];
							edge_tmp.nor[j] = R.inverse().transpose()*edge_tmp.nor[j];
							edge_tmp.loc_mid[j] = T*edge_tmp.loc_mid[j];
							edge_tmp.loc_len[j] = edge_tmp.loc_len[j]*scale;
						}

						APred->G->SetEdge(ii,i,0,edge_tmp);
					}

					// 2. Change the SM that has the LA as start.
					//    The goal locations stay the same.
					//    Transform to new origin.
					//    p' = [S]*[R,t]*p
					for(int ii=0;ii<APred->G->GetNumberOfNodes();ii++)
					{
						edge_tmp = APred->G->GetEdge(i,ii,0);

						if (edge_tmp.counter==0) { continue; }

						t = V4d3d(face_parser - old_node.centroid);

						R =
								rodriguezRot(
										V4d3d(APred->G->GetNode(ii).centroid - old_node.centroid),
										V4d3d(APred->G->GetNode(ii).centroid - face_parser		 ));
						t += R*V4d3d(APred->G->GetNode(ii).centroid - old_node.centroid);
						scale =
								V4d3d(APred->G->GetNode(ii).centroid - face_parser).norm() /
								V4d3d(APred->G->GetNode(ii).centroid - old_node.centroid).norm();

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

						APred->G->SetEdge(i,ii,0,edge_tmp);
					}

					old_node.centroid.head(3) = V4d3d(face_parser);
					APred->G->SetNode(old_node);
					break;
				}
			}
		}
	}
	// *********************************************************** [FACE ADJUST]

	// [TEST] ******************************************************************

	for(int i=0;i<points_parser.size();i++)
	{
		// 1. Filter
		DF->PreprocessDataLive(points_parser[i], APred->pva, f_win);
		DF->PreprocessContactLive(contact_parser[i], APred->contact, f_win);

		// 2. Prediction
		APred->PredictExt();

//		if(APred->AS->label1>=0)
		{
			vector<double> tmp;
			tmp.push_back(i);
			tmp.push_back(APred->AS->grasp);
			tmp.push_back(APred->AS->label1);
			tmp.push_back(APred->AS->label2);
			tmp.push_back(APred->AS->vel);
			tmp.push_back(APred->AS->surface_flag);
			tmp.push_back((checkSurfaceDistance(APred->pva[0],APred->KB->surface_eq[1])));
			tmp.push_back((double)APred->pva[0][0]);
			tmp.push_back((double)APred->pva[0][1]);
			tmp.push_back((double)APred->pva[0][2]);
			data_writeout.push_back(tmp);
		}

		//decideBoundarySphere(pva_avg[i][0], pva_avg[i][0], APred->G->GetCentroidList());

		x.push_back(i);
		y.push_back((double)APred->AS->label2);

		for(int i=APred->KB->ac["GEOMETRIC"].first;i<APred->KB->ac["GEOMETRIC"].second+1;i++)
		{
			pyy[0][i].push_back(APred->AS->goal[APred->KB->al[i]]);
		}

		pvas.push_back(APred->pva);
		goals.push_back(APred->AS->goal);
		windows.push_back(APred->AS->window);

		if (APred->AS->grasp==RELEASE)
		{
			labels_predict.push_back("RELEASE");
		}
		else if (APred->AS->pct_err<0)
		{
			labels_predict.push_back(APred->KB->al[APred->AS->label2]);
		}
		else
		{
			labels_predict.push_back("MOVE");
		}

		AParse->Parse(APred->AS);
		AParse->Display(tmpname);

		// Visualize
		if (0)
		{
			VTK = new VTKExtra (loc_int, sec_int);
			vector<Vector4d> point_zero; vector<string> label_zero;
			for(int ii=0;ii<i+1;ii++) point_zero.push_back(pvas[ii][0]);
			vector<vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			VTK->ShowConnectionTest(APred->G, point_zero, label_zero, color_code, true);
			delete VTK;
		}

	}

	// writing results
	{
		string name_tmp = filename_;
		reverse(name_tmp.begin(),name_tmp.end());
		name_tmp.erase(name_tmp.begin()+name_tmp.find("/"),name_tmp.end());
		reverse(name_tmp.begin(),name_tmp.end());
		WF->WriteFilePrediction(
				APred->G, APred->KB, (resultdir_ + name_tmp), labels_parser, labels_predict,
				goals, windows);
		WF->WriteFile_((resultdir_ + "_" + name_tmp), data_writeout);
	}

//	plotData(x,y);

//	vector<string> title; title.resize(1);
//	plotDatas(title, x, pyy);

	// Visualize
	if(0)
	{
		VTK = new VTKExtra (loc_int, sec_int);
		vector<string>goal_action;
		for(int i=0;i<APred->G->GetNumberOfNodes();i++)
		{
			goal_action.push_back(APred->G->GetNode(i).name);
		}
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int ii=0;ii<pvas.size();ii++) {point_zero.push_back(pvas[ii][0]);}
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		vector<int> 	loc_idx_zero;
		VTK->ShowData(
				point_zero, goal_action, APred->KB->al,
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
		VTK->ShowData(
				point_zero, goal_action, APred->KB->al,
				loc_idx_zero, color_code, true, false, false);
		delete VTK;
	}

	// Visualize
	if (0)
	{
		VTK = new VTKExtra (loc_int, sec_int);
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int i=0;i<APred->G->GetNumberOfNodes();i++)
		{
			label_zero.push_back(APred->G->GetNode(i).name);
		}
		for(int ii=0;ii<pvas.size();ii++) point_zero.push_back(pvas[ii][0]);
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnection(APred->G, point_zero, label_zero, color_code, true);
		delete VTK;
	}

	// ****************************************************************** [TEST]

	return EXIT_SUCCESS;
}

