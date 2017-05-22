/*
 * Train.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Train.h"

Train::Train() : 	VTK(NULL),
					RF(NULL),
					WF(NULL),
					DF(NULL),
					pvas(NULL),
					contacts(NULL),
					loc_int(-1),
					sec_int(-1),
					f_win(-1)
{
}
Train::~Train() { }

int Train::Init(
		int loc_int_,
		int sec_int_,
		int f_win_)
{
	this->InitLA(loc_int_,sec_int_,f_win_);
	this->InitSM(loc_int_,sec_int_,f_win_);
	loc_int = loc_int_;
	sec_int = sec_int_;
	f_win 	= f_win_;
	return EXIT_SUCCESS;
}

int Train::Learning(
		CGraph *G_,
		CKB *KB_,
		string filename_,
		string path_LA_,
		bool flag_)
{
	// [VARIABLES]*************************************************************
	RF = new ReadFile;
	pvas = new vector<vector<Vector4d> >; // length->motion
	contacts = new vector<int>;
	vector<Vector4d> pva; pva.clear(); pva.resize(3);
	int contact;
	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	if (RF->ReadFile_(filename_,',')==EXIT_FAILURE)
	{ return EXIT_FAILURE;	} printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	this->ClearParser();
	this->SetDataParser(RF->GetDataRF());
	if (this->ParseDataNoLabel()==EXIT_FAILURE) {return EXIT_FAILURE;}
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	if (0)
	{
		face_parser -= Vector4d(0,0.15,0,1);
		// When a graph is present.
		if(G_->GetNumberOfNodes()>0)
		{
			// Initialize.
			double scale = 1.0;
			Vector3d t;
			Matrix3d R;
			Matrix4d T = Matrix4d::Zero();
			CGraph::node_t old_node;
			CGraph::edge_t edge_tmp;

			// Check for the LA that we intend to change.
			for(int i=0;i<G_->GetNumberOfNodes();i++)
			{
				old_node = G_->GetNode(i);
				if (!strcmp(old_node.name.c_str(),"FACE"))
				{
					// 1. Change the SM that has the LA as goal.
					//    The start locations stay the same.
					//    Transform to new target goal.
					//    p' = [S]*[R]*p
					for(int ii=0;ii<G_->GetNumberOfNodes();ii++)
					{
						edge_tmp = G_->GetEdge(ii,i,0);

						if (edge_tmp.counter==0) { continue; }

						scale =
								V4d3d(face_parser		- G_->GetNode(ii).centroid).norm() /
								V4d3d(old_node.centroid - G_->GetNode(ii).centroid).norm();
						R =
								rodriguezRot(
										V4d3d(old_node.centroid - G_->GetNode(ii).centroid),
										V4d3d(face_parser		- G_->GetNode(ii).centroid));
						T = Matrix4d::Zero();
						T.block<3,3>(0,0) = R;
						T(3,3) = scale;

						for(int j=0;j<loc_int;j++)
						{
							edge_tmp.tan[j] = R*edge_tmp.tan[j];
							edge_tmp.nor[j] = R.inverse().transpose()*edge_tmp.nor[j];
							edge_tmp.loc_mid[j] = T*edge_tmp.loc_mid[j];
							edge_tmp.loc_len[j] = edge_tmp.loc_len[j]*scale;
						}

						G_->SetEdge(ii,i,0,edge_tmp);
					}

					// 2. Change the SM that has the LA as start.
					//    The goal locations stay the same.
					//    Transform to new origin.
					//    p' = [S]*[R,t]*p
					for(int ii=0;ii<G_->GetNumberOfNodes();ii++)
					{
						edge_tmp = G_->GetEdge(i,ii,0);

						if (edge_tmp.counter==0) { continue; }

						t = V4d3d(face_parser - old_node.centroid);

						R =
								rodriguezRot(
										V4d3d(G_->GetNode(ii).centroid - old_node.centroid),
										V4d3d(G_->GetNode(ii).centroid - face_parser		 ));
						t += R*V4d3d(G_->GetNode(ii).centroid - old_node.centroid);
						scale =
								V4d3d(G_->GetNode(ii).centroid - face_parser).norm() /
								V4d3d(G_->GetNode(ii).centroid - old_node.centroid).norm();

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

						G_->SetEdge(i,ii,0,edge_tmp);
					}

					old_node.centroid.head(3) = V4d3d(face_parser);
					G_->SetNode(old_node);
					break;
				}
			}
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [PREPROCESS DATA] ******************************************************
	DataFilter *DF = new DataFilter;
	DF->ResetFilter();
	for(int ii=0;ii<points_parser.size();ii++)
	{
		DF->PreprocessDataLive(points_parser[ii], pva, f_win);
		DF->PreprocessContactLive(contact_parser[ii], contact, f_win);
		pvas->push_back(pva);
		contacts->push_back(contact);
	}
	delete DF;
	printer(10);
	// ****************************************************** [PREPROCESS DATA]

	// [LOCATION AND SECTOR-MAP]***********************************************
	this->ClearLA();
	this->BuildLocationArea(G_, KB_, *pvas, contacts, flag_);
	printer(11);
	this->ClearSM();
	this->BuildSectorMap   (G_, KB_, pvas, contacts);
	printer(12);
	// ***************************************[LOCATION AND SECTOR-MAP]

	// Visualize
	if (0)
	{
		VTK = new VTKExtra (loc_int, sec_int);
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int i=0;i<G_->GetNumberOfNodes();i++)
		{ label_zero.push_back(G_->GetNode(i).name); }
//		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(G_, point_zero, label_zero, color_code, true);
		delete VTK;
	}

	delete RF;
	delete pvas;
	delete contacts;

	return EXIT_SUCCESS;
}
