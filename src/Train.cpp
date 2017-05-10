/*
 * Train.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Train.h"

Train::Train() { }

Train::~Train() { }

int Train::Learning(
	string filename_,
	string path_LA_,
	bool flag_)
{
	// [VARIABLES]*************************************************************
	vector<vector<Vector4d> > 	pvas; // length->motion
	vector<int>					contacts;
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
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [PREPROCESS DATA] ******************************************************
	this->ResetFilter();
	for(int ii=0;ii<points_parser.size();ii++)
	{
		this->PreprocessDataLive(points_parser[ii], pva, FILTER_WIN);
		this->PreprocessContactLive(contact_parser[ii], contact, FILTER_WIN);
		pvas.push_back(pva);
		contacts.push_back(contact);
	}
	printer(10);
	// ****************************************************** [PREPROCESS DATA]

	// [LOCATION AND SECTOR-MAP]***********************************************
	this->BuildLocationArea(G, KB, pvas, contacts, flag_);
	printer(11);
	this->BuildSectorMap   (G, KB, pvas, contacts);
	printer(12);
	// ***************************************[LOCATION AND SECTOR-MAP]

	// Visualize
	if (0)
	{
		vector<Vector4d> point_zero; vector<string> label_zero;
		for(int i=0;i<G->GetNumberOfNodes();i++)
		{ label_zero.push_back(G->GetNode(i).name); }
//		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnectionTest(G, points_parser, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}
