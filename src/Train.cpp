/*
 * Train.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Train.h"

Train::Train(
		const int &loc_int_,
		const int &sec_int_,
		const int &f_win_)
		: TrainLA(loc_int_, sec_int_),
				TrainSM(loc_int_, sec_int_),
				DF(new DataFilter()),
				RF(new ReadFile()),
				WF(new WriteFile()),
				loc_int(loc_int_),
				sec_int(sec_int_),
				f_win(f_win_)
{
}

Train::~Train()
{
}

int Train::Learning(
		std::shared_ptr<CGraph> G_,
		std::shared_ptr<CKB> KB_,
		const std::string &filename_,
		bool new_label_,
		bool face_)
{
	// [VARIABLES]*************************************************************
	std::vector<Eigen::Vector4d> pva(3, Eigen::Vector4d::Zero());
	int contact;
	printer(1);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	if (RF->ReadWord(filename_, ',') == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	this->ClearParser();
	this->SetDataParser(RF->GetDataWordRF());
	if (this->ParseDataNoLabel() == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	printer(9);
	// ************************************************************[PARSE DATA]

	// [FACE ADJUST]***********************************************************
	if (face_)
	{
//		face_parser -= Eigen::Vector4d(0, 0.15, 0, 1);
		face_parser[1] *= -1;
		face_parser += Eigen::Vector4d(-0.1, 0, 0.1, -1);
		// When a graph is present.
		if (G_->GetNumberOfNodes() > 0)
		{
			// Initialize.
			double scale = 1.0;
			Eigen::Vector3d t;
			Eigen::Matrix3d R;
			Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
			CGraph::node_t node1;
			CGraph::node_t node2;
			CGraph::edge_t edge;

			// Check for the LA that we intend to change.
			for (int i = 0; i < G_->GetNumberOfNodes(); i++)
			{
				node1 = G_->GetNode(i);

				if (!strcmp(node1.name.c_str(), "FACE"))
				{
					// 1. Change the SM that has the LA as goal.
					//    The start locations stay the same.
					//    Transform to new target goal.
					//    p' = [S]*[R]*p
					for (int ii = 0; ii < G_->GetNumberOfNodes(); ii++)
					{
						node2 = G_->GetNode(ii);
						edge = G_->GetEdge(ii, i, 0);

						if (edge.counter == 0)
						{
							continue;
						}

						scale = V4d3d(face_parser - node2.centroid).norm()
								/ V4d3d(node1.centroid - node2.centroid).norm();
						R = rodriguezRot(V4d3d(node1.centroid - node2.centroid),
								V4d3d(face_parser - node2.centroid));

						// Transformation matrix.
						T = Eigen::Matrix4d::Zero();
						T.block<3, 3>(0, 0) = R;
						T(3, 3) = scale;

						for (int j = 0; j < loc_int; j++)
						{
							edge.tan[j] = R * edge.tan[j];
							edge.nor[j] = R.inverse().transpose() * edge.nor[j];
							edge.loc_mid[j] = T * edge.loc_mid[j];
							edge.loc_len[j] = edge.loc_len[j] * scale;
						}

						G_->SetEdge(ii, i, 0, edge);
					}

					// 2. Change the SM that has the LA as start.
					//    The goal locations stay the same.
					//    Transform to new origin.
					//    p' = [S]*[R,t]*p
					for (int ii = 0; ii < G_->GetNumberOfNodes(); ii++)
					{
						node2 = G_->GetNode(ii);
						edge = G_->GetEdge(i, ii, 0);

						if (edge.counter == 0)
						{
							continue;
						}

						scale = V4d3d(node2.centroid - face_parser).norm()
								/ V4d3d(node2.centroid - node1.centroid).norm();
						R = rodriguezRot(V4d3d(node2.centroid - node1.centroid),
								V4d3d(node2.centroid - face_parser));
						t = V4d3d(face_parser - node1.centroid);
						t += R * V4d3d(node2.centroid - node1.centroid);

						// Transformation matrix.
						T = Eigen::Matrix4d::Zero();
						T.block<3, 3>(0, 0) = R;
						T(3, 3) = scale;

						for (int j = 0; j < edge.tan.size(); j++)
						{
							edge.tan[j] = R * edge.tan[j];
							edge.nor[j] = R.inverse().transpose() * edge.nor[j];
							edge.loc_mid[j] = T * edge.loc_mid[j];
							edge.loc_len[j] = edge.loc_len[j] * scale;
						}

						G_->SetEdge(i, ii, 0, edge);
					}

					node1.centroid.head(3) = V4d3d(face_parser);
					G_->SetNode(node1);
					break;
				}
			}
		}
	}
	// ********************************************************** [FACE ADJUST]

	// [PREPROCESS DATA] ******************************************************
	auto pvas = std::make_shared<std::vector<std::vector<Eigen::Vector4d> > >(); // length->motion
	auto contacts = std::make_shared<std::vector<int> >();

	DF->ResetFilter();
	for (int ii = 0; ii < points_parser.size(); ii++)
	{
		DF->PreprocessDataLive(points_parser[ii], pva, f_win);
		DF->PreprocessContactLive(contact_parser[ii], contact, f_win);
		pvas->push_back(pva);
		contacts->push_back(contact);
	}
	printer(10);
	// ****************************************************** [PREPROCESS DATA]

	// [LOCATION AND SECTOR-MAP]***********************************************
	this->ClearLA();
	this->BuildLocationArea(G_, KB_, pvas, contacts, new_label_);
	printer(11);
	this->ClearSM();
	this->BuildSectorMap(G_, KB_, pvas, contacts);
	printer(12);
	// ***************************************[LOCATION AND SECTOR-MAP]

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero;
		std::vector<std::string> label_zero;
		for (int i = 0; i < G_->GetNumberOfNodes(); i++)
		{
			label_zero.push_back(G_->GetNode(i).name);
		}
//		for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(G_.get(), point_zero, label_zero, color_code,
				true);
	}

	return EXIT_SUCCESS;
}
