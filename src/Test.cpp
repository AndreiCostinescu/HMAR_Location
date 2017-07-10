/*
 * Test.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Test.h"

Test::Test(
		const std::string &obj_,
		const int &loc_int_,
		const int &sec_int_,
		const int &f_win_,
		std::shared_ptr<CData> cdata_,
		std::shared_ptr<std::vector<std::string> > msg_,
		const std::string &path_,
		bool object_state_)
		: DF(new DataFilter()),
				APred(
						new ActionPrediction(obj_, loc_int_, sec_int_, object_state_)),
				AParse(new ActionParser(obj_, cdata_->KB, msg_, path_)),
				cdata(cdata_),
				RF(new ReadFile()),
				WF(new WriteFile()),
				loc_int(loc_int_),
				sec_int(sec_int_),
				f_win(f_win_)
{
}

Test::~Test()
{
}

//int Test::ReadKB(
//		const std::string &path_)
//{
//	if (RF->ReadFileKB(path_, cdata->KB) == EXIT_FAILURE)
//	{
//		return EXIT_FAILURE;
//	}
//	else
//	{
//		return EXIT_SUCCESS;
//	}
//}
//
//int Test::ReadLA(
//		const std::string &path_)
//{
//	if (RF->ReadFileLA(path_, cdata->KB->AL(), cdata->G) == EXIT_FAILURE)
//	{
//		return EXIT_FAILURE;
//	}
//	else
//	{
//		return EXIT_SUCCESS;
//	}
//}
//
//int Test::ReadGraph(
//		const std::string &path_)
//{
//	if (RF->ReadFileGraph(path_, cdata->G) == EXIT_FAILURE)
//	{
//		return EXIT_FAILURE;
//	}
//	else
//	{
//		return EXIT_SUCCESS;
//	}
//}
//
//int Test::SetMessage(
//		std::shared_ptr<std::vector<std::string> > msg_)
//{
//	AParse->SetMsg(msg_);
//	return EXIT_SUCCESS;
//}
//
//int Test::SetKB(
//		CKB *kb_)
//{
//	*cdata->KB.get() = *kb_;
//	return EXIT_SUCCESS;
//}
//
//int Test::SetOS(
//		COS *os_)
//{
//	*APred->OS = *os_;
//	return EXIT_SUCCESS;
//}

int Test::WriteWindow(
		const std::string &path_)
{
	WF->WriteFileWindow(cdata->G.get(), path_);
	return EXIT_SUCCESS;
}

int Test::ApplyGauss(
		const int &num_x_,
		const int &num_y_)
{
	std::vector<std::vector<double> > k_xy;
	k_xy.resize(num_x_);
	for (int i = 0; i < num_x_; i++)
	{
		k_xy[i].resize(num_x_);
	}
	gaussKernel(k_xy, num_x_, num_x_, 1);

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero;
		std::vector<std::string> label_zero;
		for (int i = 0; i < cdata->G->GetNumberOfNodes(); i++)
		{
			label_zero.push_back(cdata->G->GetNode(i).name);
		}
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(cdata->G.get(), point_zero, label_zero,
				color_code, true);
	}

	for (int i = 0; i < cdata->G->GetNumberOfNodes(); i++)
	{
		for (int ii = 0; ii < cdata->G->GetNumberOfNodes(); ii++)
		{
			if (i == ii)
			{
				continue;
			}

			std::vector<double> sm_tmp1 = cdata->G->GetEdgeSectorMap(i, ii, 0);
			std::vector<double> sm_tmp2 = cdata->G->GetEdgeSectorMap(i, ii, 0);

			for (int l = 0; l < loc_int; l++)
			{
				for (int s = 0; s < sec_int; s++)
				{
					double sum_tmp = 0.0;
					for (int gkx = 0; gkx < num_x_; gkx++)
					{
						for (int gky = 0; gky < num_y_; gky++)
						{
							int tmpl = l - (num_y_ / 2) + gky;
							if (tmpl < 0 || tmpl >= loc_int)
								continue;
							int tmps = (s - (num_x_ / 2) + gkx + sec_int)
									% sec_int;
							sum_tmp += sm_tmp2[tmpl * sec_int + tmps]
									* k_xy[gkx][gky];
						}
					}
					sm_tmp1[l * sec_int + s] = sum_tmp;
				}
			}
			cdata->G->SetEdgeSectorMap(i, ii, 0, sm_tmp1);
		}
	}

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero;
		std::vector<std::string> label_zero;
		for (int i = 0; i < cdata->G.get()->GetNumberOfNodes(); i++)
		{
			label_zero.push_back(cdata->G.get()->GetNode(i).name);
		}
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(cdata->G.get(), point_zero, label_zero,
				color_code, true);
	}

	return EXIT_SUCCESS;
}

int Test::TestInit()
{
	DF->ResetFilter();
	APred->Init(cdata, false);
	AParse->Init(3);
	return EXIT_SUCCESS;
}

int Test::TestFaceAdjust()
{
	face_parser[1] *= -1;
	face_parser += Eigen::Vector4d(-0.1, 0, 0.1, -1);
	// When a graph is present.
	if (cdata->G.get()->GetNumberOfNodes() > 0)
	{
		// Initialize.
		double scale = 1.0;
		Eigen::Vector3d t;
		Eigen::Matrix3d R;
		Eigen::Matrix4d T = Matrix4d::Zero();
		CGraph::node_t old_node;
		CGraph::edge_t edge_tmp;

		// Check for the LA that we intend to change.
		for (int i = 0; i < cdata->G.get()->GetNumberOfNodes(); i++)
		{
			old_node = cdata->G.get()->GetNode(i);
			if (old_node.name == "FACE")
			{
				// 1. Change the SM that has the LA as goal.
				//    The start locations stay the same.
				//    Transform to new target goal.
				//    p' = [S]*[R]*p
				for (int ii = 0; ii < cdata->G.get()->GetNumberOfNodes(); ii++)
				{
					edge_tmp = cdata->G.get()->GetEdge(ii, i, 0);

					if (edge_tmp.counter == 0)
					{
						continue;
					}

					scale =
							V4d3d(
									face_parser
											- cdata->G.get()->GetNode(ii).centroid).norm()
									/ V4d3d(
											old_node.centroid
													- cdata->G.get()->GetNode(
															ii).centroid).norm();
					R =
							rodriguezRot(
									V4d3d(
											old_node.centroid
													- cdata->G.get()->GetNode(
															ii).centroid),
									V4d3d(
											face_parser
													- cdata->G.get()->GetNode(
															ii).centroid));
					T = Matrix4d::Zero();
					T.block<3, 3>(0, 0) = R;
					T(3, 3) = scale;

					for (int j = 0; j < cdata->G.get()->GetLocInt(); j++)
					{
						edge_tmp.tan[j] = R * edge_tmp.tan[j];
						edge_tmp.nor[j] = R.inverse().transpose()
								* edge_tmp.nor[j];
						edge_tmp.loc_mid[j] = T * edge_tmp.loc_mid[j];
						edge_tmp.loc_len[j] = edge_tmp.loc_len[j] * scale;
					}

					cdata->G.get()->SetEdge(ii, i, 0, edge_tmp);
				}

				// 2. Change the SM that has the LA as start.
				//    The goal locations stay the same.
				//    Transform to new origin.
				//    p' = [S]*[R,t]*p
				for (int ii = 0; ii < cdata->G.get()->GetNumberOfNodes(); ii++)
				{
					edge_tmp = cdata->G.get()->GetEdge(i, ii, 0);

					if (edge_tmp.counter == 0)
					{
						continue;
					}

					t = V4d3d(face_parser - old_node.centroid);

					R = rodriguezRot(
							V4d3d(
									cdata->G.get()->GetNode(ii).centroid
											- old_node.centroid),
							V4d3d(
									cdata->G.get()->GetNode(ii).centroid
											- face_parser));
					t += R
							* V4d3d(
									cdata->G.get()->GetNode(ii).centroid
											- old_node.centroid);
					scale =
							V4d3d(
									cdata->G.get()->GetNode(ii).centroid
											- face_parser).norm()
									/ V4d3d(
											cdata->G.get()->GetNode(ii).centroid
													- old_node.centroid).norm();

					T = Matrix4d::Zero();
					T.block<3, 3>(0, 0) = R;
					T(3, 3) = scale;

					for (int j = 0; j < edge_tmp.tan.size(); j++)
					{
						edge_tmp.tan[j] = R * edge_tmp.tan[j];
						edge_tmp.nor[j] = R.inverse().transpose()
								* edge_tmp.nor[j];
						edge_tmp.loc_mid[j] = T * edge_tmp.loc_mid[j];
						edge_tmp.loc_len[j] = edge_tmp.loc_len[j] * scale;
					}

					cdata->G.get()->SetEdge(i, ii, 0, edge_tmp);
				}

				old_node.centroid.head(3) = V4d3d(face_parser);
				cdata->G.get()->SetNode(old_node);
				break;
			}
		}
	}
	return EXIT_SUCCESS;
}

int Test::FilterData(
		const Eigen::Vector4d &pva_in_,
		std::vector<Eigen::Vector4d> &pva_out_,
		const int &contact_in_,
		int &contact_out_)
{
	DF->PreprocessDataLive(pva_in_, pva_out_, f_win);
	DF->PreprocessContactLive(contact_in_, contact_out_, f_win);
	return EXIT_SUCCESS;
}

int Test::Predict()
{
	APred->PredictExt(cdata);
	return EXIT_SUCCESS;
}

int Test::Parser(
		const std::string &filename_)
{
	AParse->Parse(cdata->AS.get());
	AParse->Display(filename_);
	return EXIT_SUCCESS;
}

int Test::WriteResult(
		const std::string &filename_,
		const std::string &resultdir_,
		std::vector<std::string> labels_predict_,
		std::vector<std::vector<double> > data_writeout_,
		std::vector<std::map<std::string, double> > goals_,
		std::vector<std::map<std::string, double> > windows_,
		bool nolabel_)
{
	if (nolabel_)
	{
		WF->WriteFilePrediction(cdata->G.get(), cdata->KB.get(),
				(resultdir_ + filename_), labels_predict_, goals_, windows_);
	}
	else
	{
		WF->WriteFilePrediction(cdata->G.get(), cdata->KB.get(),
				(resultdir_ + filename_), labels_parser, labels_predict_,
				goals_, windows_);
	}
	WF->WriteFile_((resultdir_ + "_" + filename_), data_writeout_);
	return EXIT_SUCCESS;
}

int Test::Testing(
		const std::string &filename_,
		const std::string &resultdir_,
		bool face_)
{
	// [VARIABLES]**************************************************************
	bool nolabel = false;

	std::vector<std::string> labels_predict;
	std::vector<std::vector<Eigen::Vector4d> > pvas;

	std::vector<std::map<std::string, double> > goals;
	std::vector<std::map<std::string, double> > windows;

	std::vector<std::vector<double> > data_writeout;
	// **************************************************************[VARIABLES]

	// [Initialization] ********************************************************
	this->TestInit();

	std::string tmpname = filename_;
	replace(tmpname.begin(), tmpname.end(), '/', '_');

	printer(1);
	// ******************************************************** [Initialization]

	// [READ FILE]**************************************************************
	if (RF->ReadWord(filename_, ',') == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	printer(8);
	// **************************************************************[READ FILE]

	// [PARSE DATA]*************************************************************
	this->ClearParser();
	this->SetDataParser(RF->GetDataWordRF());
//	if (this->ParseData()==EXIT_FAILURE)
//	{
	nolabel = true;
	if (this->ParseDataNoLabel() == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
//	}
	printer(9);
	// *************************************************************[PARSE DATA]

	// [FACE ADJUST]************************************************************
	if (face_)
	{
		this->TestFaceAdjust();
	}
	// *********************************************************** [FACE ADJUST]

	// [TEST] ******************************************************************
	for (int i = 0; i < points_parser.size(); i++)
	{
		// 1. Filter
		this->FilterData(points_parser[i], *(cdata->pva), contact_parser[i],
				*(cdata->contact));

		// 2. Prediction
		this->Predict();

//		if(cdata->AS->label1>=0)
		{
			std::vector<double> tmp;
			tmp.push_back(i);
			tmp.push_back(cdata->AS->Grasp());
			tmp.push_back(cdata->AS->Label1());
			tmp.push_back(cdata->AS->Label2());
			tmp.push_back(cdata->AS->Velocity());
			tmp.push_back(cdata->AS->SurfaceFlag());
			tmp.push_back(
					(checkSurfaceDistance((*(cdata->pva))[0],
							cdata->KB->SurfaceEquation()[0])));
			tmp.push_back((double) ((*(cdata->pva))[0][0]));
			tmp.push_back((double) ((*(cdata->pva))[0][1]));
			tmp.push_back((double) ((*(cdata->pva))[0][2]));
			data_writeout.push_back(tmp);
		}

		pvas.push_back(*(cdata->pva));
		goals.push_back(cdata->AS->Goal());
		windows.push_back(cdata->AS->Window());

		if (cdata->AS->Grasp() == RELEASE)
		{
			labels_predict.push_back("RELEASE");
		}
		else if (cdata->AS->Probability() < 0)
		{
			labels_predict.push_back(
					cdata->KB.get()->AL()[cdata->AS->Label2()]);
		}
		else
		{
			labels_predict.push_back("MOVE");
		}

		this->Parser(tmpname);

		// Visualize
		if (0)
		{
			auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
			std::vector<Eigen::Vector4d> point_zero;
			std::vector<std::string> label_zero;
			for (int ii = 0; ii < i + 1; ii++)
				point_zero.push_back(pvas[ii][0]);
			std::vector<std::vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			VTK->ShowConnectionTest(cdata->G.get(), point_zero, label_zero,
					color_code, true);
		}

	}

	// writing results
	{
		this->WriteResult(tmpname, resultdir_, labels_predict, data_writeout,
				goals, windows, true);
	}

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<std::string> goal_action;
		for (int i = 0; i < cdata->G.get()->GetNumberOfNodes(); i++)
		{
			goal_action.push_back(cdata->G.get()->GetNode(i).name);
		}
		std::vector<Eigen::Vector4d> point_zero;
		std::vector<std::string> label_zero;
		for (int ii = 0; ii < pvas.size(); ii++)
		{
			point_zero.push_back(pvas[ii][0]);
		}
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		std::vector<int> loc_idx_zero;
		VTK->ShowData(point_zero, goal_action, cdata->KB.get()->AL(),
				loc_idx_zero, color_code, true, false, false);

		for (int ii = 0; ii < pvas.size(); ii++)
		{
			if (!strcmp(labels_parser[ii].c_str(), "MOVE"))
				point_zero[ii][3] = -1;
			else if (!strcmp(labels_parser[ii].c_str(), "RELEASE"))
				point_zero[ii][3] = -1;
			else
				point_zero[ii][3] = 1;
		}
		VTK->ShowData(point_zero, goal_action, cdata->KB.get()->AL(),
				loc_idx_zero, color_code, true, false, false);
	}

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero;
		std::vector<std::string> label_zero;
		for (int i = 0; i < cdata->G.get()->GetNumberOfNodes(); i++)
		{
			label_zero.push_back(cdata->G.get()->GetNode(i).name);
		}
		for (int ii = 0; ii < pvas.size(); ii++)
			point_zero.push_back(pvas[ii][0]);
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnection(cdata->G.get(), point_zero, label_zero, color_code,
				true);
	}

	// ****************************************************************** [TEST]

	return EXIT_SUCCESS;
}

