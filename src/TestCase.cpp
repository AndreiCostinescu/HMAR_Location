/*
 * TestCase.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#include "TestCase.h"

//=============================================================================
// CONSTUCTOR / DESTRUCTOR
//=============================================================================

TestCase::TestCase()
{

#ifdef PC
	EVAL		= "../Scene1";
	RESULT 		= "../Result";
	DATA_DIR 	= "../recording";
#else
	EVAL 		= "Scene1";
	RESULT 		= "Result";
	DATA_DIR	= "recording";
#endif

	dict[1] = "CUP";
	dict[2] = "ORG";
	dict[3] = "SPG";
	dict[4] = "KNF";

printf("==============================================================================\n");
printf("|| TEST CASE INITIALIZED                                                    ||\n");
printf("==============================================================================\n");
}

TestCase::~TestCase()
{
printf("==============================================================================\n");
printf("|| TEST CASE ENDED                                                          ||\n");
printf("==============================================================================\n");
}


//=============================================================================
// CHOOSING
//=============================================================================

void TestCase::Choose(int x_)
{
	vector<int> idxs;

//	0 	: labelling of dataSet
//	1-4 : training
//	5-8 : testing

	switch(x_)
	{
		case 0:
		{
			// start with 001
			for(int i=1;i<10;i++) idxs.push_back(i);
			this->Lbl(idxs);
			break;
		}
		case 1:
		{
			// start with 001
			int idx = 1;
			idxs.push_back(idx);
			this->TrnInd(idxs, dict[1]);
			break;
		}
		case 2:
		{
			int idx = 2;
			idxs.push_back(idx);
			this->TrnInd(idxs, dict[2]);
			break;
		}
		case 3:
		{
			for(int idx=3;idx<9;idx++)
			{
				idxs.push_back(idx);
			}
			this->TrnInd(idxs, dict[3]);
			break;
		}
		case 4:
		{
			int idx = 9;
			idxs.push_back(idx);
			this->TrnInd(idxs, dict[4]);
			break;
		}
		case 5:
		{
			// start with 001
			int idx = 1;
			idxs.push_back(idx);
			this->Tst(idxs, dict[1]);
			break;
		}
		case 6:
		{
			int idx = 2;
			idxs.push_back(idx);
			this->Tst(idxs, dict[2]);
			break;
		}
		case 7:
		{
			for(int idx=3;idx<9;idx++)
			{
				idxs.push_back(idx);
			}
			this->Tst(idxs, dict[3]);
			break;
		}
		case 8:
		{
			int idx = 9;
			idxs.push_back(idx);
			this->Tst(idxs, dict[4]);
			break;
		}
		case 9:
		{
			EVAL = "Scene2";
			// start with 001
			int idx = 1;
			idxs.push_back(idx);
			this->TrnIndLA(idxs, dict[1]);
			break;
		}
		case 10:
		{
			EVAL = "Scene2";
			int idx = 2;
			idxs.push_back(idx);
			this->TrnIndLA(idxs, dict[2]);
			break;
		}
		case 11:
		{
			EVAL = "Scene2";
			for(int idx=3;idx<9;idx++)
			{
				idxs.push_back(idx);
			}
			this->TrnIndLA(idxs, dict[3]);
			break;
		}
		case 12:
		{
			EVAL = "Scene2";
			int idx = 9;
			idxs.push_back(idx);
			this->TrnIndLA(idxs, dict[4]);
			break;
		}
		case 13:
		{
			EVAL = "Scene2";
			// start with 001
			int idx = 1;
			idxs.push_back(idx);
			this->Tst(idxs, dict[1]);
			break;
		}
		case 14:
		{
			EVAL = "Scene2";
			int idx = 2;
			idxs.push_back(idx);
			this->Tst(idxs, dict[2]);
			break;
		}
		case 15:
		{
			EVAL = "Scene2";
			for(int idx=3;idx<9;idx++)
			{
				idxs.push_back(idx);
			}
			this->Tst(idxs, dict[3]);
			break;
		}
		case 16:
		{
			EVAL = "Scene2";
			int idx = 9;
			idxs.push_back(idx);
			this->Tst(idxs, dict[4]);
			break;
		}
		default:
			break;
	}
}

//=============================================================================
// TEST CASES
//=============================================================================

int TestCase::TrnInd(vector<int> idx_, string object_)
{
	bool flag;
	int n=0;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label_list;
	pair<int,string> pair_tmp(-1,"");

	printf("==============================================================================\n");
	printf("|| TrnInd START                                                             ||\n");
	printf("==============================================================================\n");

	// Reading label
	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	if (this->ReadFileKB("./kb/", KB)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading data files
	if (this->ReadFileName(DATA_DIR, idx_, file_list, n)==EXIT_FAILURE)
	{ return EXIT_FAILURE; } printer(26);

	this->DirectoryCheck(EVAL + "/" + object_);

	// Subject
	for(int f=0;f<n;f++)
	{
		G = new Graph(object_);

		vector<int> al_tmp_idx; al_tmp_idx.resize(KB.al.size());

		for(int ff=0;ff<n;ff++)
		{
			if (f == ff) continue;

			// Data per subject
			for(int fff=0;fff<file_list[ff].size();fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = EVAL + "/" + object_ + "/" + to_string(f);
				this->DirectoryCheck(dir_s);

				// read available location areas
				path = dir_s + "/location_area.txt";
				this->ReadFileLA(G,KB.al,path);

				// action filename pair
				pair_tmp = file_list[ff][fff];

				// for initial labelling
				if (1)
				{
					for(int i=0;i<KB.al.size();i++)
					{
						if (al_tmp_idx[i]>0) continue;
						for(int ii=0;ii<label_list[pair_tmp.first].size();ii++)
						{
							if (!strcmp(KB.al[i].c_str(),label_list[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				// learning process
				if (this->Learning(pair_tmp.second, path, flag)==EXIT_FAILURE)
				{ printer(29); return EXIT_FAILURE;	} printer(30);

				// writing location areas data
				if(G->GetNumberOfNodes()>0)
				{
					remove(path.c_str());
					this->WriteFileLA(G, KB, path.c_str());
				}
			}
		}

		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/graph.txt";
		this->WriteFileGraph(G, path);
		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/window.txt";
		this->WriteFileWindow(G,path);

		delete G; G = NULL;
	}

	printf("==============================================================================\n");
	printf("|| TrnInd END                                                               ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::TrnIndLA(vector<int> idx_, string object_)
{
	bool flag;
	int n=0;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label_list;
	pair<int,string> pair_tmp(-1,"");

	printf("==============================================================================\n");
	printf("|| TrnIndLA START                                                           ||\n");
	printf("==============================================================================\n");

	// Reading label
	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	if (this->ReadFileKB("./kb/", KB)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading data files
	if (this->ReadFileName(DATA_DIR, idx_, file_list, n)==EXIT_FAILURE)
	{ return EXIT_FAILURE; } printer(26);

	this->DirectoryCheck(EVAL + "/" + object_);

	// Subject
	for(int f=0;f<n;f++)
	{
		G = new Graph(object_);

		vector<int> al_tmp_idx; al_tmp_idx.resize(KB.al.size());

		for(int ff=0;ff<n;ff++)
		{
			if (f == ff) continue;

			// Data per subject
			for(int fff=0;fff<file_list[ff].size();fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = EVAL + "/" + object_ + "/" + to_string(f);
				this->DirectoryCheck(dir_s);

				// read available location areas
				path = dir_s + "/location_area.txt";
				this->ReadFileLA(G,KB.al,path);

				// action filename pair
				pair_tmp = file_list[ff][fff];

				// for initial labelling
				if (1)
				{
					for(int i=0;i<KB.al.size();i++)
					{
						if (al_tmp_idx[i]>0) continue;
						for(int ii=0;ii<label_list[pair_tmp.first].size();ii++)
						{
							if (!strcmp(KB.al[i].c_str(),label_list[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				// learning process
				if (this->Learning(pair_tmp.second, path, flag)==EXIT_FAILURE)
				{ printer(29); return EXIT_FAILURE;	} printer(30);

				// writing location areas data
				if(G->GetNumberOfNodes()>0)
				{
					remove(path.c_str());
					this->WriteFileLA(G, KB, path.c_str());
				}
			}
		}

		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/graph.txt";
		this->WriteFileGraph(G, path);
		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/window.txt";
		this->WriteFileWindow(G,path);

		delete G; G = NULL;
	}

	printf("==============================================================================\n");
	printf("|| TrnIndLA END                                                             ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Tst(vector<int> idx_, string object_)
{
	int sub_num;

	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label_list;

	int num_x = 9;
	int num_y = 9;
	vector<vector<double> > k_xy; k_xy.resize(num_x);
	for(int i=0;i<num_x;i++) { k_xy[i].resize(num_y); }
	gaussKernel(k_xy, num_x, num_y, 3);

	vector<double> sm_tmp1; reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
	vector<double> sm_tmp2; reshapeVector(sm_tmp2, LOC_INT*SEC_INT);
	double sum_tmp = 0.0;

	printf("==============================================================================\n");
	printf("|| TST START                                                                ||\n");
	printf("==============================================================================\n");

	// Reading label
	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	if (this->ReadFileKB("./kb/", KB)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading data files
	file_list.clear();
	this->ReadFileName(DATA_DIR, idx_, file_list, sub_num);

	this->DirectoryCheck(EVAL + "/" + object_);

	printf("==============================================================================\n");
	printf("|| TESTING                                                                  ||\n");
	printf("==============================================================================\n");

	// Subject
	for(int f=0;f<sub_num;f++)
	{

		G = new Graph(object_);

		// read available location areas
		dir_s = EVAL + "/" + object_ + "/" + to_string(f);
		path  = dir_s + "/location_area.txt";
		if (this->ReadFileLA(G,KB.al,path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
		path = 	dir_s + "/graph.txt";
		if (this->ReadFileGraph(G,path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}

		// Visualize
		if (0)
		{
			vector<Vector4d> point_zero; vector<string> label_zero;
			for(int i=0;i<G->GetNumberOfNodes();i++)
			{ label_zero.push_back(G->GetNode(i).name); }
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnectionTest(G, point_zero, label_zero, color_code, true);
		}

		if(1)
		{
			for(int i=0;i<G->GetNumberOfNodes();i++)
			{
				for(int ii=0;ii<G->GetNumberOfNodes();ii++)
				{
					if (i==ii) { continue; }

					reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
					G->GetEdgeSectorMap(i,ii,0,sm_tmp1);
					sm_tmp2 = sm_tmp1;

					for(int l=0;l<LOC_INT;l++)
					{
						for(int s=0;s<SEC_INT;s++)
						{
							sum_tmp = 0.0;
							for(int gkx=0;gkx<num_x;gkx++)
							{
								for(int gky=0;gky<num_y;gky++)
								{
									int tmpl = l-(num_y/2)+gky; if(tmpl < 0 || tmpl>=LOC_INT) continue;
									int tmps = (s-(num_x/2)+gkx+SEC_INT)%SEC_INT;
									sum_tmp += sm_tmp2[tmpl*SEC_INT + tmps] * k_xy[gkx][gky];
								}
							}
							sm_tmp1[l*SEC_INT+s] = sum_tmp;
						}
					}
					G->SetEdgeSectorMap(i,ii,0,sm_tmp1);
				}
			}
		}

		// Visualize
		if (0)
		{
			vector<Vector4d> point_zero; vector<string> label_zero;
			for(int i=0;i<G->GetNumberOfNodes();i++)
			{ label_zero.push_back(G->GetNode(i).name); }
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnectionTest(G, point_zero, label_zero, color_code, true);
		}

		path = 	dir_s + "/window.txt";
		this->WriteFileWindow(G,path);

		dir_s = RESULT + "/" + object_ + "/";
		this->DirectoryCheck(dir_s);
		dir_s = RESULT + "/" + object_ + "/" + to_string(f) + "/";
		this->DirectoryCheck(dir_s);

		// Data per subject
		for(int ff=0;ff<file_list[f].size();ff++)
		{
			// action filename pair
			// pair_tmp = file_list[ff][fff];

			char num[4]; sprintf(num,"%03d",file_list[f][ff].first);

//			if(strcmp(num,"006")) continue;

			path = dir_s + string(num) + "/";
			this->DirectoryCheck(path);

			if (this->Testing(file_list[f][ff].second, path)==EXIT_FAILURE)
			{ printer(43); return EXIT_FAILURE; }
			else
			{ printer(44); }
		}

		delete G; G = NULL;
	}

	printf("==============================================================================\n");
	printf("|| TST END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

//int TestCase::Dpl(vector<int> idx_, string object_)
//{
//	int sub_num;
//
//	string dir_s, path;
//	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
//	map<int,vector<string> > label_list;
//
//	int num_x = 5;
//	int num_y = 1;
//	vector<vector<double> > k_xy; k_xy.resize(num_x);
//	for(int i=0;i<num_x;i++) { k_xy[i].resize(num_y); }
//	gaussKernel(k_xy, num_x, num_y, 1);
//
//	vector<double> sm_tmp1; reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
//	vector<double> sm_tmp2; reshapeVector(sm_tmp2, LOC_INT*SEC_INT);
//	double sum_tmp = 0.0;
//
//	printf("==============================================================================\n");
//	printf("|| DPL START                                                                ||\n");
//	printf("==============================================================================\n");
//
//	// Reading label
//	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
//	{ return EXIT_FAILURE; }
//
//	// Reading surface
//	// Reading action labels
//	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
//	// Reading object specific labels
//	// - reads the object specific labels and saves them
//	if (this->ReadFileKB("./kb/", KB)==EXIT_FAILURE)
//	{ return EXIT_FAILURE; }
//
//	// Reading data files
//	file_list.clear();
//	this->ReadFileName(DATA_DIR, idx_, file_list, sub_num);
//
//	this->DirectoryCheck(EVAL + "/" + object_);
//
//	printf("==============================================================================\n");
//	printf("|| Deploying                                                                ||\n");
//	printf("==============================================================================\n");
//
//	// Subject
//	for(int f=0;f<sub_num;f++)
//	{
//
//		G = new Graph(object_);
//
//		// read available location areas
//		dir_s = EVAL + "/" + object_ + "/" + to_string(f);
//		path  = dir_s + "/location_area.txt";
//		if (this->ReadFileLA(G,path)==EXIT_FAILURE)
//		{return EXIT_FAILURE;}
//		path = 	dir_s + "/graph.txt";
//		if (this->ReadFileGraph(G,path)==EXIT_FAILURE)
//		{return EXIT_FAILURE;}
////		path = 	dir_s + "/window.txt";
////		this->WriteFileWindow(G,path);
//
//		if(1)
//		{
//			for(int i=0;i<G->GetNumberOfNodes();i++)
//			{
//				for(int ii=0;ii<G->GetNumberOfNodes();ii++)
//				{
//					if (i==ii) { continue; }
//
//					reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
//					G->GetEdgeSectorMap(i,ii,0,sm_tmp1);
//					sm_tmp2 = sm_tmp1;
//
//					for(int l=0;l<LOC_INT;l++)
//					{
//						for(int s=0;s<SEC_INT;s++)
//						{
//							sum_tmp = 0.0;
//							for(int gkx=0;gkx<num_x;gkx++)
//							{
//								sum_tmp +=
//										sm_tmp2
//										[l*SEC_INT +
//										 ((s-(num_x/2)+gkx+SEC_INT)%SEC_INT)] *
//										k_xy[gkx][0];
//							}
//
//							sm_tmp1[l*SEC_INT+s] = sum_tmp;
//						}
//					}
//
//					G->SetEdgeSectorMap(i,ii,0,sm_tmp1);
//				}
//			}
//		}
//
//
//		dir_s = RESULT + "/" + object_ + "/";
//		this->DirectoryCheck(dir_s);
//		dir_s = RESULT + "/" + object_ + "/" + to_string(f) + "/";
//		this->DirectoryCheck(dir_s);
//
//		// Data per subject
//		for(int ff=0;ff<file_list[f].size();ff++)
//		{
//			// action filename pair
//			// pair_tmp = file_list[ff][fff];
//
//			char num[4]; sprintf(num,"%03d",file_list[f][ff].first);
//			path = dir_s + string(num) + "/";
//			this->DirectoryCheck(path);
//
//			if (this->Testing(file_list[f][ff].second, path)==EXIT_FAILURE)
//			{
//				printer(43);
//				return EXIT_FAILURE;
//			}
//			else
//			{
//				printer(44);
//			}
//		}
//
//		delete G; G = NULL;
//	}
//
//	printf("==============================================================================\n");
//	printf("|| DPL END                                                                  ||\n");
//	printf("==============================================================================\n");
//
//	return EXIT_SUCCESS;
//}

int TestCase::Lbl(vector<int> idx_)
{

	int n, b, e; n=b=e=0;

	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<string, string> label_ref_list;
	vector<Vector3d> labels_ref;
	vector<string> labels_ref_name;
	map<int,vector<string> > label_list;

	ReadFile RF;
	WriteFile WF;
	DataParser P;

	printf("==============================================================================\n");
	printf("|| LABELLING START                                                          ||\n");
	printf("==============================================================================\n");

	// Reading label_list
	if (RF.ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading file names for location references of the labels
	if (RF.ReadLabelFileName("./label/", label_ref_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Parsing the data from the reference files
	typedef map<string, string>::iterator it_type;
	for(it_type itr=label_ref_list.begin();itr!=label_ref_list.end();itr++)
	{
		if (RF.ReadFile_(itr->second, ',')==EXIT_FAILURE)
		{ return EXIT_FAILURE; } printer(47);

		b = e;
		e = b + RF.GetDataRF().size();
		P.SetDataParser(RF.GetDataRF());
		P.ParseDataNoLabel();
		vector<Vector4d> tmp = P.GetPointParser();
		vector<Vector3d> tmp2; tmp2.resize(tmp.size());
		for(int i=0;i<tmp.size();i++) tmp2[i] = V4d3d(tmp[i]);
		labels_ref.insert(labels_ref.end(),tmp2.begin(),tmp2.end());
		for(int i=b;i<e;i++) { labels_ref_name.push_back(itr->first); }
	}

	// Reading data file name
	if (RF.ReadFileName(DATA_DIR, idx_, file_list, n)==EXIT_FAILURE)
	{ return EXIT_FAILURE; } printer(26);

	// Subject
	for(int f=0;f<n;f++)
	{
		// Data per subject
		for(int ff=0;ff<file_list[f].size();ff++)
		{
			if (RF.ReadFile_(file_list[f][ff].second, ',')==EXIT_FAILURE)
			{ return EXIT_FAILURE; } printer(8);

			P.SetDataParser(RF.GetDataRF());
			P.ParseDataNoLabel();
			WF.RewriteDataFile(
				file_list[f][ff].second,
				RF.GetDataRF(),
				P.GetPointParser(),
				P.GetContactParser(),
				P.GetFaceParser(),
				labels_ref,
				labels_ref_name,
				label_list[file_list[f][ff].first]);
			printer(48);

//			// Visualize
//			if (0)
//			{
//				vector<vector<unsigned char> > color_code; colorCode(color_code);
//				vector<string>  goal_action, al; goal_action.resize(5);
//				vector<int> 	loc_idx_zero;
//				showData(
//						p_tmp, goal_action, al,
//						loc_idx_zero, color_code, true, false, false);
//			}
		}
	}

	printf("==============================================================================\n");
	printf("|| LABELLING END                                                            ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}
