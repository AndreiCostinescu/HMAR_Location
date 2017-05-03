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

void TestCase::Choose(int x_, int y_)
{
	vector<int> idxs;
	int tar = x_;

	switch(x_)
	{
		case 1:
		{
			int idx = 1; // start with 001
			idxs.push_back(idx);
			break;
		}
		case 2:
		{
			int idx = 2;
			idxs.push_back(idx);
			break;
		}
		case 3:
		{
			for(int idx=3;idx<9;idx++)
			{
				idxs.push_back(idx);
			}
			break;
		}
		case 4:
		{
			int idx = 9;
			idxs.push_back(idx);
			break;
		}
		case 5:
		{
			tar = 3;
			int idx = 3;
			idxs.push_back(idx);
			break;
		}
		default:
			break;
	}

	switch(y_)
	{
		case LBL: this->Lbl(idxs);			break;
		case TRN: this->Trn(idxs,dict[tar]); break;
		case TST: this->Tst(idxs,dict[tar]); break;
		case DPL: this->Dpl(idxs,dict[tar]); break;
		default :							break;
	}
}

//=============================================================================
// TEST CASES
//=============================================================================

int TestCase::Trn(vector<int> idx_, string object_)
{

	string tar = object_;

	bool flag;
	int f, ff, fff, res, n, i, ii;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label_list;
	pair<int,string> pair_tmp(-1,"");

	kb_t kb;
	Graph *G;

	printf("==============================================================================\n");
	printf("|| TRN START                                                                ||\n");
	printf("==============================================================================\n");

	// Reading label
	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	if (this->ReadFileKB("./kb/", kb)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading data files
	file_list.clear();
	this->ReadFileName(DATA_DIR, idx_, file_list, n);

	this->DirectoryCheck(EVAL + "/" + tar);

	printf("==============================================================================\n");
	printf("|| TRAINNING                                                                ||\n");
	printf("==============================================================================\n");

	// Subject
	for(f=0;f<n;f++)
	{
		G = new Graph(EVAL, tar);
		G->setKB(kb);

		vector<string> al_tmp; al_tmp = kb.al;
		vector<int> al_tmp_idx; al_tmp_idx.resize(al_tmp.size());

		for(ff=0;ff<n;ff++)
		{
			if(f == ff) continue;

			// Data per subject
			for(fff=0;fff<file_list[ff].size();fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = EVAL + "/" + tar + "/" + to_string(f);
				this->DirectoryCheck(dir_s);

				// read available location areas
				path = dir_s + "/location_area.txt";
				this->ReadFileLA(G,path);

				// action filename pair
				pair_tmp = file_list[ff][fff];

				// for initial labelling
				if (1)
				{
					for(i=0;i<al_tmp.size();i++)
					{
						if (al_tmp_idx[i]>0) continue;
						for(ii=0;ii<label_list[pair_tmp.first].size();ii++)
						{
							if (!strcmp(al_tmp[i].c_str(),label_list[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				res = this->Learning(pair_tmp.second, G, flag);

				if (res==EXIT_FAILURE)	{ printer(29); return EXIT_FAILURE;	}
				else					{ printer(30);						}

				if(G->getNumberOfNodes()>0)
				{
					remove(path.c_str());
					this->WriteFileLA(G, path.c_str());
				}
			}
		}

		path = 	EVAL + "/" + tar + "/" + to_string(f) + "/graph.txt";
		this->WriteFileGraph(G, path);
		path = 	EVAL + "/" + tar + "/" + to_string(f) + "/window.txt";
		this->WriteFileWindow(G,path);

		delete G;
	}

	printf("==============================================================================\n");
	printf("|| TRN END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Tst(vector<int> idx_, string object_)
{
	int sub_num;

	string tar = object_;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label_list;

	kb_t kb;
	Graph *G;

	int num_x = 5;
	int num_y = 1;
	vector<vector<double> > k_xy; k_xy.resize(num_x);
	for(int i=0;i<num_x;i++) { k_xy[i].resize(num_y); }
	gaussKernel(k_xy, num_x, num_y, 1);
//cout << k_xy[0][0] << " ";
//cout << k_xy[1][0] << " ";
//cout << k_xy[2][0] << endl;


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
	if (this->ReadFileKB("./kb/", kb)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading data files
	file_list.clear();
	this->ReadFileName(DATA_DIR, idx_, file_list, sub_num);

	this->DirectoryCheck(EVAL + "/" + tar);

	printf("==============================================================================\n");
	printf("|| TESTING                                                                  ||\n");
	printf("==============================================================================\n");

	// Subject
	for(int f=0;f<sub_num;f++)
	{

		G = new Graph(EVAL, tar);
		G->setKB(kb);

		// read available location areas
		dir_s = EVAL + "/" + tar + "/" + to_string(f);
		path  = dir_s + "/location_area.txt";
		if (this->ReadFileLA(G,path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
		path = 	dir_s + "/graph.txt";
		if (this->ReadFileGraph(G,path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
//		path = 	dir_s + "/window.txt";
//		this->WriteFileWindow(G,path);

		// Visualize
		if (0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			for(int i=0;i<G->getNumberOfNodes();i++)
			{
				node_tt node_tmp = {};
				G->getNode(i, node_tmp);
				label_zero.push_back(node_tmp.name);
			}
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnectionTest(G, point_zero, label_zero, color_code, true);
		}

		if(1)
		{
			for(int i=0;i<G->getNumberOfNodes();i++)
			{
				for(int ii=0;ii<G->getNumberOfNodes();ii++)
				{
					if (i==ii) { continue; }

					reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
					G->getEdgeSectorMap(i,ii,0,sm_tmp1);
					sm_tmp2 = sm_tmp1;

					for(int l=0;l<LOC_INT;l++)
					{
						for(int s=0;s<SEC_INT;s++)
						{
							sum_tmp = 0.0;
							for(int gkx=0;gkx<num_x;gkx++)
							{
								sum_tmp +=
										sm_tmp2
										[l*SEC_INT +
										 ((s-(num_x/2)+gkx+SEC_INT)%SEC_INT)] *
										k_xy[gkx][0];
							}

							sm_tmp1[l*SEC_INT+s] = sum_tmp;
						}
					}

					G->setEdgeSectorMap(i,ii,0,sm_tmp1);
				}
			}
		}

		// Visualize
		if (0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			for(int i=0;i<G->getNumberOfNodes();i++)
			{
				node_tt node_tmp = {};
				G->getNode(i, node_tmp);
				label_zero.push_back(node_tmp.name);
			}
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnectionTest(G, point_zero, label_zero, color_code, true);
		}


		dir_s = RESULT + "/" + tar + "/";
		this->DirectoryCheck(dir_s);
		dir_s = RESULT + "/" + tar + "/" + to_string(f) + "/";
		this->DirectoryCheck(dir_s);

		// Data per subject
		for(int ff=0;ff<file_list[f].size();ff++)
		{
			// action filename pair
			// pair_tmp = file_list[ff][fff];

			char num[4]; sprintf(num,"%03d",file_list[f][ff].first);

//			if(strcmp(num,"005")) continue;

			path = dir_s + string(num) + "/";
			this->DirectoryCheck(path);

			if (this->Testing(file_list[f][ff].second, path, G)==EXIT_FAILURE)
			{
				printer(43);
				return EXIT_FAILURE;
			}
			else
			{
				printer(44);
			}
		}

		delete G;
	}

	printf("==============================================================================\n");
	printf("|| TST END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Dpl(vector<int> idx_, string object_)
{
	int sub_num;

	string tar = object_;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label_list;

	kb_t kb;
	Graph *G;

	int num_x = 5;
	int num_y = 1;
	vector<vector<double> > k_xy; k_xy.resize(num_x);
	for(int i=0;i<num_x;i++) { k_xy[i].resize(num_y); }
	gaussKernel(k_xy, num_x, num_y, 1);

	vector<double> sm_tmp1; reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
	vector<double> sm_tmp2; reshapeVector(sm_tmp2, LOC_INT*SEC_INT);
	double sum_tmp = 0.0;

	printf("==============================================================================\n");
	printf("|| DPL START                                                                ||\n");
	printf("==============================================================================\n");

	// Reading label
	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	if (this->ReadFileKB("./kb/", kb)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading data files
	file_list.clear();
	this->ReadFileName(DATA_DIR, idx_, file_list, sub_num);

	this->DirectoryCheck(EVAL + "/" + tar);

	printf("==============================================================================\n");
	printf("|| Deploying                                                                ||\n");
	printf("==============================================================================\n");

	// Subject
	for(int f=0;f<sub_num;f++)
	{

		G = new Graph(EVAL, tar);
		G->setKB(kb);

		// read available location areas
		dir_s = EVAL + "/" + tar + "/" + to_string(f);
		path  = dir_s + "/location_area.txt";
		if (this->ReadFileLA(G,path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
		path = 	dir_s + "/graph.txt";
		if (this->ReadFileGraph(G,path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
//		path = 	dir_s + "/window.txt";
//		this->WriteFileWindow(G,path);

		if(1)
		{
			for(int i=0;i<G->getNumberOfNodes();i++)
			{
				for(int ii=0;ii<G->getNumberOfNodes();ii++)
				{
					if (i==ii) { continue; }

					reshapeVector(sm_tmp1, LOC_INT*SEC_INT);
					G->getEdgeSectorMap(i,ii,0,sm_tmp1);
					sm_tmp2 = sm_tmp1;

					for(int l=0;l<LOC_INT;l++)
					{
						for(int s=0;s<SEC_INT;s++)
						{
							sum_tmp = 0.0;
							for(int gkx=0;gkx<num_x;gkx++)
							{
								sum_tmp +=
										sm_tmp2
										[l*SEC_INT +
										 ((s-(num_x/2)+gkx+SEC_INT)%SEC_INT)] *
										k_xy[gkx][0];
							}

							sm_tmp1[l*SEC_INT+s] = sum_tmp;
						}
					}

					G->setEdgeSectorMap(i,ii,0,sm_tmp1);
				}
			}
		}


		dir_s = RESULT + "/" + tar + "/";
		this->DirectoryCheck(dir_s);
		dir_s = RESULT + "/" + tar + "/" + to_string(f) + "/";
		this->DirectoryCheck(dir_s);

		// Data per subject
		for(int ff=0;ff<file_list[f].size();ff++)
		{
			// action filename pair
			// pair_tmp = file_list[ff][fff];

			char num[4]; sprintf(num,"%03d",file_list[f][ff].first);
			path = dir_s + string(num) + "/";
			this->DirectoryCheck(path);

			if (this->Deploying(file_list[f][ff].second, path, G)==EXIT_FAILURE)
			{
				printer(43);
				return EXIT_FAILURE;
			}
			else
			{
				printer(44);
			}
		}

		delete G;
	}

	printf("==============================================================================\n");
	printf("|| DPL END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Lbl(vector<int> idx_)
{

	int f, fff, res, n, b, e; f=fff=res=n=b=e=0;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<string, string> label_ref_list;
	vector<point_d> labels_ref;
	vector<string> labels_ref_name;
	map<int,vector<string> > label_list;

	printf("==============================================================================\n");
	printf("|| LBL START                                                                ||\n");
	printf("==============================================================================\n");

	// Reading label_list
	if (this->ReadFileLabel("./kb/", label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	label_ref_list.clear();
	this->ReadLabelFileName("./label/", label_ref_list);

	typedef map<string, string>::iterator it_type;
	for(it_type itr=label_ref_list.begin();itr!=label_ref_list.end();itr++)
	{
		b = e;

		if (this->ReadFile_(itr->second, ',')==EXIT_FAILURE)
		{ return EXIT_FAILURE; } printer(47);

		e = b + this->GetDataRF().size();

		this->SetDataParser(this->GetDataRF());
		this->ParseDataNoLabel();
		vector<point_d> tmp = this->GetPointParser();
		labels_ref.insert(labels_ref.end(),tmp.begin(),tmp.end());

		for(int i=b;i<e;i++) { labels_ref_name.push_back(itr->first); }
	}

	// Reading data files
	file_list.clear();
	this->ReadFileName(DATA_DIR, idx_, file_list, n);

	// Subject
	for(f=0;f<n;f++)
	{
		// Data per subject
		for(fff=0;fff<file_list[f].size();fff++)
		{

			if (this->ReadFile_(file_list[f][fff].second, ',')==EXIT_FAILURE)
			{ return EXIT_FAILURE; } printer(8);

			this->SetDataParser(this->GetDataRF());
			this->ParseDataNoLabel();

			vector<point_d> p_tmp = this->GetPointParser();
			this->RewriteDataFile(
				file_list[f][fff].second, this->GetDataRF(), p_tmp,
				this->GetContactParser(), this->GetFaceParser(),
				labels_ref, labels_ref_name, label_list[file_list[f][fff].first]);

			printer(48);

			// Visualize
			if (0)
			{
				vector<vector<unsigned char> > color_code; colorCode(color_code);
				vector<string>  goal_action, al; goal_action.resize(5);
				vector<int> 	loc_idx_zero;
				showData(
						p_tmp, goal_action, al,
						loc_idx_zero, color_code, true, false, false);
			}
		}
	}

	printf("==============================================================================\n");
	printf("|| LBL END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}
