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

TestCase::TestCase() :
		sub_num(0),
		file_list(NULL),
		label_list(NULL),
		KB(NULL),
		message(NULL),
		G(NULL),
		RF(NULL),
		WF(NULL)
{

#ifdef PC
	EVAL		= "../Scene1";
	EVAL2		= "../Scene2";
	RESULT 		= "../Result";
	RESULT2 	= "../Result2";
	KB_DIR 		= "../kb";
	DATA_DIR 	= "../recording";
#else
	EVAL 		= "Scene1";
	EVAL2 		= "Scene2";
	RESULT 		= "Result";
	RESULT2 	= "Result2";
	KB_DIR		= "kb";
	DATA_DIR	= "recording";
#endif

	dict[1] = "CUP";
	dict[2] = "ORG";
	dict[3] = "SPG";
	dict[4] = "KNF";
	dict[5] = "PT1";
	dict[6] = "PT2";

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

	switch(x_)
	{
		case 0:
		{
			idxs = {1,2,3,4,5,6,7,8,9,10,11};
			this->Lbl(idxs);
			break;
		}
		case 1: // start with 001
		{
			idxs = {1};
			this->TrnIndLA(idxs, dict[1]);
			break;
		}
		case 2:
		{
			idxs = {2};
			this->TrnIndLA(idxs, dict[2]);
			break;
		}
		case 3:
		{
			idxs = {3,4,5,6,7,8};
			this->TrnIndLA(idxs, dict[3]);
			break;
		}
		case 4:
		{
			idxs = {9};
			this->TrnIndLA(idxs, dict[4]);
			break;
		}
		case 5:
		{
			// start with 001
			idxs = {1};
			this->Tst(idxs, dict[1]);
			break;
		}
		case 6:
		{
			idxs = {2};
			this->Tst(idxs, dict[2]);
			break;
		}
		case 7:
		{
			idxs = {3,4,5,6,7,8};
			this->Tst(idxs, dict[3]);
			break;
		}
		case 8:
		{
			idxs = {9};
			this->Tst(idxs, dict[4]);
			break;
		}
		case 9:
		{
			EVAL = EVAL2;
			// start with 001
			idxs = {1};
			this->TrnIndLA(idxs, dict[1]);
			break;
		}
		case 10:
		{
			EVAL = EVAL2;
			idxs = {2};
			this->TrnIndLA(idxs, dict[2]);
			break;
		}
		case 11:
		{
			EVAL = EVAL2;
			idxs = {3,4,5,6,7,8};
			this->TrnIndLA(idxs, dict[3]);
			break;
		}
		case 12:
		{
			EVAL = EVAL2;
			idxs = {9};
			this->TrnIndLA(idxs, dict[4]);
			break;
		}
		case 13:
		{
			RESULT = RESULT2;
			// start with 001
			idxs = {1};
			this->Tst(idxs, dict[1]);
			break;
		}
		case 14:
		{
			RESULT = RESULT2;
			idxs = {2};
			this->Tst(idxs, dict[2]);
			break;
		}
		case 15:
		{
			RESULT = RESULT2;
			idxs = {3,4,5,6,7,8};
			this->Tst(idxs, dict[3]);
			break;
		}
		case 16:
		{
			RESULT = RESULT2;
			idxs = {9};
			this->Tst(idxs, dict[4]);
			break;
		}
		case 17:
		{
			EVAL = EVAL2;
			idxs = {10};
			this->TrnInd(idxs, dict[5]);
			break;
		}
		case 18:
		{
			RESULT = RESULT2;
			idxs = {10};
			this->Tst(idxs, dict[5]);
			break;
		}
		case 19:
		{
			EVAL = EVAL2;
			idxs = {11};
			this->TrnInd(idxs, dict[6]);
			break;
		}
		case 20:
		{
			RESULT = RESULT2;
			idxs = {11};
			this->Tst(idxs, dict[6]);
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
	string dir_s, path;
	pair<int,string> pair_tmp(-1,"");

	printf("==============================================================================\n");
	printf("|| TrnInd   START                                                           ||\n");
	printf("==============================================================================\n");

	file_list 	= new map<int,map<int,pair<int,string> > >;
	label_list	= new map<int,vector<string> >;
	message		= new vector<string>;
	KB			= new CKB;
	RF 			= new ReadFile;
	WF 			= new WriteFile;

	this->ReadFileExt(idx_, object_);

	Train *T;

	// Subject
	for(int f=0;f<sub_num;f++)
	{
		T = new Train;
		T->Init(LOC_INT,SEC_INT,FILTER_WIN);

		G = new CGraph;
		G->SetObject(object_);
		G->SetLocInt(LOC_INT);
		G->SetSecInt(SEC_INT);

		// to check if a LA has already been seen.
		vector<int> al_tmp_idx; al_tmp_idx.resize(KB->al.size());

		for(int ff=0;ff<sub_num;ff++)
		{
			// Data per subject
			for(int fff=0;fff<(*file_list)[ff].size();fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = EVAL + "/" + object_ + "/" + to_string(f);
				directoryCheck(dir_s);

				// read available location areas
				path = dir_s + "/location_area.txt";
				RF->ReadFileLA(G,KB->al,path);

				// action filename pair
				pair_tmp = (*file_list)[ff][fff];

				// for initial labelling
				if (1)
				{
					for(int i=0;i<KB->al.size();i++)
					{
						if (al_tmp_idx[i]>0) continue;
						for(int ii=0;ii<(*label_list)[pair_tmp.first].size();ii++)
						{
							if (!strcmp(KB->al[i].c_str(),(*label_list)[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				// learning process
				if (T->Learning(G, KB, pair_tmp.second, path, flag)==EXIT_FAILURE)
				{ printer(29); return EXIT_FAILURE;	} printer(30);

				// writing location areas data
				if(G->GetNumberOfNodes()>0)
				{
					remove(path.c_str());
					WF->WriteFileLA(G, KB, path.c_str());
				}
			}
		}

		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/graph.txt";
		WF->WriteFileGraph(G, path);
		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/window.txt";
		WF->WriteFileWindow(G,path);

		delete G;
	}

	delete file_list;
	delete label_list;
	delete message;
	delete KB;
	delete RF;
	delete WF;

	printf("==============================================================================\n");
	printf("|| TrnInd  END                                                              ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::TrnIndLA(vector<int> idx_, string object_)
{
	bool flag;
	string dir_s, path;
	pair<int,string> pair_tmp(-1,"");

	printf("==============================================================================\n");
	printf("|| TrnIndLA START                                                           ||\n");
	printf("==============================================================================\n");

	file_list 	= new map<int,map<int,pair<int,string> > >;
	label_list	= new map<int,vector<string> >;
	message		= new vector<string>;
	KB			= new CKB;
	RF 			= new ReadFile;
	WF 			= new WriteFile;

	this->ReadFileExt(idx_, object_);

	Train *T;

	// Subject
	for(int f=0;f<sub_num;f++)
	{
		T = new Train;
		T->Init(LOC_INT,SEC_INT,FILTER_WIN);

		G = new CGraph;
		G->SetObject(object_);
		G->SetLocInt(LOC_INT);
		G->SetSecInt(SEC_INT);

		// to check if a LA has already been seen.
		vector<int> al_tmp_idx; al_tmp_idx.resize(KB->al.size());

		for(int ff=0;ff<sub_num;ff++)
		{
			if (f == ff) continue;

			// Data per subject
			for(int fff=0;fff<(*file_list)[ff].size();fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = EVAL + "/" + object_ + "/" + to_string(f);
				directoryCheck(dir_s);

				// read available location areas
				path = dir_s + "/location_area.txt";
				RF->ReadFileLA(G,KB->al,path);

				// action filename pair
				pair_tmp = (*file_list)[ff][fff];

				// for initial labelling
				if (1)
				{
					for(int i=0;i<KB->al.size();i++)
					{
						if (al_tmp_idx[i]>0) continue;
						for(int ii=0;ii<(*label_list)[pair_tmp.first].size();ii++)
						{
							if (!strcmp(KB->al[i].c_str(),(*label_list)[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				// learning process
				if (T->Learning(G, KB, pair_tmp.second, path, flag)==EXIT_FAILURE)
				{ printer(29); return EXIT_FAILURE;	} printer(30);

				// writing location areas data
				if(G->GetNumberOfNodes()>0)
				{
					remove(path.c_str());
					WF->WriteFileLA(G, KB, path.c_str());
				}
			}
		}

		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/graph.txt";
		WF->WriteFileGraph(G, path);
		path = 	EVAL + "/" + object_ + "/" + to_string(f) + "/window.txt";
		WF->WriteFileWindow(G,path);

		delete G;
	}

	delete file_list;
	delete label_list;
	delete message;
	delete KB;
	delete RF;
	delete WF;

	printf("==============================================================================\n");
	printf("|| TrnIndLA END                                                             ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Tst(vector<int> idx_, string object_)
{
	string dir_s, path;

	printf("==============================================================================\n");
	printf("|| TST START                                                                ||\n");
	printf("==============================================================================\n");

	file_list 	= new map<int,map<int,pair<int,string> > >;
	label_list	= new map<int,vector<string> >;
	message		= new vector<string>;
	KB			= new CKB;

	this->ReadFileExt(idx_, object_);

	Test *T;

	// Subject
	for(int f=0;f<sub_num;f++)
	{
		// directory of learned data of a subject
		dir_s = EVAL + "/" + object_ + "/" + to_string(f);

		T = new Test;
		T->Init(LOC_INT, SEC_INT, FILTER_WIN, object_);

		// set parse message
		T->SetMessage(*message);

		// set kb
		T->SetKB(KB);

		// read available location areas
		path  = dir_s + "/location_area.txt";
		if (T->ReadLA(path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}

		// read available sector map
		path = 	dir_s + "/graph.txt";
		if (T->ReadGraph(path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}

		// apply gauss filter
		T->ApplyGauss(5,5);

		// write window constraint
		path = 	dir_s + "/window.txt";
		if (T->WriteWindow(path)==EXIT_FAILURE)
		{return EXIT_FAILURE;}

		dir_s = RESULT + "/" + object_ + "/";
		directoryCheck(dir_s);
		dir_s = RESULT + "/" + object_ + "/" + to_string(f) + "/";
		directoryCheck(dir_s);

		// Data per subject
		for(int ff=0;ff<(*file_list)[f].size();ff++)
		{
			// action filename pair
			// pair_tmp = file_list[ff][fff];

			char num[4]; sprintf(num,"%03d",(*file_list)[f][ff].first);

//			if(strcmp(num,"006")) continue;

			path = dir_s + string(num) + "/";
			directoryCheck(path);

			if (T->Testing((*file_list)[f][ff].second, path)==EXIT_FAILURE)
			{ printer(43); return EXIT_FAILURE; }
			else
			{ printer(44); }
		}

		delete T;
	}

	delete file_list;
	delete label_list;
	delete message;
	delete KB;

	printf("==============================================================================\n");
	printf("|| TST END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Lbl(vector<int> idx_)
{

	int n, b, e; n=b=e=0;
	string path;

	map<string, string> label_ref_list;
	vector<Vector3d> labels_ref;
	vector<string> labels_ref_name;

	file_list 	= new map<int,map<int,pair<int,string> > >;  // subject, file number, action, filename
	label_list	= new map<int,vector<string> >;

	ReadFile RF;
	WriteFile WF;
	DataParser P;

	printf("==============================================================================\n");
	printf("|| LABELLING START                                                          ||\n");
	printf("==============================================================================\n");

	// Reading label_list
	path = KB_DIR + "/";
	if (RF.ReadFileLabel(path, label_list)==EXIT_FAILURE)
	{ return EXIT_FAILURE; }

	// Reading file names for location references of the labels
	path = "./label/";
	if (RF.ReadLabelFileName(path, label_ref_list)==EXIT_FAILURE)
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
		for(int ff=0;ff<(*file_list)[f].size();ff++)
		{
			if (RF.ReadFile_((*file_list)[f][ff].second, ',')==EXIT_FAILURE)
			{ return EXIT_FAILURE; } printer(8);

			P.SetDataParser(RF.GetDataRF());
			P.ParseDataNoLabel();
			WF.RewriteDataFile(
				(*file_list)[f][ff].second,
				RF.GetDataRF(),
				P.GetPointParser(),
				P.GetContactParser(),
				P.GetFaceParser(),
				labels_ref,
				labels_ref_name,
				(*label_list)[(*file_list)[f][ff].first]);
			printer(48);

			// Visualize
			if (0)
			{
				VTKExtra *VTK = new VTKExtra(LOC_INT,SEC_INT);
				vector<vector<unsigned char> > color_code; VTK->ColorCode(color_code);
				vector<string>  goal_action, al; goal_action.resize(5);
				vector<int> 	loc_idx_zero;
				VTK->ShowData(
						P.GetPointParser(), goal_action, al,
						loc_idx_zero, color_code, true, false, false);
				delete VTK;
			}
		}
	}

	printf("==============================================================================\n");
	printf("|| LABELLING END                                                            ||\n");
	printf("==============================================================================\n");

	delete file_list;
	delete label_list;

	return EXIT_SUCCESS;
}

int TestCase::ReadFileExt(vector<int> idx_, string object_)
{
	string path;

	// Reading data filenames
	ReadFile RF;
	path = DATA_DIR;
	if (RF.ReadFileName(path, idx_, file_list, sub_num)==EXIT_FAILURE)
	{ return EXIT_FAILURE; } printer(26);

	// Reading action sequence label
	path = KB_DIR + "/label.txt";
	if (RF.ReadFileLabel(path, label_list)==EXIT_FAILURE)
	{return EXIT_FAILURE;}

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	path = KB_DIR + "/";
	if (RF.ReadFileKB(path, KB)==EXIT_FAILURE)
	{return EXIT_FAILURE;}

	// read parse message
	path =  KB_DIR + "/message.txt";
	if (RF.ReadMsg(path, message)==EXIT_FAILURE)
	{return EXIT_FAILURE;}

	// Check if data was trained.
	directoryCheck(EVAL + "/" + object_);

	return EXIT_SUCCESS;
}

