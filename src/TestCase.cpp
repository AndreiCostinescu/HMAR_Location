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

void TestCase::choose(int x_)
{
	switch(x_)
	{
		case 1:
		{
			vector<int> idxs;
			for(int idx=5;idx<11;idx++)
			{
				if(idx==5) continue;
				if(idx==7) continue;
				if(idx==9) continue;
				idxs.push_back(idx);
			}
			this->TC1(idxs);
			break;
		}
		case 2:
		{
			vector<int> idxs;
			for(int idx=5;idx<11;idx++)
			{
				if(idx==5) continue;
				if(idx==7) continue;
				if(idx==9) continue;
				idxs.push_back(idx);
			}
			this->TC2(idxs);
			break;
		}

		default:
			break;
	}
}

//=============================================================================
// TEST CASES
//=============================================================================

int TestCase::TC1(vector<int> idx_)
{

#ifdef PC
	string EVAL		= "../Scene1";
	string DATA_DIR = "../../KINECT/recording/";
#else
	string EVAL 	= "Scene1";
	string DATA_DIR	= "../KINECT/recording/";
#endif

	string tar = "Slide";

	bool flag;
	int f, ff, fff, res, n, i, ii;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label;
	pair<int,string> pair_tmp(-1,"");

	kb_t kb;
	ReadFile RF;
	Graph *G;

	printf("==============================================================================\n");
	printf("|| TC1 START                                                                ||\n");
	printf("==============================================================================\n");

	// TODO manualy label?
	// Reading label
	res = readFileLabel(DATA_DIR, label);
	if (res==EXIT_FAILURE)	{ return EXIT_FAILURE;	}

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	res = readFileKB(DATA_DIR, kb);
	if (res==EXIT_FAILURE)	{ return EXIT_FAILURE;	}

	// Reading data files
	RF.readFileName(DATA_DIR, idx_);
	file_list = RF.getFileList();
	n = RF.getNumberOfSubject();

	directoryCheck(EVAL + "/" + tar);

	printf("==============================================================================\n");
	printf("|| TRAINNING                                                                ||\n");
	printf("==============================================================================\n");

	// Subject
	for(f=0;f<n;f++)
	{
		for(ff=0;ff<n;ff++)
		{
			if(f == ff) continue;

			G = new Graph(EVAL, tar);
			G->setKB(kb);

			vector<string> al_tmp; al_tmp = kb.al;
			vector<int> al_tmp_idx; al_tmp_idx.resize(al_tmp.size());

			// Data per subject
			for(fff=0;fff<file_list[ff].size();fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = EVAL + "/" + tar + "/" + to_string(f);
				directoryCheck(dir_s);

				// read available location areas
				path = dir_s + "/location_area.txt";
				readFileExt(G, path.c_str(), 0);

				// action filename pair
				pair_tmp = file_list[ff][fff];

				// for initial labelling
				if (1)
				{
					for(i=0;i<al_tmp.size();i++)
					{
						if (al_tmp_idx[i]>0) continue;
						for(ii=0;ii<label[pair_tmp.first].size();ii++)
						{
							if (!strcmp(al_tmp[i].c_str(),label[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				res = learning(pair_tmp.second, G, flag);

				if (res==EXIT_FAILURE)	{ printer(29); return EXIT_FAILURE;	}
				else					{ printer(30);						}

				if(G->getNumberOfNodes()>0)
				{
					remove(path.c_str());
					writeFile(G, path.c_str(), 0);
				}
			}

			path = 	EVAL + "/" + tar + "/" + to_string(f) + "/graph.txt";
			writeFile(G, path.c_str(), 1);

			delete G;
		}
	}

	printf("==============================================================================\n");
	printf("|| TC1 END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::TC2(vector<int> idx_)
{

#ifdef PC
	string EVAL		= "../Scene1";
	string DATA_DIR = "../../KINECT/recording/";
#else
	string EVAL 	= "Scene1";
	string RESULT 	= "Result";
	string DATA_DIR	= "../KINECT/recording/";
#endif

	string tar = "Slide";

	int f, ff, fff, res, n;
	string dir_s, path;
	map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
	map<int,vector<string> > label;

	kb_t kb;
	ReadFile RF;
	Graph *G;

	printf("==============================================================================\n");
	printf("|| TC2 START                                                                ||\n");
	printf("==============================================================================\n");

	// Reading label
	res = readFileLabel(DATA_DIR, label);
	if (res==EXIT_FAILURE)	{ return EXIT_FAILURE;	}

	// Reading surface
	// Reading action labels
	// - reads the labels and initializes a zero list prediction/filter with the same length as the label
	// Reading object specific labels
	// - reads the object specific labels and saves them
	res = readFileKB(DATA_DIR, kb);
	if (res==EXIT_FAILURE)	{ return EXIT_FAILURE;	}

	// Reading data files
	RF.readFileName(DATA_DIR, idx_);
	file_list = RF.getFileList();
	n = RF.getNumberOfSubject();

	directoryCheck(EVAL + "/" + tar);

	printf("==============================================================================\n");
	printf("|| TESTING                                                                  ||\n");
	printf("==============================================================================\n");

	// Subject
	for(f=0;f<n;f++)
	{
		for(ff=0;ff<n;ff++)
		{
			if(f != ff) continue;

			G = new Graph(EVAL, tar);
			G->setKB(kb);

			// read available location areas
			dir_s = EVAL + "/" + tar + "/" + to_string(f);
			path  = dir_s + "/location_area.txt";
			if (readFileExt(G, path.c_str(), 0)==EXIT_FAILURE) { return EXIT_FAILURE; }

			path = 	dir_s + "/graph.txt";
			readFileExt(G, path.c_str(), 4);

			dir_s = RESULT + "/" + tar + "/";
			directoryCheck(dir_s);

			dir_s = RESULT + "/" + tar + "/" + to_string(f) + "/";
			directoryCheck(dir_s);

			// Data per subject
			for(fff=1;fff<file_list[ff].size();fff++)
			{
				// action filename pair
				// pair_tmp = file_list[ff][fff];

				res = testing(file_list[ff][fff].second, dir_s, fff, G);

				if (res==EXIT_FAILURE)	{ printer(43); return EXIT_FAILURE;	}
				else					{ printer(44);						}
			}

			delete G;

		}
	}

	printf("==============================================================================\n");
	printf("|| TC2 END                                                                  ||\n");
	printf("==============================================================================\n");

	return EXIT_SUCCESS;
}
