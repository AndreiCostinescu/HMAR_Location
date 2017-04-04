//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#include "dataDeclaration.h"
#include "util.h"

#ifdef PC
	string DATA_DIR_ 		= "../../KINECT/recording/";
	string SCENE__ 			= "../Scene/";
#else
	string DATA_DIR_		= "../KINECT/recording/";
	string SCENE__ 			= "Scene/";
#endif

//=============================================================================
// Global
//=============================================================================

//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{
	int n, nn, l, f, ff;

	struct dirent **name_list0, **name_list1;

	string dir_sub_name, dir_act_name, path;

	string scn = "Kitchen";
	string obj = "001";


	Graph *G, Graph_main(scn, obj); G = &Graph_main;

//	// [ACTION LABELS]*********************************************************
//	path =  SCENE__ + "/action_label.txt";
//	if (readFileExt(G, path.c_str(), 2)==EXIT_FAILURE) {return EXIT_FAILURE;}
//	// *********************************************************[ACTION LABELS]
//	// [OBJ MASK]**************************************************************
//	path =  SCENE__ + scn + "/" + obj + "/obj_action_label.txt";
//	if (readFileExt(G, path.c_str(), 1)==EXIT_FAILURE) {return EXIT_FAILURE;}
//	// **************************************************************[OBJ MASK]
//	// [SAVED LOCATION]********************************************************
//	path =  SCENE__ + scn + "/" + obj + "/location_area.txt";
//	if (readFileExt(G, path.c_str(), 0)==EXIT_FAILURE) {return EXIT_FAILURE;}
//	// ********************************************************[SAVED LOCATION]
//
//	path = 	SCENE__ + scn + "/" + obj + "/loc_data_begin.txt";
//	readFileExt(G, path.c_str(), 10);
//	path = 	SCENE__ + scn + "/" + obj + "/loc_data_mid.txt";
//	readFileExt(G, path.c_str(), 11);
//	path = 	SCENE__ + scn + "/" + obj + "/loc_data_end.txt";
//	readFileExt(G, path.c_str(), 12);
//	path = 	SCENE__ + scn + "/" + obj + "/loc_data_tangent.txt";
//	readFileExt(G, path.c_str(), 13);
//	path = 	SCENE__ + scn + "/" + obj + "/loc_data_normal.txt";
//	readFileExt(G, path.c_str(), 14);
//	path = 	SCENE__ + scn + "/" + obj + "/counter.txt";
//	readFileExt(G, path.c_str(), 15);
//	path = 	SCENE__ + scn + "/" + obj + "/sec_data_max.txt";
//	readFileExt(G, path.c_str(), 16);
//	path = 	SCENE__ + scn + "/" + obj + "/sec_data_const.txt";
//	readFileExt(G, path.c_str(), 17);

	printf("==============================================================================\n");
	printf("|| SYSTEM START                                                             ||\n");
	printf("==============================================================================\n");

	n = scandir(DATA_DIR_.c_str(), &name_list0, folderSelect, alphasort);
	if (n==0)	{printer(25); return 0;	}
	else		{printer(26);			}
	for(f=0;f<3;f++)
	{
		dir_sub_name = DATA_DIR_ + name_list0[f]->d_name;
		nn =
				scandir(dir_sub_name.c_str(), &name_list1, folderSelect2,
						alphasort);
		if (nn==0)	{printer(27); return 0;	}
		else		{printer(28);			}
		for(ff=0;ff<nn;ff++)
		{
			if (strcmp(name_list1[ff]->d_name, obj.c_str())) continue;
			dir_act_name = dir_sub_name + "/" + name_list1[ff]->d_name;
			l = learning(dir_act_name, G);
			if (l==EXIT_FAILURE)	{printer(29); return 0; }
			else					{printer(30);			}
		}
		directoryCheck(SCENE__ + scn + "/" + obj);
		path =  SCENE__ + scn + "/" + obj + "/location_area.txt";
		writeFile(Graph_main, path.c_str(), 0);
	}

	path = 	SCENE__ + scn + "/" + obj + "/loc_data_begin.txt";
	writeFile(Graph_main, path.c_str(), 10);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_mid.txt";
	writeFile(Graph_main, path.c_str(), 11);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_end.txt";
	writeFile(Graph_main, path.c_str(), 12);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_tangent.txt";
	writeFile(Graph_main, path.c_str(), 13);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_normal.txt";
	writeFile(Graph_main, path.c_str(), 14);
	path = 	SCENE__ + scn + "/" + obj + "/counter.txt";
	writeFile(Graph_main, path.c_str(), 15);
	path = 	SCENE__ + scn + "/" + obj + "/sec_data_max.txt";
	writeFile(Graph_main, path.c_str(), 16);
	path = 	SCENE__ + scn + "/" + obj + "/sec_data_const.txt";
	writeFile(Graph_main, path.c_str(), 17);

	// [ACTION LABELS]*********************************************************
	path =  SCENE__ + "/action_label.txt";
	if (readFileExt(G, path.c_str(), 2)==EXIT_FAILURE) {return EXIT_FAILURE;}
	// *********************************************************[ACTION LABELS]
	// [OBJ MASK]**************************************************************
	path =  SCENE__ + scn + "/" + obj + "/obj_action_label.txt";
	if (readFileExt(G, path.c_str(), 1)==EXIT_FAILURE) {return EXIT_FAILURE;}
	// **************************************************************[OBJ MASK]
	// [SAVED LOCATION]********************************************************
	path =  SCENE__ + scn + "/" + obj + "/location_area.txt";
	if (readFileExt(G, path.c_str(), 0)==EXIT_FAILURE) {return EXIT_FAILURE;}
	// ********************************************************[SAVED LOCATION]
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_begin.txt";
	readFileExt(G, path.c_str(), 10);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_mid.txt";
	readFileExt(G, path.c_str(), 11);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_end.txt";
	readFileExt(G, path.c_str(), 12);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_tangent.txt";
	readFileExt(G, path.c_str(), 13);
	path = 	SCENE__ + scn + "/" + obj + "/loc_data_normal.txt";
	readFileExt(G, path.c_str(), 14);
	path = 	SCENE__ + scn + "/" + obj + "/counter.txt";
	readFileExt(G, path.c_str(), 15);
	path = 	SCENE__ + scn + "/" + obj + "/sec_data_max.txt";
	readFileExt(G, path.c_str(), 16);
	path = 	SCENE__ + scn + "/" + obj + "/sec_data_const.txt";
	readFileExt(G, path.c_str(), 17);

	// Visualize
	if(1)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(G, point_zero, label_zero, color_code, true);
	}

	for(f=0;f<1;f++)
	{
		dir_sub_name = DATA_DIR_ + name_list0[f]->d_name;
		nn =
				scandir(dir_sub_name.c_str(), &name_list1, folderSelect2,
						alphasort);
		if (nn==0)	{printer(27); return 0;	}
		else		{printer(28);			}
		for(ff=0;ff<nn;ff++)
		{
			if (strcmp(name_list1[ff]->d_name, obj.c_str())) continue;
			dir_act_name = dir_sub_name + "/" + name_list1[ff]->d_name;
			l = testing(dir_act_name, G);
			if (l==EXIT_FAILURE)	{printer(29); return 0; }
			else					{printer(30);			}
		}
	}

	printf("==============================================================================\n");
	printf("|| SYSTEM END                                                               ||\n");
	printf("==============================================================================\n");

	return 0;
}




