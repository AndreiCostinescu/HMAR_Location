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
	string DATA_DIR_SINGLE 	= "../../KINECT/recording/";
	string DATA_DIR_ALL		= "../../KINECT/recording/";
	string TEST_DIR_ 		= "../../KINECT/recording/";
#else
	string DATA_DIR_SINGLE_ = "../KINECT/recording/";
	string DATA_DIR_ALL_	= "../KINECT/recording/";
	string TEST_DIR_ 		= "../KINECT/recording/";
#endif

//=============================================================================
// Global
//=============================================================================

stack<clock_t> tictoc_stack;
void tic() {tictoc_stack.push(clock());}
void toc()
{
	cout << "Time elapsed: "
       << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
       << std::endl;
	tictoc_stack.pop();
}

//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{
	printf("==============================================================================\n");
	printf("|| SYSTEM START                                                             ||\n");
	printf("==============================================================================\n");


	string object, scene, DATA_DIR_ALL, DATA_DIR_SINGLE, TEST_DIR;

	scene   		= "Kitchen";

//	object  		= "ALL";
//	DATA_DIR_ALL 	= DATA_DIR_ALL_ 	+ object	+ string("/");
//	result 			= learnLocationArea(DATA_DIR_ALL, scene, object);
//
//	if (result==EXIT_SUCCESS)
//		cout << "SUCCESS"<< endl;
//	else if (result==EXIT_FAILURE)
//		cerr << "FAILURE" << endl;
//	else
//		cout << "UNKNOWN" << endl;


	object  			= "04";
	DATA_DIR_SINGLE 	= DATA_DIR_SINGLE_	+ object	+ string("/");
	learning( DATA_DIR_SINGLE, scene, object);



//
//	object  			= "04";
//	TEST_DIR 			= TEST_DIR_ 		+ object 	+ string("/test/");
//	testing(TEST_DIR, scene, object);

//		vector<double> x,y1,y2,y0;
//		for(int i=0;i<num_points;i++)
//		{
//			x.push_back(i);
//			y0.push_back(l2Norm(pos_vel_acc_avg[i][0]));
//			y1.push_back(l2Norm(pos_vel_acc_avg[i][1]));
//			y2.push_back(l2Norm(pos_vel_acc_avg[i][2]));
//		}
//
//		plotData(x, y0);
//		plotData(x, y1);
//		plotData(x, y2);


	printf("==============================================================================\n");
	printf("|| SYSTEM END                                                               ||\n");
	printf("==============================================================================\n");

	return 0;
}




