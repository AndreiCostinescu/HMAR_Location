/*
 * Test.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef TEST_H_
#define TEST_H_

#include "VTKExtra.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "DataParser.h"
#include "DataFilter.h"
#include "ActionParser.h"
#include "ActionPrediction.h"

class Test : public DataParser
{
public:
	Test();
	virtual ~Test();

	int Init(int loc_int_, int sec_int_, int f_win_, string obj_);
	int ReadKB(string path_);
	int ReadLA(string path_);
	int ReadGraph(string path_);
	int SetMessage(vector<string> msg_);
	int SetKB(CKB *kb_);

	int WriteWindow(string path_);

	int ApplyGauss(int num_x_, int num_y_);

	int Testing(
			string filename_,
			string resultdir_);

private:
	DataFilter *DF;
	ActionPrediction *APred;
	ActionParser *AParse;
	ReadFile *RF;
	WriteFile *WF;
	VTKExtra *VTK;

	int loc_int;
	int sec_int;
	int f_win;

};

#endif /* TEST_H_ */
