/*
 * Train.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef TRAIN_H_
#define TRAIN_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "CData.h"
#include "TrainLA.h"
#include "TrainSM.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "DataParser.h"
#include "DataFilter.h"

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Train :  public DataParser, public TrainLA, public TrainSM
{
public:
	Train();
	virtual ~Train();

	int Init(
			int loc_int_,
			int sec_int_,
			int f_win_);

	int Learning(
		CGraph *G_,
		CKB *KB_,
		string filename_,
		string path_LA_,
		bool flag_);

private:
	VTKExtra *VTK;
	ReadFile *RF;
	WriteFile *WF;
	DataFilter *DF;

	int loc_int;
	int sec_int;
	int f_win;

	vector<vector<Vector4d> > *pvas; // length->motion
	vector<int> *contacts;
};

#endif /* TRAIN_H_ */
