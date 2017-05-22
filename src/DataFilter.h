/*
 * DataFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 *      Detail: Moving average filter is applied on the data.
 */

#ifndef DATAFILTER_H_
#define DATAFILTER_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "algo.h"

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class DataFilter
{
public:
	DataFilter();
	virtual ~DataFilter();

	void ResetFilter();
	int PreprocessDataLive(
			Vector4d pos_,
			vector<Vector4d> &pos_vel_acc_avg_, //motion
			unsigned int window_);
	int PreprocessContactLive(
			int contact_,
			int &contact_out_,
			unsigned int window_);

private:
	vector<vector<Vector4d> > 	pos_vel_acc_mem;
	vector<int> 				contact_mem;
};

#endif /* DATAFILTER_H_ */
