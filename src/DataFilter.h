/*
 * DataFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef DATAFILTER_H_
#define DATAFILTER_H_

#include "dataDeclaration.h"
#include "algo.h"

class DataFilter
{
	public:
		DataFilter();
		virtual ~DataFilter();

		void ResetFilter();

		int PreprocessDataLive(
			point_d pos_,
			vector<point_d> &pos_vel_acc_avg_, //motion
			unsigned int window_);

		int PreprocessContactLive(
			int &contact_,
			unsigned int window_);

	private:
		vector<vector<point_d> > 	pos_vel_acc_mem;
		vector<int> 				contact_mem;

};

#endif /* DATAFILTER_H_ */
