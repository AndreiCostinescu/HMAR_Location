/*
 * Filter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef FILTER_H_
#define FILTER_H_

#include "dataDeclaration.h"
#include "algo.h"

class Filter
{
	public:
		Filter();
		virtual ~Filter();

		void ResetFilter();

		int PreprocessDataLive(
			point_d pos_,
			vector<point_d> &pos_vel_acc_avg_, //motion
			unsigned int window_);

		int PreprocessContactLive(
			int &contact_,
			unsigned int window_);

	private:
		vector< vector< point_d > > pos_vel_acc_mem;
		vector<int> 				contact_mem;

};

#endif /* FILTER_H_ */
