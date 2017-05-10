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
