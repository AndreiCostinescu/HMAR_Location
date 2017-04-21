/*
 * Train.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef TRAIN_H_
#define TRAIN_H_

#include "Test.h"
#include "TrainLA.h"
#include "TrainSM.h"
#include "labeling_loc.h"

class Train : public Test, public TrainLA, public TrainSM
{
	public:
		Train();
		virtual ~Train();

		int Learning(
			string filename_,
			Graph *Graph_,
			bool flag_);
};

#endif /* TRAIN_H_ */
