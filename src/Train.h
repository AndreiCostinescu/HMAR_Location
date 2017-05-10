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

class Train : public Test, public TrainLA, public TrainSM
{
	public:
		Train();
		virtual ~Train();

		int Learning(
			string filename_,
			string path_LA_,
			bool flag_);
};

#endif /* TRAIN_H_ */
