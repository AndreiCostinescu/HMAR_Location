/*
 * Test.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef TEST_H_
#define TEST_H_

#include "vtkExtra.h"
#include "Prediction.h"
#include "DataParser.h"
#include "DataFilter.h"
#include "DataContainer.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "ActionPrediction.h"

class Test : public ReadFile, public WriteFile, public DataContainer, public DataFilter, public DataParser
{
	public:
		Test();
		virtual ~Test();

		int Testing(
			string filename_,
			string resultdir_,
			Graph *Graph_);

		int Deploying(
			string filename_,
			string resultdir_,
			Graph *Graph_);
};

#endif /* TEST_H_ */
