/*
 * Test.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef TEST_H_
#define TEST_H_

#include "Deployment.h"
#include "DataParser.h"

class Test : public Deployment, public DataParser
{
	public:
		Test();
		virtual ~Test();

		int Testing(
			string filename_,
			string resultdir_);

};

#endif /* TEST_H_ */
