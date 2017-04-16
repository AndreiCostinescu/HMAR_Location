/*
 * TestCase.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#ifndef TESTCASE_H_
#define TESTCASE_H_

#include "dataDeclaration.h"
#include "util.h"
#include "ReadFile.h"

class TestCase {
public:
	TestCase();
	virtual ~TestCase();
	void choose(int x_);
	int TC1(vector<int> idx_);
	int TC2(vector<int> idx_);
};

#endif /* TESTCASE_H_ */
