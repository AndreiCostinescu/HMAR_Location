/*
 * TestCase.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#ifndef TESTCASE_H_
#define TESTCASE_H_

#include "Train.h"

class TestCase : public Train
{

	public:
		TestCase();
		virtual ~TestCase();
		void Choose(int x_, int y_);
		int Trn(vector<int> idx_, string object_);
		int Tst(vector<int> idx_, string object_);
		int Dpl(vector<int> idx_, string object_);
		int Lbl(vector<int> idx_);

	private:
		string EVAL, RESULT, DATA_DIR;
		map<int,string> dict;

};

#endif /* TESTCASE_H_ */
