/*
 * TestCase.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#ifndef TESTCASE_H_
#define TESTCASE_H_

//#include "algo.h"
#include "core.h"
#include "Test.h"
#include "print.h"
#include "Train.h"

#define FILTER_WIN 5
#define	LOC_INT 100
#define	SEC_INT 36

class TestCase
{
	public:
		TestCase();
		virtual ~TestCase();
		void Choose(int x_);
		int TrnInd(vector<int> idx_, string object_);
		int TrnIndLA(vector<int> idx_, string object_);
		int Tst(vector<int> idx_, string object_);
		int Lbl(vector<int> idx_);
		int ReadFileExt(vector<int> idx_, string object_);

	private:
		string EVAL, EVAL2, RESULT, RESULT2, KB_DIR, DATA_DIR;
		map<int,string> dict;

		int sub_num;

		map<int,map<int,pair<int,string> > > *file_list; // subject, file number, action, filename
		map<int,vector<string> > *label_list;
		vector<string> *message;
		CKB *KB;
		CGraph *G;
		ReadFile *RF;
		WriteFile *WF;

};

#endif /* TESTCASE_H_ */
