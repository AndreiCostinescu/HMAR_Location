/*
 * ActionParser.h
 *
 *  Created on: May 21, 2017
 *      Author: chen
 *      Detail: Parse the action state using a decision tree.
 */

#ifndef ACTIONPARSER_H_
#define ACTIONPARSER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "CGraph.h"
#include "CKB.h"
#include "CAS.h"

using namespace std;

class ActionParser
{
public:
	ActionParser();
	virtual ~ActionParser();

	int ReadMsg(string path_);
	void SetMsg(vector<string> msg_) { message = msg_; }

	void Init(
			string obj_,
			map<string,pair<int,int> > ac_,
			vector<string> al_,
			map<string,map<string,string> > ol_,
			int delay_);
	string Decode(string obj_, string loc_, string msg_);

	void Delay_(CAS *s_);
	void Parse(CAS *s_);
	void DT0();
	void DT1();
	void DT2_1();
	void DT2_2();
	void DT3_1();
	void DT3_2();
	void DT4_1(double x_);
	void DT4_2(vector<double> x_);
	void Display(string filename_);

private:
	map<string,pair<int,int> > 		ac;
	vector<string> 					al;
	map<string,map<string,string> > ol;

	vector<string> 	message;
	vector<int> 	message_num;

	string 		obj;
	string 		loc;
	int			delay_factor;
	vector<CAS>	state_mem;
	string 		output;
	string 		output_mem;
	string 		label;
	string 		label_mem;
	string 		repeat;
	bool		grasp_flag;
	bool		start_loc;

	string o;
	string l;
	string a;

	string ob_ac;

	double addFunction (double x, double y) { return fabs(x)+fabs(y); }

};

#endif /* ACTIONPARSER_H_ */
