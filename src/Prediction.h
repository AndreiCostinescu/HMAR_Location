/*
 * Prediction.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#ifndef PREDICTION_H_
#define PREDICTION_H_

#include "dataDeclaration.h"
#include "algo.h"

class Prediction {
public:
	Prediction(
		int delay_,
		map<string,pair<int,int> > ac_,
		vector<string> al_);
	virtual ~Prediction();

	void Delay_(state_t s_);
	void Parse(state_t s_);
	void DT0();
	void DT1();
	void DT2_1();
	void DT2_2();
	void DT3();
	void DT4_1(double x_);
	void DT4_2(vector<double> x_);
	void Display();

private:
	map<string,pair<int,int> > 		ac;
	vector<string> 					al;

	int					delay_factor;
	vector<state_t>		state_mem;
	string 				output;
	string 				output_mem;
	string 				label;
	string 				label_mem;
	string 				repeat;
	bool				grasp_flag;
	bool				start_loc;
};

#endif /* PREDICTION_H_ */
