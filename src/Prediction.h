/*
 * Prediction.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#ifndef PREDICTION_H_
#define PREDICTION_H_

#include "dataDeclaration.h"

class Prediction {
public:
	Prediction(
		int delay_,
		map<string,pair<int,int> > ac_,
		vector<string> al_);
	virtual ~Prediction();

	int checkContact(int x_);
	void predict(
		int label_,
		string name_,
		vector<map<string, double> > prediction_);
	void delay();

	void delay_(state_t s_);

	void parse(state_t s_);
	void display();


private:
	map<string,pair<int,int> > 		ac;
	vector<string> 					al;

	int					delay_factor;
	int					decide;
	pair<string,int> 	mem_delay;
	vector<double> 		probability;



	vector<state_t>		state_mem;
	string 				output;
	string 				output_mem;

};

#endif /* PREDICTION_H_ */
