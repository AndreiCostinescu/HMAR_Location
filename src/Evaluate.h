/*******************************************************************************
 * Evaluate.h
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 *      Detail: Evaluate and update the action state.
 ******************************************************************************/

#ifndef EVALUATE_H_
#define EVALUATE_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "CGraph.h"
#include "CAS.h"

using namespace std;

class Evaluate
{
	public:
		Evaluate();
		virtual ~Evaluate();

		int UpdateStateNode(CGraph *G_, CAS *AS_);
		int UpdateStateEdge(CGraph *G_, CAS *AS_);

	protected:
		int label1_eval;
		double vel_eval;
		double surface_dist_eval;
		vector<double> win_eval;
		vector<double> pct_err_eval;
		vector<string> al_eval;
		map<string,pair<int,int> > ac_eval;
};

#endif /* EVALUATE_H_ */
