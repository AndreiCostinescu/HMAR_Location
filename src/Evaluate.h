/*
 * Evaluate.h
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 */

#ifndef EVALUATE_H_
#define EVALUATE_H_

#include "dataDeclaration.h"
#include "Graph.h"

class Evaluate
{
	public:
		Evaluate();
		virtual ~Evaluate();

		int UpdateStateNode(Graph *G_);
		int UpdateStateEdge(Graph *G_);

	protected:

		int label1_eval;
		double vel_eval;

		double surface_dist_eval;

		vector<double> win_eval;
		vector<double> pct_err_eval;

		vector<string> al_eval;
		map<string,pair<int,int> > ac_eval;

		state_t state_eval;
};

#endif /* EVALUATE_H_ */
