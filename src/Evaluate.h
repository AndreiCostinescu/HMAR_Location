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
		Evaluate(Graph *Graph_);
		virtual ~Evaluate();

		void setVelocity(double x_) 		{vel_eval = x_;}
		void setLabel(int x_) 				{label1_eval = x_;}
		void setWindow(vector<double> x_) 	{win_eval = x_;}
		void setPctErr(vector<double> x_)	{pct_err_eval = x_;}
		void setGraph(Graph *x_) 			{G = x_;}
		state_t getState() 					{return state_eval;}

		void UpdateEval();

	protected:

		int label1_eval;
		double vel_eval;

		vector<double> win_eval;
		vector<double> pct_err_eval;

		state_t state_eval;
		Graph *G;
};

#endif /* EVALUATE_H_ */
