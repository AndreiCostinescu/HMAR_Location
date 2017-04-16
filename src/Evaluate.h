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

		void setVelocity(double x_) 		{vel = x_;}
		void setLabel(int x_) 				{label1 = x_;}
		void setWindow(vector<double> x_) 	{win = x_;}
		void setPctErr(vector<double> x_)	{pct_err = x_;}
		void setGraph(Graph *x_) 			{G = x_;}
		state_t getState() 					{return state;}

		void update();

	private:
		int label1;

		double vel;

		vector<double> win;
		vector<double> pct_err;

		Graph *G;

		state_t state;
};

#endif /* EVALUATE_H_ */
