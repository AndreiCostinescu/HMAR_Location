/*
 * Evaluate.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 */

#include "Evaluate.h"

Evaluate::Evaluate(
	Graph *Graph_)
{
	label1_eval = 0;
	vel_eval 	= 0.0;

	win_eval.clear();
	pct_err_eval.clear();

	state_eval = {};
	G = Graph_;
}

Evaluate::~Evaluate() {}

int Evaluate::UpdateEval()
{
	state_eval = G->getState();

	double max_tmp = *max_element(pct_err_eval.begin(), pct_err_eval.end());

	node_tt node_tmp = {};

	if (max_tmp<=0)
	{
		state_eval.pct_err = 0;
	}
	else
	{
		state_eval.pct_err = max_tmp;
		int idx =
					distance(
							pct_err_eval.begin(),
							max_element(pct_err_eval.begin(), pct_err_eval.end()));

		vector<string> al_tmp = G->getActionLabel();
		map<string,pair<int,int> > ac_tmp = G->getActionCategory();

		node_tmp = {};
		G->getNode(label1_eval, node_tmp);
		string tmp1 = node_tmp.name;
		G->getNode(idx, node_tmp);
		string tmp2 = node_tmp.name;

		int c= 0;
		for(int i=0;i<al_tmp.size();i++)
		{
			if (!strcmp(tmp1.c_str(),al_tmp[i].c_str()))
			{
				state_eval.label1 = i;
				c++;
			}
			else if (!strcmp(tmp2.c_str(),al_tmp[i].c_str()))
			{
				state_eval.label2 = i;
				c++;
			}
			if (c==2) break;
		}

		state_eval.con = node_tmp.contact;
		state_eval.sur = node_tmp.surface;
	}

	state_eval.mov = vel_eval;

	for(int i=0;i<G->getNumberOfNodes();i++)
	{
		G->getNode(i, node_tmp);
		state_eval.goal[node_tmp.name] = pct_err_eval[i];
		state_eval.window[node_tmp.name] = win_eval[i];
	}

	G->setState(state_eval);

	return EXIT_SUCCESS;
}
