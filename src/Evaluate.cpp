/*
 * Evaluate.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 */

#include "Evaluate.h"

Evaluate::Evaluate() : 	label1_eval(0),
						vel_eval(0.0),
						surface_dist_eval(0.0),
						state_eval{}
{
}

Evaluate::~Evaluate() {}

int Evaluate::UpdateStateNode(Graph *G_)
{
	state_eval = G_->GetActionState();
	state_eval.mov = vel_eval;
	state_eval.sur_dist = surface_dist_eval;
	G_->SetActionState(state_eval);
	return EXIT_SUCCESS;
}

int Evaluate::UpdateStateEdge(Graph *G_)
{
	state_eval = G_->GetActionState();

	double max_val = *max_element(pct_err_eval.begin(), pct_err_eval.end());

	if (max_val<=0)
	{
		state_eval.pct_err 	= 0.0;
		state_eval.sur 		= 0;
	}
	else
	{
		state_eval.pct_err = max_val;
		int idx =
					distance(
							pct_err_eval.begin(),
							max_element(
									pct_err_eval.begin(),
									pct_err_eval.end()));

		int c= 0;
		for(int i=0;i<al_eval.size();i++)
		{
			if (!strcmp(
					G_->GetNode(label1_eval).name.c_str(),
					al_eval[i].c_str()))
			{
				state_eval.label1 = i;
				c++;
			}
			if (!strcmp(
					G_->GetNode(idx).name.c_str(),
					al_eval[i].c_str()))
			{
				state_eval.label2 = i;
				c++;
			}
			if (c==2) break;
		}

		state_eval.sur = G_->GetNode(idx).surface;
	}

	state_eval.mov = vel_eval;

	for(int i=0;i<G_->GetNumberOfNodes();i++)
	{
		state_eval.goal  [G_->GetNode(i).name] = pct_err_eval[i];
		state_eval.window[G_->GetNode(i).name] = win_eval[i];
	}

	G_->SetActionState(state_eval);

	return EXIT_SUCCESS;
}
