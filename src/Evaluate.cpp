/*******************************************************************************
 * Evaluate.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 *      Detail: Evaluate and update the action state.
 ******************************************************************************/

#include "Evaluate.h"

Evaluate::Evaluate() : 	label1_eval(-1),
						vel_eval(0.0),
						surface_dist_eval(0.0)
{
}

Evaluate::~Evaluate() {}

int Evaluate::UpdateStateNode(CGraph *G_, CAS *AS_)
{
	AS_->vel = vel_eval;
	AS_->surface_dist = surface_dist_eval;
	return EXIT_SUCCESS;
}

int Evaluate::UpdateStateEdge(CGraph *G_, CAS *AS_)
{
	double max_val = *max_element(pct_err_eval.begin(), pct_err_eval.end());

	if (max_val<=0)
	{
		AS_->pct_err		= 0.0;
		AS_->surface_flag	= 0;
		AS_->surface_name	= "";
	}
	else
	{
		AS_->pct_err = max_val;
		int idx =
					distance(
							pct_err_eval.begin(),
							max_element(
									pct_err_eval.begin(),
									pct_err_eval.end()));

		// Saving the LA as integers instead of string.
		int c= 0;
		for(int i=0;i<al_eval.size();i++)
		{
			if (!strcmp(
					G_->GetNode(label1_eval).name.c_str(),
					al_eval[i].c_str()))
			{
				AS_->label1 = i;
				c++;
			}
			if (!strcmp(
					G_->GetNode(idx).name.c_str(),
					al_eval[i].c_str()))
			{
				AS_->label2 = i;
				c++;
			}
			if (c==2) break;
		}

		AS_->surface_flag = G_->GetNode(idx).surface_flag;
		AS_->surface_name = G_->GetNode(idx).name;
	}

	AS_->vel = vel_eval;

	for(int i=0;i<G_->GetNumberOfNodes();i++)
	{
		AS_->goal  [G_->GetNode(i).name] = pct_err_eval[i];
		AS_->window[G_->GetNode(i).name] = win_eval[i];
	}

	return EXIT_SUCCESS;
}
