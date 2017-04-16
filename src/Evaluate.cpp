/*
 * Evaluate.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 */

#include "Evaluate.h"

Evaluate::Evaluate()
{
	G = new Graph("A","A");
	label1 = 0;
	vel = 0;
}

Evaluate::~Evaluate() {}

void Evaluate::update()
{
	state = G->getState();

	double max_tmp = *max_element(pct_err.begin(), pct_err.end());
	if (max_tmp<=0)	{state.pct_err = -1;}
	else			{state.pct_err = max_tmp;}

	int idx =
			distance(
					pct_err.begin(),
					max_element(pct_err.begin(), pct_err.end()));

	vector<string> al_tmp = G->getActionLabel();
	map<string,pair<int,int> > ac_tmp = G->getActionCategory();

	node_tt node_tmp = {};
	G->getNode(label1, node_tmp);
	string tmp1 = node_tmp.name;
	G->getNode(idx, node_tmp);
	string tmp2 = node_tmp.name;

	int c= 0;
	for(int i=0;i<al_tmp.size();i++)
	{
		if (!strcmp(tmp1.c_str(),al_tmp[i].c_str()))
		{
			state.label1 = i;
			c++;
		}
		else if (!strcmp(tmp2.c_str(),al_tmp[i].c_str()))
		{
			state.label2 = i;
			c++;
		}
		if (c==2) break;
	}

	state.con = node_tmp.contact;
	state.sur = node_tmp.surface;

	state.mov = vel;

	for(int i=0;i<G->getNumberOfNodes();i++)
	{
		G->getNode(i, node_tmp);
		state.goal[node_tmp.name] = pct_err[i];
	}
}
