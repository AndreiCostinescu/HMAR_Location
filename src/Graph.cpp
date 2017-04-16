/*
 * Graph.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#include "Graph.h"


Graph::Graph(
	string scene_,
	string target_)
{
	scene 	= scene_;
	target 	= target_;
}

void Graph::setKB(
		kb_t x_)
{
	surface 	= x_.surface;
	surface_eq	= x_.surface_eq;
	surface_lim	= x_.surface_lim;
	action_label= x_.al;
	action_cat	= x_.ac;
	object_label= x_.ol;
	for(int i=action_cat["GEOMETRIC"].first;i<action_cat["GEOMETRIC"].second+1;i++)
	{
		state.goal[action_label[i]] = 0.0;
	}
}

void Graph::initFilter()
{
	map<string,double> mask;
	for(int i=0;i<action_label.size();i++)
	{
		filter_init[action_label[i]] 		= 0.0;
		prediction_init[action_label[i]] 	= 0.0;
	}
}

void Graph::setInitFilter(
	vector<int> val_)
{
	if (val_.size()!=filter_init.size())
	{
		cerr << "[WARNING] : DIM(FILTER MASK) != DIM(ACTION LABEL)" << endl;
	}
	else
	{
		for(int i=0;i<val_.size();i++)
		{
			filter_init[action_label[i]] = val_[i];
		}
	}
}

void Graph::expandFilter(
	int x_)
{
	while (filter.size() < x_)
	{
		filter.push_back(filter_init);
		prediction_reset.push_back(prediction_init);
	}
}

void Graph::updateFilter(
	int x_)
{
	filter[x_][node_list[x_].name] = 1.0;
}

//=============================================================================
// NODES
//=============================================================================

vector<point_d> Graph::getCentroidList()
{
	vector<point_d> tmp;
	tmp.resize(node_list.size());
	for(int i=0;i<node_list.size();i++)
	{
		tmp[i] = node_list[i].centroid;
	}
	return tmp;
}

vector<int> Graph::getSurfaceFlagList()
{
	vector<int> tmp;
	tmp.resize(node_list.size());
	for(int i=0;i<node_list.size();i++)
	{
		tmp[i] = node_list[i].surface;
	}
	return tmp;
}

//=============================================================================
// EDGES
//=============================================================================

void Graph::addEmptyEdgeForNewNode(
	int idx_)
{
	if(edge_list.size()<idx_+1)
	{
		edge_list.resize(idx_+1);
		for(int i=0;i<edge_list.size();i++)
		{
			if(edge_list[i].size()<=getNumberOfNodes())
			{
				edge_list[i].resize(getNumberOfNodes());
				for(int ii=0;ii<edge_list[i].size();ii++)
				{
					if (edge_list[i][ii].size()==0)
						edge_list[i][ii].push_back({});
					edge_list[i][ii][0].loc_start.resize(LOC_INT);
					edge_list[i][ii][0].loc_mid.resize(LOC_INT);
					edge_list[i][ii][0].loc_end.resize(LOC_INT);
					edge_list[i][ii][0].tan.resize(LOC_INT);
					edge_list[i][ii][0].nor.resize(LOC_INT);
					edge_list[i][ii][0].sector_map.resize(LOC_INT*SEC_INT);
					edge_list[i][ii][0].sector_const.resize(LOC_INT*SEC_INT);
					edge_list[i][ii][0].mov_const.resize(2);
				}
			}
		}
	}
}

