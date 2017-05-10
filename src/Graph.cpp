/*
 * Graph.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#include "Graph.h"


Graph::Graph(
	string object_)
{
	object 	= object_;
}

//void Graph::SetKB(
//		kb_t x_)
//{
//	surface_mid	= x_.surface_mid;
//	surface_min	= x_.surface_min;
//	surface_max	= x_.surface_max;
//	surface_eq	= x_.surface_eq;
//	surface_lim	= x_.surface_lim;
//	surface_rot	= x_.surface_rot;
//	action_label= x_.al;
//	action_cat	= x_.ac;
//	object_label= x_.ol;
//	for(int i=action_cat["GEOMETRIC"].first;i<action_cat["GEOMETRIC"].second+1;i++)
//	{
//		action_state.goal[action_label[i]] = 0.0;
//	}
//}

//kb_t Graph::GetKB()
//{
//	kb_t kb_tmp = {};
//	kb_tmp.surface_mid = surface_mid;
//	kb_tmp.surface_min = surface_min;
//	kb_tmp.surface_max = surface_max;
//	kb_tmp.surface_eq = surface_eq;
//	kb_tmp.al = action_label;
//	kb_tmp.ac = action_cat;
//	kb_tmp.ol = object_label;
//	return kb_tmp;
//}

//=============================================================================
// NODES
//=============================================================================

vector<Vector4d> Graph::GetCentroidList()
{
	vector<Vector4d> tmp;
	tmp.resize(node_list.size());
	for(int i=0;i<node_list.size();i++)
	{
		tmp[i] = node_list[i].centroid;
	}
	return tmp;
}

vector<int> Graph::GetSurfaceFlagList()
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
			if(edge_list[i].size()<=GetNumberOfNodes())
			{
				edge_list[i].resize(GetNumberOfNodes());
				for(int ii=0;ii<edge_list[i].size();ii++)
				{
					if (edge_list[i][ii].size()==0)
						edge_list[i][ii].push_back({});
					edge_list[i][ii][0].loc_mid.resize(LOC_INT);
					edge_list[i][ii][0].loc_len.resize(LOC_INT);
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

