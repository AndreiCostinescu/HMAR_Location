/*******************************************************************************
 * CGraph.cpp
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: A graph structure with edges and nodes.
 *     			Node list contains the nodes.
 *     			Edge list contains the edges in the form of a list.
 *     			([node1][node2][number of edges])
 ******************************************************************************/

#include "CGraph.h"

CGraph::CGraph( ) :	OBJECT(""),
					LOC_INT(-1),
					SEC_INT(-1)
{
}
CGraph::~CGraph() { }

/*******************************************************************************
 * Nodes
 ******************************************************************************/

vector<Vector4d> CGraph::GetCentroidList()
{
	vector<Vector4d> tmp;
	for(int i=0;i<node_list.size();i++)
	{
		tmp.push_back(node_list[i].centroid);
	}
	return tmp;
}

vector<int> CGraph::GetSurfaceFlagList()
{
	vector<int> tmp;
	for(int i=0;i<node_list.size();i++)
	{
		tmp.push_back(node_list[i].surface_flag);
	}
	return tmp;
}

/*******************************************************************************
 * Edges
 ******************************************************************************/

void CGraph::addEmptyEdgeForNewNode(
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
					edge_list[i][ii][0].tan.resize(LOC_INT);
					edge_list[i][ii][0].nor.resize(LOC_INT);
					edge_list[i][ii][0].loc_mid.resize(LOC_INT);
					edge_list[i][ii][0].loc_len.resize(LOC_INT);
					edge_list[i][ii][0].sector_map.resize(LOC_INT*SEC_INT);
					edge_list[i][ii][0].mov_const.resize(2);
				}
			}
		}
	}
}

