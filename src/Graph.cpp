/*
 * Graph.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#include "Graph.h"

void Graph::addNode(
	string name_,
	vector<data_t> data_,
	unsigned int category_,
	int surface_num_)
{
	node.name     		= name_;
	node.index    		= nodes.size();
	node.category 		= category_;
	node.surface_num	= surface_num_;
	node.data    	 	= data_;
	nodes.push_back(node);
	edges.push_back(emptyEdgeList());
}

void Graph::extendNode(
	vector<data_t> data_,
	unsigned int node_num)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data used to extend node is empty." << endl;
	else if(data_.size()==1)
		nodes[node_num].data.push_back(data_[0]);
	else
		nodes[node_num].data.insert(nodes[node_num].data.end(),
				                    data_.begin(), data_.end());
}

bool Graph::checkNode(
	unsigned int node_index1_)
{
	check_flag = false;
	if(nodes.size() > 0)
		if(nodes.size()-1 >= node_index1_)
			check_flag = true;
	return check_flag;
}

void Graph::addEdge(
	unsigned int node_index1_,
	unsigned int node_index2_,
	vector<data_t> data_)
{
	edge.begin_index = node_index1_;
	edge.end_index   = node_index2_;
	edge.cost 	     = 0;
	edge.data        = data_;
	edges[node_index1_].push_back(edge);
}

void Graph::extendEdge(
	unsigned int node_index1_,
	unsigned int node_index2_,
	vector<data_t> data_,
	unsigned int edge_num_)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data to extend edge is empty." << endl;
	else if(data_.size()==1)
		edges[node_index1_][edge_num_].data.push_back(data_[0]);
	else
		edges[node_index1_][edge_num_].data.insert(
				edges[node_index1_][edge_num_].data.end(),
				data_.begin(), data_.end());
}

bool Graph::checkEdge(
	unsigned int node_index1_,
	unsigned int node_index2_,
	unsigned int &edge_num_ )
{
	check_flag = false;
	if(edges[node_index1_].size()>0)
		for(int i=0;i<edges[node_index1_].size();i++)
			if(edges[node_index1_][i].end_index == node_index2_)
			{
				check_flag = true;
				edge_num_ = (unsigned int)i;
			}
	return check_flag;
}

vector<vector<double> > Graph::getEdgeDataLabel(
	bool pos_,
	bool vel_,
	bool acc_)
{
	vector<vector<double> > output;
	vector<vector<edge_tt> > list = getEdgeList();
	vector<data_t> data;
	// list of nodes with edges
	for(int i=0; i<list.size();i++)
	{
		// list of edges for each node
		for(int ii=0; ii<list[i].size();ii++)
		{
			data = list[i][ii].data;
			// list of data in each edge
			for(int iii=0; iii<data.size();iii++)
			{
				vector<double> tmp_out;
				tmp_out.push_back(i);
				if(pos_)
				{
					point_t tmp_pos = data[iii].pos;
					tmp_out.push_back(tmp_pos.x);
					tmp_out.push_back(tmp_pos.y);
					tmp_out.push_back(tmp_pos.z);
				}
				if(vel_)
				{
					point_t tmp_vel = data[iii].vel;
					tmp_out.push_back(tmp_vel.x);
					tmp_out.push_back(tmp_vel.y);
					tmp_out.push_back(tmp_vel.z);
				}
				if(acc_)
				{
					point_t tmp_acc = data[iii].acc;
					tmp_out.push_back(tmp_acc.x);
					tmp_out.push_back(tmp_acc.y);
					tmp_out.push_back(tmp_acc.z);
				}
				output.push_back(tmp_out);
			}
		}
	}
	return output;
}

vector<vector<double> > Graph::getNodeDataLabel(
	bool pos_,
	bool vel_,
	bool acc_)
{
	vector<vector<double> > output;
	vector<node_tt> list = getNodeList();
	vector<data_t> data;
	// list of nodes
	for(int i=0; i<list.size();i++)
	{
		//printf("Category %02d %02d\n", i, list[i].category);
		data = list[i].data;
		// list of node data
		for(int ii=0; ii<data.size();ii++)
		{
			vector<double> tmp_out;
			if(pos_)
			{
				point_t tmp_pos = data[ii].pos;
				tmp_out.push_back(tmp_pos.x);
				tmp_out.push_back(tmp_pos.y);
				tmp_out.push_back(tmp_pos.z);
				tmp_out.push_back(i);
			}
			if(vel_)
			{
				point_t tmp_vel = data[ii].vel;
				tmp_out.push_back(tmp_vel.x);
				tmp_out.push_back(tmp_vel.y);
				tmp_out.push_back(tmp_vel.z);
				tmp_out.push_back(i);
			}
			if(acc_)
			{
				point_t tmp_acc = data[ii].acc;
				tmp_out.push_back(tmp_acc.x);
				tmp_out.push_back(tmp_acc.y);
				tmp_out.push_back(tmp_acc.z);
				tmp_out.push_back(i);
			}
			output.push_back(tmp_out);
		}
	}
	return output;
}
