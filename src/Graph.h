/*
 * Graph.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "dataDeclaration.h"

class Graph
{
public:
	Graph(){};

	node_tt getNode(unsigned int node_index){return nodes[node_index];}
	vector<node_tt> getNodeList(){return nodes;}
	vector<vector<edge_tt> > getEdgeList(){return edges;}

	void addNode(
		string name_,
		vector<data_t> data_,
		unsigned int category_,
		int surface_num_);
	void extendNode(
		vector<data_t> data_,
		unsigned int node_num_);
	bool checkNode(
		unsigned int node_index_);
	void addEdge(
		unsigned int node_index1_,
		unsigned int node_index2_,
		vector<data_t> data_);
	void extendEdge(
		unsigned int node_index1_,
		unsigned int node_index2_,
		vector<data_t> data_,
		unsigned int edge_num_);
	bool checkEdge(
		unsigned int node_index1_,
		unsigned int node_index2_,
		unsigned int &edge_num_ );
	vector<vector<double> > getNodeDataLabel(
		bool pos_=false,
		bool vel_=false,
		bool acc_=false);
	vector<vector<double> > getEdgeDataLabel(
		bool pos_=false,
		bool vel_=false,
		bool acc_=false);

	virtual ~Graph(){}

private:
	bool check_flag;
	node_tt node;
	edge_tt edge;
	vector<node_tt> nodes;
	vector<vector<edge_tt> > edges;
	vector<edge_tt> emptyEdgeList()
	{
		vector<edge_tt> empty_edge_list;
		return empty_edge_list;
	}
};


#endif /* GRAPH_H_ */
