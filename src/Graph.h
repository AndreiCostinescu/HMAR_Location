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

	Graph(string object_);
	virtual ~Graph(){}

//=============================================================================
// Set/Get
//=============================================================================

	string GetObject() {return object;}
	void SetObject(string object_) {object = object_;}

	kb_t GetKB();
	void SetKB(kb_t x_);

	vector<double> GetSurfaceLimit() {return surface_lim;}
	void SetSurfaceLimit(vector<double> x_) {surface_lim = x_;}

	vector<vector<double> > GetSurfaceEq() {return surface_eq;}
	void SetSurfaceEq(vector<vector<double> > x_) {surface_eq = x_;}

	vector<point_d> GetSurface() {return surface;}
	void SetSurface(vector<point_d> x_) {surface = x_;}

	map<string,map<string,string> > GetObjectLabel() {return object_label;}
	void SetObjectLabel(map<string,map<string,string> > object_label_) {object_label = object_label_;}

	map<string,pair<int,int> > GetActionCategory() {return action_cat;}
	void SetActionCategory(map<string,pair<int,int> > x_) {action_cat = x_;}

	vector<string> GetActionLabel() {return action_label;}
	void SetActionLabel(vector<string> x_) {action_label = x_;}

	void GetPredictionReset(vector<map<string, double> > &x_) {x_ = prediction_reset;}

	void GetPrediction(vector<map<string, double> > &x_) {x_ = prediction;}
	void SetPrediction(vector<map<string, double> > x_) {prediction = x_;}

	state_t GetActionState() {return action_state;}
	void SetActionState(state_t x_) {action_state = x_;}

//=============================================================================
// NODES
//=============================================================================

	int GetNode(
		int  idx_,
		node_tt &x_)
	{
		if (idx_<node_list.size())
		{
			x_ = node_list[idx_];
			return EXIT_SUCCESS;
		}
		else
		{
			return EXIT_FAILURE;
		}
	}

	int SetNode(
		node_tt	node_)
	{
		if (node_list.size() < node_.index+1)
		{
			node_list.resize(node_.index+1);
		}
		node_list[node_.index] = node_;
		return EXIT_SUCCESS;
	}

	int GetNumberOfNodes()
	{
		return node_list.size();
	}

	vector<node_tt> GetNodeList()
	{
		return node_list;
	}

	vector<point_d> GetCentroidList();

	vector<int> GetSurfaceFlagList();

//=============================================================================
// EDGES
//=============================================================================

	void addEmptyEdgeForNewNode(
		int	idx_);

	vector<vector<vector<edge_tt> > > GetListOfEdges()
	{
		return edge_list;
	}

	edge_tt GetEdge(
		int n1_,
		int n2_,
		int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_];
	}

	void SetEdge(
		int 	n1_,
		int 	n2_,
		int 	edge_num_,
		edge_tt edge_)
	{
		edge_list[n1_][n2_][edge_num_] = edge_;
	}

	int GetEdgeCounter(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_].counter;
	}

	void SetEdgeCounter(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		unsigned int x_)
	{
		edge_list[n1_][n2_][edge_num_].counter += x_;
	}

	void GetEdgeMovementConstraint(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<int> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].mov_const;
	}

	void SetEdgeMovementConstraint(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<int> x_)
	{
		edge_list[n1_][n2_][edge_num_].mov_const = x_;
	}

	void GetEdgeLocMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].loc_mem;
	}

	void SetEdgeLocMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].loc_mem = x_;
	}

	void GetEdgeSecMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].sec_mem;
	}

	void SetEdgeSecMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].sec_mem = x_;
	}

	void GetEdgeSectorMap(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].sector_map;
	}

	void SetEdgeSectorMap(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].sector_map = x_;
	}

	void GetEdGetan(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].tan;
	}

	void SetEdGetan(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> x_)
	{
		edge_list[n1_][n2_][edge_num_].tan = x_;
	}

	void GetEdgeNor(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].nor;
	}

	void SetEdgeNor(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> x_)
	{
		edge_list[n1_][n2_][edge_num_].nor = x_;
	}


private:
	string 								object; // what kind of action Set is being evaluated
	vector<node_tt> 					node_list;
	vector<vector<vector<edge_tt> > > 	edge_list;
	map<string, double>					filter_init;
	map<string, double>					prediction_init;
	vector<map<string, double> >		filter;
	vector<map<string, double> >		prediction;
	vector<map<string, double> >		prediction_reset;
	map<string,pair<int,int> >			action_cat;
	vector<string>						action_label; //fixed given beforehand

	map<string,map<string,string> > 	object_label;

	vector<point_d> 					surface;
	vector<vector<double> > 			surface_eq;
	vector<double> 						surface_lim;

	kb_t 								kb;
	state_t 							action_state;

};


#endif /* GRAPH_H_ */
