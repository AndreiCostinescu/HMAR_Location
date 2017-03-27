/*
 * util.h
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#ifndef UTIL_H_
#define UTIL_H_


#include "readWriteFile.h"
#include "dataDeclaration.h"
#include "algo.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "labeling_loc.h"
#include "labeling_sec.h"
#include "predicting.h"
#include "misc.h"

// ============================================================================
// Modules
// ============================================================================

int learnLocationArea(
	string dirname_,
	string scene,
	string object);

int learning(
	string dirname_,
	string scene,
	string object);

int testing(
	string dirname_,
	string scene,
	string object);

// ============================================================================
// Data
// ============================================================================
void parseData2Point(
	vector<vector<string> > data_,
	vector<point_d> 		&points_,
	vector<int> 			&contact_);

void preprocessDataLive(
	point_d 					pos_,
	vector< vector< point_d > > &pos_vel_acc_mem_, // motion -> length(empty at beginning)
	vector<point_d> 			&pos_vel_acc_avg_, //motion
	unsigned int 				window_);


#endif /* UTIL_H_ */
