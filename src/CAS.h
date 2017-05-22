/*******************************************************************************
 * CAS.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: ActionState container.
 ******************************************************************************/

#ifndef CAS_H_
#define CAS_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

class CAS
{

public:

	CAS();
	virtual ~CAS();

	int grasp;					// if it is grasped
	int label1;					// last node
	int label2;					// next node
	double vel;					// velocity
	double pct_err;				// highest probability
	int surface_flag;			// which surface
	string surface_name;		// which surface
	double surface_dist;		// surface distance
	map<string,double> goal;	// probabilities
	map<string,double> window;	// window radius
};

#endif /* CAS_H_ */
