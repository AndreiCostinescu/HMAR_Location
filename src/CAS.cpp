/*******************************************************************************
 * CAS.cpp
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: ActionState container.
 ******************************************************************************/

#include "CAS.h"

CAS::CAS():	grasp(0),
			label1(-1),
			label2(-1),
			vel(0.0),
			pct_err(-1),
			surface_flag(0),
			surface_name(""),
			surface_dist(0.0)
{
}

//int grasp;					// if it is grasped
//int label1;					// last node
//int label2;					// next node
//double vel;					// velocity
//double pct_err;				// highest probability
//int surface_flag;			// which surface
//string surface_name;		// which surface
//double surface_dist;		// surface distance
//map<string,double> goal;	// probabilities
//map<string,double> window;	// window radius

CAS::~CAS() { }
