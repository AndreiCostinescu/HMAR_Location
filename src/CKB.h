/*
 * CKB.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 */

#ifndef CKB_H_
#define CKB_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class CKB
{

public:

	CKB();
	virtual ~CKB();

	vector<Vector4d> surface_eq;
	vector<Vector3d> surface_mid;
	vector<Vector3d> surface_min; // from mid
	vector<Vector3d> surface_max; // from mid
	vector<Matrix3d> surface_rot;
	vector<double>   surface_lim;
	map<int,vector<string> > label;
	map<string,pair<int,int> > ac;
	vector<string> al;
	map<string,map<string,string> >	ol;
};

#endif /* CKB_H_ */
