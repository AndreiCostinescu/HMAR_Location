/*******************************************************************************
 * CData.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: Container for input data.
 ******************************************************************************/

#ifndef CONTAINERDATA_H_
#define CONTAINERDATA_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "CGraph.h"
#include "CKB.h"
#include "CAS.h"

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class CData
{
	public:
		CData();
		virtual ~CData();

		CGraph *G;
		CKB *KB;
		CAS *AS;
		vector<Vector4d> pva;
		int contact;
};

#endif /* CONTAINERDATA_H_ */
