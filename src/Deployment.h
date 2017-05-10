/*
 * Deployment.h
 *
 *  Created on: May 8, 2017
 *      Author: chen
 */

#ifndef DEPLOYMENT_H_
#define DEPLOYMENT_H_

#include "ActionPrediction.h"
#include "DataFilter.h"

class Deployment : 	public ActionPrediction,
					public DataFilter
{

	public:

		Deployment();
		virtual ~Deployment();

		int Deploy(
			Vector4d point_,
			int contact_);

};

#endif /* DEPLOYMENT_H_ */
