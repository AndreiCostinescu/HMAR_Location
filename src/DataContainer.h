/*
 * DataContainer.h
 *
 *  Created on: May 8, 2017
 *      Author: chen
 */

#ifndef DATACONTAINER_H_
#define DATACONTAINER_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "misc.h"

class DataContainer
{

	public:
		DataContainer();
		virtual ~DataContainer();

	protected:
		Graph *G;

};

#endif /* DATACONTAINER_H_ */
