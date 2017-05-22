/*
 * CData.cpp
 *
 *  Created on: May 20, 2017
 *      Author: chen
 */

#include "CData.h"

CData::CData() : G(NULL), KB(NULL), AS(NULL), contact(0)
{
	pva.clear();
	pva.resize(3);
}

CData::~CData() { }

