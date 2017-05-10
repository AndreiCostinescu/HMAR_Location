/*
 * Deployment.cpp
 *
 *  Created on: May 8, 2017
 *      Author: chen
 */

#include "Deployment.h"

Deployment::Deployment() { }

Deployment::~Deployment() { }

int Deployment::Deploy(
	Vector4d point_,
	int contact_)
{
	// 1. Filter
	this->PreprocessDataLive(point_, pva, FILTER_WIN);
	this->PreprocessContactLive(contact_, contact, FILTER_WIN);

	// 2. Prediction
	this->PredictExt();

	return EXIT_SUCCESS;
}
