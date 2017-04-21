/*
 * Filter.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Filter.h"

Filter::Filter() { }

Filter::~Filter() { }

void Filter::ResetFilter()
{
	reshapeVector(pos_vel_acc_mem,3); //pva
	contact_mem.clear();
}

int Filter::PreprocessDataLive(
	point_d pos_,
	vector<point_d> &pva_avg_, //motion
	unsigned int window_)
{
	point_d vel = minusPoint(pos_, pva_avg_[0]);
	point_d acc = minusPoint(vel , pva_avg_[1]);
	vector<point_d> tmp; tmp.resize(3);
	tmp[0] = pos_; tmp[1] = vel; tmp[2] = acc;
	for(int i=0;i<3;i++)
	{
		if(pos_vel_acc_mem[i].size() == window_)
		{
			pva_avg_[i] = movingAverage(tmp[i], pos_vel_acc_mem[i]);
		}
		else if (pos_vel_acc_mem[i].size()>0)
		{
			pva_avg_[i] = averagePointIncrement(tmp[i], pos_vel_acc_mem[i]);
		}
		else
		{
			pos_vel_acc_mem[i].push_back(tmp[i]);
			pva_avg_[i] 	= tmp[i];
			pva_avg_[i].l 	= UNCLASSIFIED;
			for(int ii=i+1;ii<3;ii++)
			{
				pva_avg_[ii].x =
				pva_avg_[ii].y =
				pva_avg_[ii].z =
				pva_avg_[ii].l = UNCLASSIFIED;
			}
			break;
		}
		pva_avg_[i].l = UNCLASSIFIED;
	}
	return EXIT_SUCCESS;
}

int Filter::PreprocessContactLive(
	int &contact_,
	unsigned int window_)
{
	if(contact_mem.size() == window_)
	{
		contact_ = movingAverage(contact_, contact_mem);
	}
	else if (contact_mem.size()>0)
	{
		contact_ = movingAverageIncrement(contact_, contact_mem);
	}
	else
	{
		contact_mem.push_back(contact_);
	}
	return EXIT_SUCCESS;
}
