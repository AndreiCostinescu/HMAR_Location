/*
 * COS.h
 *
 *  Created on: May 22, 2017
 *      Author: chen
 */

#ifndef COS_H_
#define COS_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

class COS
{
private:
	using mSvvD = std::map<std::string, std::vector<std::vector<double> > >;

	int label_os;
	std::vector<std::string> label_list_os; // list of object state labels
	std::map<std::string,int> la_os; // list of object state labels
	mSvvD transition_os; // transition between the object state
	mSvvD transition_os_la;
	mSvvD transition_la_os;

public:
	COS();
	virtual ~COS();

	/*
	 * current os label
	 */
	virtual int OSLabel() const
	{
		return label_os;
	}
	virtual void OSLabel(int x_)
	{
		label_os = x_;
	}

	/*
	 * list of object state labels
	 */
	virtual std::vector<std::string> OSLabelList() const
	{
		return label_list_os;
	}
	virtual void OSLabelList(std::vector<std::string> x_)
	{
		label_list_os = x_;
	}

	/*
	 * list of LA object state correspondence
	 */
	virtual std::map<std::string,int> LAOSMap() const
	{
		return la_os;
	}
	virtual void LAOSMap(std::map<std::string,int> x_)
	{
		la_os = x_;
	}

	/*
	 * P(O_t|LA_t)
	 */
	virtual mSvvD TransitionLAOS() const
	{
		return transition_la_os;
	}
	virtual void TransitionLAOS(mSvvD x_)
	{
		transition_la_os = x_;
	}

	/*
	 * P(LA_t|O_t)
	 */
	virtual mSvvD TransitionOSLA() const
	{
		return transition_os_la;
	}
	virtual void TransitionOSLA(mSvvD x_)
	{
		transition_os_la = x_;
	}

	/*
	 * P(O_t|O_t-1)
	 */
	virtual mSvvD TransitionOS() const
	{
		return transition_os;
	}
	virtual void TransitionOS(mSvvD x_)
	{
		transition_os = x_;
	}
};

#endif /* COS_H_ */
