/*
 * Prediction.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#include "Prediction.h"

Prediction::Prediction(
	int delay_,
	map<string,pair<int,int> > ac_,
	vector<string> al_)
{
	ac				= ac_;
	al				= al_;

	delay_factor	= delay_;
	decide 			= -1;
	output			= "";
	output_mem		= "";
	mem_delay		= make_pair("",1);
	probability.clear();
}


Prediction::~Prediction() {
	// TODO Auto-generated destructor stub
}

int Prediction::checkContact(int x_)
{
	if (x_==0)
	{
		output = "[---] NO CONTACT\n";
		this->delay();
		return EXIT_FAILURE;
	}
	else
	{
		return EXIT_SUCCESS;
	}
}


void Prediction::predict(
	int label_,
	string name_,
	vector<map<string, double> > prediction_)
{
	if (label_<0)
	{
		probability.clear();
		probability.resize(ac["GEOMETRIC"].second-ac["GEOMETRIC"].first+1);
		for(int iii=ac["GEOMETRIC"].first;
				iii<ac["GEOMETRIC"].second+1;
				iii++)
		{
			probability[iii] = prediction_[iii][al[iii]];
		}
		if (*max_element(probability.begin(), probability.end()) <= 0)
		{
			output = "[GLA] UNKNOWN GOAL ";
		}
		else
		{
			decide =
					distance(
							probability.begin(),
							max_element(
									probability.begin(),
									probability.end()));
			output = "[GLA] " + al[decide] + " ";
		}

		probability.clear();
		probability.resize(ac["MOVEMENT"].second-ac["MOVEMENT"].first+1);
		for(int iii=ac["MOVEMENT"].first;
				iii<ac["MOVEMENT"].second+1;
				iii++)
		{
			probability[iii-ac["MOVEMENT"].first] =
					prediction_[decide][al[iii]];
		}
		if (*max_element(probability.begin(), probability.end()) <= 0)
		{
			output += "[MOV] UNKNOWN MOV ";
		}
		else
		{
			output +=
					"[MOV] " +
					al[ ac["MOVEMENT"].first +
						distance(
								probability.begin(),
								max_element(
										probability.begin(),
										probability.end())) ] +
					" ";
		}

		probability.clear();
		probability.resize(ac["GENERIC"].second-ac["GENERIC"].first+1);
		for(int iii=ac["GENERIC"].first;
				iii<ac["GENERIC"].second+1;
				iii++)
		{
			probability[iii-ac["GENERIC"].first] =
					prediction_[decide][al[iii]];
		}
		if (*max_element(probability.begin(), probability.end()) <= 0)
		{
			output += "[GEN] UNKNOWN GEN\n";
		}
		else
		{
			output +=
					"[GEN] " +
					al[ ac["GENERIC"].first +
						distance(
								probability.begin(),
								max_element(
										probability.begin(),
										probability.end()))] +
					"\n";
		}
	}
	else
	{
		output = "[CLA] " + name_ + "\n";
	}
	this->delay();
}

void Prediction::delay()
{
	if(strcmp(output.c_str(),output_mem.c_str()))
	{
		mem_delay.first = output;
		mem_delay.second = 1;
	}
	else
	{
		if(mem_delay.second==delay_factor)
		{
			mem_delay.second+=1;
			printf("%s", output.c_str());
		}
		else
		{
			if(mem_delay.second<delay_factor)
			{
				mem_delay.second+=1;
			}
		}
	}
	output_mem = output;
}

void Prediction::delay_(state_t s_)
{
	if (state_mem.size() > 0)
	{
		if (s_.label2 != state_mem.back().label2)
		{
			state_mem.clear();
		}

		state_mem.push_back(s_);

		if (state_mem.size() > delay_factor)
		{
			state_mem.erase(state_mem.begin());
		}
	}
	else
	{
		state_mem.push_back(s_);
	}
}

void Prediction::display()
{
	state_t s = state_mem.back();

	if (state_mem.size() >= delay_factor)
	{
//		if (!s.grasp)
//		{
//			output = "NOTHING\n";
//		}
//		else if (s.pct_err<0)
//		{
//			output = "IN LA\n";
//		}
//		else
//		{
//			if(s.grasp)	output = "Grasp ";
//			else		output = "Release ";
//
//			if (s.label1 < 0)
//				output += "from UNKNOWN ";
//			else
//				output += "from " + al[s.label1] + " ";
//
//			if (s.label2 < 0)
//				output += "to UNKNOWN ";
//			else
//				output += "to " + al[s.label2] + " ";
//
//			output += "with " + to_string(s.pct_err) + " ";
//			output += "moving at " + to_string(s.mov) + " ";
//			output += "contact " + to_string(s.con) + " ";
//			output += "surface " + to_string(s.sur) + "\n";
//		}
		if (!s.grasp)
		{
			output = "RELEASE\n";
		}
		else if (s.pct_err<0)
		{
			output = al[s.label2] + "\n";
		}
		else
		{
			output = "MOVE\n";
		}
	}

	if (strcmp(output_mem.c_str(),output.c_str()))
		printf("%s", output.c_str());

	output_mem = output;

//	for(int i=ac["GEOMETRIC"].first;i<ac["GEOMETRIC"].second+1;i++)
//	{
//		cout << s.goal[al[i]] << " ";
//	}
//	cout << endl;

}

void Prediction::parse(
	state_t s_)
{
	this->delay_(s_);
}

