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
	grasp_flag 		= true;
	start_loc		= true;

	ac				= ac_;
	al				= al_;

	delay_factor	= delay_;
	output			= "";
	output_mem		= "";
	label			= "";
	label_mem		= "";
	repeat			= "I think";
}


Prediction::~Prediction() {}

//int Prediction::CheckContact(int x_)
//{
//	if (x_==0)
//	{
//		output = "[---] NO CONTACT\n";
//		this->Delay();
//		return EXIT_FAILURE;
//	}
//	else
//	{
//		return EXIT_SUCCESS;
//	}
//}


//void Prediction::Predict(
//	int label_,
//	string name_,
//	vector<map<string, double> > prediction_)
//{
//	if (label_<0)
//	{
//		probability.clear();
//		probability.resize(ac["GEOMETRIC"].second-ac["GEOMETRIC"].first+1);
//		for(int iii=ac["GEOMETRIC"].first;
//				iii<ac["GEOMETRIC"].second+1;
//				iii++)
//		{
//			probability[iii] = prediction_[iii][al[iii]];
//		}
//		if (*max_element(probability.begin(), probability.end()) <= 0)
//		{
//			output = "[GLA] UNKNOWN GOAL ";
//		}
//		else
//		{
//			decide =
//					distance(
//							probability.begin(),
//							max_element(
//									probability.begin(),
//									probability.end()));
//			output = "[GLA] " + al[decide] + " ";
//		}
//
//		probability.clear();
//		probability.resize(ac["MOVEMENT"].second-ac["MOVEMENT"].first+1);
//		for(int iii=ac["MOVEMENT"].first;
//				iii<ac["MOVEMENT"].second+1;
//				iii++)
//		{
//			probability[iii-ac["MOVEMENT"].first] =
//					prediction_[decide][al[iii]];
//		}
//		if (*max_element(probability.begin(), probability.end()) <= 0)
//		{
//			output += "[MOV] UNKNOWN MOV ";
//		}
//		else
//		{
//			output +=
//					"[MOV] " +
//					al[ ac["MOVEMENT"].first +
//						distance(
//								probability.begin(),
//								max_element(
//										probability.begin(),
//										probability.end())) ] +
//					" ";
//		}
//
//		probability.clear();
//		probability.resize(ac["GENERIC"].second-ac["GENERIC"].first+1);
//		for(int iii=ac["GENERIC"].first;
//				iii<ac["GENERIC"].second+1;
//				iii++)
//		{
//			probability[iii-ac["GENERIC"].first] =
//					prediction_[decide][al[iii]];
//		}
//		if (*max_element(probability.begin(), probability.end()) <= 0)
//		{
//			output += "[GEN] UNKNOWN GEN\n";
//		}
//		else
//		{
//			output +=
//					"[GEN] " +
//					al[ ac["GENERIC"].first +
//						distance(
//								probability.begin(),
//								max_element(
//										probability.begin(),
//										probability.end()))] +
//					"\n";
//		}
//	}
//	else
//	{
//		output = "[CLA] " + name_ + "\n";
//	}
//	this->Delay();
//}
//
//void Prediction::Delay()
//{
//	if(strcmp(output.c_str(),output_mem.c_str()))
//	{
//		mem_delay.first = output;
//		mem_delay.second = 1;
//	}
//	else
//	{
//		if(mem_delay.second==delay_factor)
//		{
//			mem_delay.second+=1;
//			printf("%s", output.c_str());
//		}
//		else
//		{
//			if(mem_delay.second<delay_factor)
//			{
//				mem_delay.second+=1;
//			}
//		}
//	}
//	output_mem = output;
//}

void Prediction::Delay_(state_t s_)
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

void Prediction::Parse(
	state_t s_)
{
	this->Delay_(s_);
}



//	0. Grasp
//	- no: end
//	- yes: 1.
//
//	1. Position (pct_err)
//	- -1: 2.1. (in LA)
//	- 0: end
//	- xx: 2.2. (in SM)
//
//	2.1. LA
//	- start?
//	-- yes: mov?
//	--- yes: end
//	--- no: end
//	-- no: mov?
//	--- yes: end
//	--- no: end
//
//	2.2. SM
//	- yes: 3.
//	- no: end
//
//	3. trajectory number
//	- just 1: 4.1.
//	- multiple: 4.2.
//
//	4.1. 1 trajectory
//	- pct_err > x% ?
//	--yes: end
//	--no: end
//
//	4.2. multiple trajectory
//	- ratio of max > x% ?
//	-- yes: end
//	-- no: ratio of next max - end

// GRASP
void Prediction::DT0()
{
	if (!state_mem.back().grasp)
	{
		if(!grasp_flag)
		{
			grasp_flag = true;
			output = "You have released an object...";
		}
		else
		{
			output = "Nothing is happening...";
		}
		start_loc = true;
		label  = "RELEASE";
	}
	else
	{
		if(grasp_flag)
		{
			printf("You grabbed an object.\n");
			grasp_flag = false;
		}
		this->DT1();
	}
}

// LA / SM
void Prediction::DT1()
{
	// in LA
	if (state_mem.back().pct_err<0)
	{
		this->DT2_1();
		label  = al[state_mem.back().label2];
	}
	else if (state_mem.back().pct_err==0)
	{
		output = "This is strange. I do not know what you are doing. ";
		label  = "UNKNOWN";
	}
	// in SM
	else
	{
		start_loc = false;
		this->DT2_2();
	}
}

// LA start loc?
void Prediction::DT2_1()
{
	if(start_loc)
	{
		if(state_mem.back().mov>0)
		{
			output =
					"You are moving an object that was previously " +
					al[state_mem.back().label2] + ". ";
		}
		else
		{
			output =
					"You are grabbing an object that was previously " +
					al[state_mem.back().label2] + ". ";
		}
	}
	else
	{
		if(state_mem.back().mov>0)
		{
			output = "You are " + al[state_mem.back().label2] + " something. ";
		}
		else
		{
			output =
					"You are grabbing an object. I think you should be " +
					al[state_mem.back().label2] + " something. ";
		}
	}
}

// SM eval
void Prediction::DT2_2()
{
	if(state_mem.back().mov>0)
	{
		this->DT3();
	}
	else
	{
		output =
				"You are grabbing an object. I think you will be " +
				al[state_mem.back().label2] + " something. ";
		label  = "STOP";
	}
}

// prediction on SM
void Prediction::DT3()
{
	vector<double> tmp, tmp_idx;
	for(int i=ac["GEOMETRIC"].first;i<ac["GEOMETRIC"].second+1;i++)
	{
		if(state_mem.back().goal[al[i]]>0)
		{
			tmp.push_back(state_mem.back().goal[al[i]]);
			tmp_idx.push_back(i);
		}
	}

	// only 1 confident traj
	if(tmp.size()==1)
	{
		this->DT4_1(tmp.back());
		label  = "MOVE";
	}
	// multiple confident traj
	else if(tmp.size()>1)
	{
		this->DT4_2(tmp);
		label  = "MOVE";
	}
	// unknown
	else
	{
		output = "This is strange. I do not know what you are doing. ";
		label  = "UNKNOWN";
	}
}

// only 1 confident traj
void Prediction::DT4_1(
	double x_)
{
	if(x_ > 0.5)
	{
		output =
				"I think you are going to " + al[state_mem.back().label2] +
				" something. ";
	}
	else
	{
		output =
				"I think you are going to " + al[state_mem.back().label2] +
				" something. But i am not so sure. ";
	}
}

// multiple confident traj
void Prediction::DT4_2(
	vector<double> x_)
{
	double sum_tmp = accumulate(x_.begin(), x_.end(), 0.0, addFunction);
	for(int i=0;i<x_.size();i++)
	{
		x_[i] /= sum_tmp;
	}

	if(*max_element(x_.begin(), x_.end()) > 0.33)
	{
		output =
				"I think you are going to " + al[state_mem.back().label2] +
				" something. But there are other possibilities.";
	}
	else
	{
		int idx =
					distance(
							x_.begin(),
							max_element(x_.begin(), x_.end()));
		x_.erase(x_.begin()+idx);
		idx =
					distance(
							x_.begin(),
							max_element(x_.begin(), x_.end()));
		output =
				"I think you are going to " + al[state_mem.back().label2] +
				" something. But i am not so sure. " +
				"Maybe you are going to " + al[idx] +
				" something. ";
	}
}

void Prediction::Display()
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
//		if (!s.grasp)
//		{
//			output = "RELEASE\n";
//		}
//		else if (s.pct_err<0)
//		{
//			output = al[s.label2] + "\n";
//		}
//		else
//		{
//			output = "MOVE\n";
//		}

		this->DT0();
	}

	if (strcmp(output_mem.c_str(),output.c_str()))
	{
		if (!strncmp(output_mem.c_str(),repeat.c_str(),strlen(repeat.c_str())) &&
			!strncmp(output.c_str(),repeat.c_str(),strlen(repeat.c_str())))
		{
			printf("No, %s\n", output.c_str());
		}
		else
		{
			printf("%s\n", output.c_str());
		}
	}
	output_mem = output;

	if (strcmp(label_mem.c_str(),label.c_str()))
	{
		label_mem = label;
//		cout << label << endl;
	}


//	for(int i=ac["GEOMETRIC"].first;i<ac["GEOMETRIC"].second+1;i++)
//	{
//		cout << s.goal[al[i]] << " ";
//	}
//	cout << endl;

}

