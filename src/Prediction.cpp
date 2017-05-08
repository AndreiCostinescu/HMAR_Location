/*
 * Prediction.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#include "Prediction.h"

Prediction::Prediction(
	string obj_,
	map<string,pair<int,int> > ac_,
	vector<string> al_,
	map<string,map<string,string> > ol_,
	int delay_)
{
	grasp_flag 		= true;
	start_loc		= true;

	ac				= ac_;
	al				= al_;
	ol				= ol_;

	obj				= obj_;

	delay_factor	= delay_;
	output			= "";
	output_mem		= "";
	label			= "";
	label_mem		= "";
	repeat			= "I think";

	this->Init();

	dict[2]  = "Nothing is happening.";
	dict[3]  = "You are moving the ";

}

Prediction::~Prediction() {}

void Prediction::Init( )
{
//	dict[0]  = " ";
//	dict[1]  = ".";
//	dict[10] = "the";
//	dict[11] = "has";
//	dict[12] = "been";
//	dict[13] = "released";
//	dict[14] = "nothing";
//	dict[15] = "is";
//	dict[16] = "happening";
//	dict[17] = "you";
//	dict[18] = "are";
//	dict[19] = "moving";
//	dict[20] = "that";
//	dict[21] = "was";
//	dict[22] = "should";
//	dict[23] = "be";
//	dict[24] = "but";
//	dict[25] = "grabbing";
//	dict[26] = "have";
//	dict[27] = "stopped";
//	dict[28] = "i think";
//	dict[29] = "going";
//	dict[30] = "to";
//	dict[31] = "i am";
//	dict[32] = "not";
//	dict[33] = "sure";
//	dict[34] = "maybe";
//	dict[35] = "i don't";
//	dict[36] = "know";
//	dict[37] = "whats";
//	dict[38] = "happening";
//
//	int n = 11;
//	reshapeVector(msg,n);
//
//	msg[0]  = {10,0,-1,0,11,0,12,0,13,1};
//	msg[1]  = {14,0,15,0,16,1};
//	msg[2]  = {17,0,18,0,19,0,10,0,-1,0,20,0,21,0,-3,1};
//	msg[3]  = {17,0,18,0,25,0,10,0,-1,0,20,0,21,0,-3,1};
//	msg[4]  = {17,0,18,0,-3,1};
//	msg[5]  = {17,0,26,0,27,0,-3,1};
//	msg[6]  = {28,0,17,0,18,0,29,0,30,0,-3,1};
//	msg[7]  = {28,0,17,0,18,0,29,0,30,0,-3,1,24,0,17,0,26,0,27,1};
//	msg[8]  = {34,0,17,0,18,0,29,0,30,0,-3,1,31,0,32,0,33,1};
//	msg[9]  = {34,0,17,0,18,0,29,0,30,0,-3,1,24,0,17,0,26,0,27,1};
//	msg[10] = {35,0,36,0,37,0,38,1};

	string path = "kb/message.txt";
	vector<vector<string> > data;
	ifstream src_file(path);
	while (src_file)
	{
		string file_line_;
		if (!getline( src_file, file_line_ )) break;
		istringstream line_( file_line_ );
		vector <string> data_line_;
		while (line_)
		{
		   string word;
		   if (!getline( line_, word, ' ')) break;
		   data_line_.push_back( word );
		}
		data.push_back( data_line_ );
	}

	int n = 11;
	reshapeVector(message,n);
	for(int i=0;i<11;i++)
	{
		message[0]  = data[0];
		message[1]  = data[1];
		message[2]  = data[2];
		message[3]  = data[3];
		message[4]  = data[4];
		message[5]  = data[5];
		message[6]  = data[6];
		message[7]  = data[7];
		message[8]  = data[8];
		message[9]  = data[9];
		message[10] = data[10];
	}

}

string Prediction::Decode(
	string obj_,
	string loc_,
	vector<string> msg_)
{
	string output = "";

//	for(int i=0;i<msg_.size();i++)
//	{
//		if (msg_[i] == -1)
//			output = output + obj_;
//		else if (msg_[i] == -2)
//			output = output + loc_;
//		else if (msg_[i] == -3)
//		{
//			output = output + ob_ac;
//			if (msg_[i-2] == 18)
//				output = output + "ing";
//			else if (msg_[i-2] == 21)
//				output = output + "ed";
//		}
//		else
//			output = output + dict[msg_[i]];
//	}

	for(int i=0;i<msg_.size();i++)
	{
		if (!strcmp(msg_[i].c_str(),"object"))
			output = output + obj_;
		else if (!strcmp(msg_[i].c_str(),"location"))
			output = output + loc_;
		else if (!strcmp(msg_[i].c_str(),"action"))
		{
			output = output + ob_ac;
			if (!strcmp(msg_[i-2].c_str(),"are"))
				output = output + "ing";
			else if (!strcmp(msg_[i-2].c_str(),"was"))
				output = output + "ed";
		}
		else
			output = output + msg_[i];

		output = output + " ";
	}

	return output;
}

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
	this->Display();
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
			output = this->Decode(obj, "", message[0]);
		}
		else
		{
			output = this->Decode(obj, "", message[1]);
		}
		start_loc = true;
		label  = "RELEASE";
	}
	else
	{
		if(grasp_flag)
		{
			printf("the %s has been grabbed .\n", obj.c_str());
			this->DT1();
			grasp_flag = false;
		}
		else
		{
			this->DT1();
		}
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
	if(grasp_flag)
	{
		if(state_mem.back().mov>0)
		{
			output = this->Decode(obj, "", message[2]);
		}
		else
		{
			output = this->Decode(obj, "", message[3]);
		}
	}
	else
	{
		if(state_mem.back().mov>0)
		{
			output = this->Decode(obj, "", message[4]);
		}
		else
		{
			output = this->Decode(obj, "", message[5]);
		}
	}
}

// SM eval
void Prediction::DT2_2()
{
	this->DT3();
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
	}
	// multiple confident traj
	else if(tmp.size()>1)
	{
		this->DT4_2(tmp);
	}
	// unknown
	else
	{
		output = this->Decode(obj, "", message[10]);
		label  = "UNKNOWN";
	}
}

// only 1 confident traj
void Prediction::DT4_1(
	double x_)
{
	if(state_mem.back().mov>0)
	{
		output = this->Decode(obj, "", message[6]);
		label  = "MOVE";
	}
	else
	{
		output = this->Decode(obj, "", message[7]);
		label  = "STOP";
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
		if(state_mem.back().mov>0)
		{
			output = this->Decode(obj, "", message[6]);
			label  = "MOVE";
		}
		else
		{
			output = this->Decode(obj, "", message[7]);
			label  = "STOP";
		}
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
		if(state_mem.back().mov>0)
		{
			output = this->Decode(obj, "", message[8]);
			output = output + "Perhaps you are going to " + al[idx] + ".";
			label  = "MOVE";
		}
		else
		{
			output = this->Decode(obj, "", message[9]);
			label  = "STOP";
		}
	}
}

void Prediction::Display()
{
	if (state_mem.size() >= delay_factor)
	{
		if(state_mem.back().label2 < 0)
		{
			ob_ac = "";
		}
		else
		{
			ob_ac =
					(ol[obj][al[state_mem.back().label2]].empty() ?
							al[state_mem.back().label2] :
							ol[obj][al[state_mem.back().label2]]);
		}
		this->DT0();
	}

	// Extra info for transitions
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
	}

}

