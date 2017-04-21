/*
 * Parser.h
 *
 *  Created on: Apr 18, 2017
 *      Author: chen
 */

#ifndef PARSER_H_
#define PARSER_H_

#include "dataDeclaration.h"

class Parser
{
	public:
		Parser();
		virtual ~Parser();

		void ClearParser();

		int ParseDataNoLabel();
		int ParseData();

		void SetDataParser(vector<vector<string> > data_);

		vector<int>		GetFrameParser() 	{return frames;}
		vector<int>		GetContactParser()	{return contact;}
		point_d 		GetFaceParser() 	{return face;}
		vector<string> 	GetLabelParser() 	{return labels;}
		vector<point_d>	GetPointParser() 	{return points;}

	private:
		vector<vector<string> > data_parser;
		vector<int>				frames;
		vector<int>				contact;
		vector<point_d>			points;
		point_d 				face;
		vector<string> 			labels;

};

#endif /* PARSER_H_ */
