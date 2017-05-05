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

		vector<int>		GetFrameParser() 	{return frames_parser;}
		vector<int>		GetContactParser()	{return contact_parser;}
		point_d 		GetFaceParser() 	{return face_parser;}
		vector<string> 	GetLabelParser() 	{return labels_parser;}
		vector<point_d>	GetPointParser() 	{return points_parser;}

	protected:
		vector<int>				frames_parser;
		vector<int>				contact_parser;
		vector<point_d>			points_parser;
		point_d 				face_parser;
		vector<string> 			labels_parser;

	private:
		vector<vector<string> > data_parser;

};

#endif /* PARSER_H_ */
