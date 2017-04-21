//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#include "dataDeclaration.h"
#include "TestCase.h"

//=============================================================================
// Global
//=============================================================================

//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{

	TestCase TC;

	// Label
//	TC.Choose(0, LBL);

//	// Train Cup
	TC.Choose(1, TRN);
	TC.Choose(1, TST);

//	// Train Org
//	TC.Choose(2, TRN);
//	TC.Choose(2, TST);

//	// Train Spg
//	TC.Choose(3, TRN);
//	TC.Choose(3, TST);

//	// Train Knf
//	TC.Choose(4, TRN);
//	TC.Choose(4, TST);

	return 0;
}




