//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#include "memory"
#include "Preprocessing.h"
#include "TestCase.h"
#include "festival.h"

//=============================================================================
// Global
//=============================================================================

//=============================================================================
// MAIN
//=============================================================================
int main(
		int argc,
		char *argv[])
{
	printf(
			"==============================================================================\n");
	printf(
			"|| SYSTEM START                                                             ||\n");
	printf(
			"==============================================================================\n");

	// [DATA1] *****************************************************************
#ifdef DATA1
		// Preprocessing
		if (1)
		{
			std::string kb_path
			{ "Data1/kb/" };
			std::vector<string> objs
			{ "CUP", "ORG", "SPG", "KNF" };

			std::map<std::string, std::vector<int> > idxs;
			idxs["CUP"] =
			{	1};
			idxs["ORG"] =
			{	2};
			idxs["SPG"] =
			{	3,4,5,6,7,8};
			idxs["KNF"] =
			{	9};

			Preprocessing PP;
			PP.Transitions(kb_path, objs, idxs);
			PP.Surfaces("Data1/kb/surface_.txt", "Data1/kb/surface.txt");
		}

		// Dataset evaluation
		auto TC = std::make_shared<TestCase>();
		TC->Choose(0); // label
		TC->Choose(100);
		TC->Choose(200);
		TC->Choose(300);
#endif
	// ***************************************************************** [DATA1]

	// [DATA2] *****************************************************************
#ifdef DATA2
		// Preprocessing
		if (1)
		{
			std::string kb_path
			{ "Data2/kb/" };
			std::vector<string> objs
			{ "CUP", "APP", "SPG" };

			std::map<std::string, std::vector<int> > idxs;
			idxs["CUP"] =
			{	1,2};
			idxs["APP"] =
			{	3,4};
			idxs["SPG"] =
			{	5,6};

			Preprocessing PP;
			PP.Transitions(kb_path, objs, idxs);
			PP.Surfaces("Data2/kb/surface_.txt", "Data2/kb/surface.txt");
		}

		// Dataset evaluation
		auto TC = std::make_shared<TestCase>();
		TC->Choose(1000);
		TC->Choose(2000);
		TC->Choose(3000);
#endif
	// ***************************************************************** [DATA2]

	// [AUDIO] *****************************************************************
	if (0)
	{
		{
			EST_Wave wave;
			int heap_size = 210000;  // default scheme heap size
			int load_init_files = 1; // we want the festival init files loaded

			festival_initialize(load_init_files, heap_size);

			// Say simple file
			festival_say_file("/etc/motd");

			festival_eval_command("(voice_ked_diphone)");
			// Say some text;
			festival_say_text("hello world");

			// Convert to a waveform
			festival_text_to_wave("hello world", wave);
			//wave.save("/tmp/wave.wav","riff");

			// festival_say_file puts the system in async mode so we better
			// wait for the spooler to reach the last waveform before exiting
			// This isn't necessary if only festival_say_text is being used (and
			// your own wave playing stuff)
			festival_wait_for_spooler();
		}

		{
			int heap_size = 210000;  // default scheme heap size
			int load_init_files = 1; // we want the festival init files loaded
			festival_initialize(load_init_files, heap_size);
			for (int i = 0; i < 100; i++)
			{
				//		sleep(5);
				festival_eval_command("(voice_ked_diphone)");
				festival_say_text("i think you are going to drink");
			}
		}
	}
	// ***************************************************************** [AUDIO]

	printf(
			"==============================================================================\n");
	printf(
			"|| SYSTEM END                                                               ||\n");
	printf(
			"==============================================================================\n");

	return 0;
}

