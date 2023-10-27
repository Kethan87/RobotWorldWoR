#include "Config.hpp"

#include "MainApplication.hpp"

#include "Logger.hpp"
#include "Trace.hpp"
#include "FileTraceFunction.hpp"
#include "CompassLidarSensor.hpp"
#include "CompassOdometerSensor.hpp"

#include <iostream>
#include <string>
#include <stdexcept>

// \file

/**
 * @see Application::MainApplication::setCommandlineArguments
 *
 * @param argc The number of arguments
 * @param argv The value of the arguments
 * @return 0 on success, undefined integer otherwise
 */
int main( 	int argc,
			char* argv[])
{

	//Base::Trace::setTraceFunction( std::make_unique<Base::FileTraceFunction>("trace", "log", true));

	if(argc == 1)
	{
		Application::Logger::log(std::string("No arguments are given. The default standard deviations, will be used."));
		Model::odometerSttdev = 1;
		Model::compasStddev = 2;
		Model::lidarStddev = 10;
	} else
	{
		std::ifstream inputFile(argv[1]);
		if (!inputFile.is_open()) {
		        std::cerr << "Error: Could not open the file." << std::endl;
		        Application::Logger::log(std::string("No arguments are given. The default standard deviations, will be used."));
		        Model::odometerSttdev = 1;
		        Model::compasStddev = 2;
		        Model::lidarStddev = 10;
		        return -1;
		  }
		std::string line;
		int i = 0;
		while (std::getline(inputFile, line)) {
			std::cout << line << std::endl;
			if(i == 0)
			{
				Model::odometerSttdev = std::stoi(line);
			} else if (i == 1) {
				Model::odometerSttdev = std::stoi(line);
				Model::compasStddev = std::stoi(line);
			} else if (i == 2) {
				Model::odometerSttdev = std::stoi(line);
				Model::compasStddev = std::stoi(line);
				Model::lidarStddev = std::stoi(line);
			} else {
				Application::Logger::log(std::string("Too many arguments are given. The default standard deviations, will be used."));
				Model::odometerSttdev = 1;
				Model::compasStddev = 2;
				Model::lidarStddev = 10;
			}
			i++;
		}
	}
	try
	{
		// Call the wxWidgets main variant
		// This will actually call Application
		int result = runGUI( argc, argv);
		return result;
	}
	catch (std::exception& e)
	{
		Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": ") + e.what());
		std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
	}
	catch (...)
	{
		Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": unknown exception"));
		std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
	}
	return 0;
}
