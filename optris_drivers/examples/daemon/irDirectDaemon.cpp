
#include <iostream>

#ifdef _WIN32
//needed before windows.h ... 
#include<winsock2.h>
#include<WS2tcpip.h>
#else
#include <csignal>
#endif

// Optris logging interface
#include <IRLogger.h>
// Class wrapping callback routines
#include "IRDaemon.h"

#define DIRECT_DAEMON_PORT 1337

evo::IRDaemon* _daemon = NULL;

#ifdef _WIN32
BOOL ctrlHandler(DWORD fdwCtrlType)
{
  if(_daemon) _daemon->exit();
	return true;
}
#else
void sigHandler(int signum)
{
  if(_daemon) _daemon->exit();
}
#endif

int main(int argc, char* argv[])
{

  //-- Adjust Log level ------------------------------------------------------------
  evo::IRLogger::setVerbosity(evo::IRLOG_ERROR, evo::IRLOG_OFF, "daemon.log");
  //--------------------------------------------------------------------------------


  //-- Install signal handler-------------------------------------------------------
#ifdef _WIN32
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlHandler, TRUE);
#else
	std::signal(SIGINT, sigHandler);
#endif
	//--------------------------------------------------------------------------------
	

	//-- Read device parameters ------------------------------------------------------
	evo::IRDeviceParams params;
	if(argc < 2)
	{
    std::cout << "No parameters given, taking default parameters. If you want to specify your own parameters, pass an xml file:" << std::endl << " ";
		std::cout << argv[0] << " <path to xml file>" << std::endl;
		IRDeviceParams_InitDefault(params);
	}
  else
  {
    evo::IRDeviceParamsReader::readXMLC(argv[1], params);
  }
  IRDeviceParams_Print(params);
  //--------------------------------------------------------------------------------


  //-- IRDaemon instantiation ------------------------------------------------------
  _daemon = new evo::IRDaemon();

  _daemon->run(&params, DIRECT_DAEMON_PORT);

  delete _daemon;
  //--------------------------------------------------------------------------------

	return 0;
}
