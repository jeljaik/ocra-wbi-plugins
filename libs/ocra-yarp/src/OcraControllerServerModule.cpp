/*! \file       OcraControllerServerModule.h
 *  \brief      Module class for the controller server.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-yarp.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ocra-yarp/OcraControllerServerModule.h"

using namespace ocra_yarp;

OcraControllerServerModule::OcraControllerServerModule()
: controller_options(OcraControllerOptions())
{
}

bool OcraControllerServerModule::configure(yarp::os::ResourceFinder &rf)
{
    /*!< Parse the configuration file. */

    controller_options.robotName = rf.check("robot") ? rf.find("robot").asString().c_str() : "icub";
    controller_options.threadPeriod = rf.check("threadPeriod") ? rf.find("threadPeriod").asInt() : DEFAULT_THREAD_PERIOD;
    controller_options.serverName = rf.check("local") ? rf.find("local").asString().c_str() : "OcraControllerServer";
    controller_options.runInDebugMode = rf.check("debug");
    controller_options.isFloatingBase = rf.check("floatingBase");

    if( rf.check("taskSet") )
    {
        std::string xmlExt = ".xml";
        std::string fileName = rf.find("taskSet").asString().c_str();

        std::size_t fileExtensionStart = fileName.find_last_of(".");
        if (fileExtensionStart == std::string::npos) {
            fileName += xmlExt;
        }else{
            std::string extension = fileName.substr(fileExtensionStart);
            if (extension != xmlExt) {
                fileName = fileName.substr(0, fileExtensionStart) + xmlExt;
            }
        }
        std::size_t subDirEnd = fileName.find_first_of("/");
        if (subDirEnd == std::string::npos) {
            fileName = "taskSets/" + fileName;
        }else{
            std::string subDir = fileName.substr(0,subDirEnd);
            if (subDir != "taskSets/") {
                fileName = "taskSets/" + fileName.substr(subDirEnd+1);
            }
        }

        controller_options.startupTaskSetPath = rf.findFileByName(fileName).c_str();
    }

    if( rf.check("sequence") )
    {
        controller_options.startupSequence = rf.find("sequence").asString().c_str();
    }



    // Get wbi options from the canonical file
    if ( !rf.check("wbi_conf_file") )
    {
        fprintf(stderr, "[ERR] wocraController: Impossible to open wholeBodyInterface: wbi_conf_file option missing");
    }
    std::string wbiConfFile = rf.findFile("wbi_conf_file");
    controller_options.yarpWbiOptions.fromConfigFile(wbiConfFile);
    // Overwrite the robot parameter that could be present in wbi_conf_file
    controller_options.yarpWbiOptions.put("robot", controller_options.robotName);
    robotInterface = std::make_shared<yarpWbi::yarpWholeBodyInterface>(controller_options.serverName.c_str(), controller_options.yarpWbiOptions);

    wbi::IDList robotJoints;
    std::string robotJointsListName = "ROBOT_MAIN_JOINTS";
    if(!yarpWbi::loadIdListFromConfig(robotJointsListName, controller_options.yarpWbiOptions, robotJoints))
    {
        fprintf(stderr, "[ERR] wocraController: Impossible to load wbiId joint list with name %s\n", robotJointsListName.c_str());
    }
    robotInterface->addJoints(robotJoints);


    // Make sure all the add* functions are done before the "init"
    if(!robotInterface->init())
    {
        fprintf(stderr, "Error while initializing whole body interface. Closing module\n"); return false;
    }

    ctrlThread = std::make_shared<OcraControllerServerThread>(controller_options, robotInterface);

    if(!ctrlThread->start())
    {
        fprintf(stderr, "Error while initializing wocraController thread. Closing module.\n"); return false;
    }

    fprintf(stderr,"wocraController thread started\n");

    return true;
}


bool OcraControllerServerModule::interruptModule()
{
    if(ctrlThread)
        ctrlThread->suspend();
    return true;
}

bool OcraControllerServerModule::close()
{
    /* Stop the control thread. */
    if(ctrlThread){
        ctrlThread->stop();
    }

    /* Stop the WBI threads. */
    if(robotInterface){
        if(!robotInterface->close())
            printf("Error while closing robot interface\n");
    }

    /* Print performance information */
    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", controller_options.threadPeriod, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*controller_options.threadPeriod)
        printf("Next time you could set a lower period to improve the controller performance.\n");
    else if(avgTime>1.3*controller_options.threadPeriod)
        printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

    return true;
}

bool OcraControllerServerModule::updateModule()
{
    if (ctrlThread==0)
    {
        printf("ControlThread pointers are zero\n");
        return false;
    }

    ctrlThread->getEstPeriod(avgTime, stdDev);
    ctrlThread->getEstUsed(avgTimeUsed, stdDevUsed); // real duration of run()
    if(avgTimeUsed > 1.3 * controller_options.threadPeriod)
    {
        printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, controller_options.threadPeriod);
        printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }

    return true;
}

void OcraControllerServerModule::printHelp()
{
    std::cout<< "Possible parameters" << std::endl << std::endl;
    std::cout<< "\t--context :Where to find an user defined .ini file e.g. /locomotionCtrl" <<std::endl;
    std::cout<< "\t--from :Name of the file.ini to be used for configuration." <<std::endl;
    std::cout<< "\t--rate :Period used by the module. Default set to 10ms." <<std::endl;
    std::cout<< "\t--robot :Robot name (icubSim or icub). Set to icub by default." <<std::endl;
    std::cout<< "\t--local :Prefix of the ports opened by the module. Set to the module name by default, i.e. basicWholeBodyInterfaceModule." <<std::endl;
    std::cout<< "\t--taskSet :A path to an XML file containing a set of tasks. The tasks will be created when the controller is started. Set to empty by default." <<std::endl;
    std::cout<< "\t--sequence :A string identifying a predefined scenario. The scenarios (sets of tasks and control logic) are defined in sequenceCollection and will be created when the controller is started. Set to empty by default." <<std::endl;
    std::cout<< "\t--debug :If this flag is present then the controller will run in Debug mode which allows each joint to be tested individually." <<std::endl;
    std::cout<< "\t--floatingBase :If this flag is present then the controller will run in using a floating base dynamic model and control. Defaults to false, or fixed base if no flag is present." <<std::endl;
}