/*
 *  \file       main.cpp
 *  \brief      A module that makes iCub walk based on mixed-integer model predictive control.
 *  \details
 *  \Author     [Jorhabib Eljaik](http://github.com/jeljaik)
 *  \date       25 November 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of walking-client.
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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/ControllerClient.h>
#include <ocra-recipes/ClientManager.h>

#include "walking-client/WalkingClient.h"

int main (int argc, char * argv[])
{
    yarp::os::Log yLog;
    yarp::os::Network yarp;

    double network_timeout = 10.0;
    if (!yarp.checkNetwork(network_timeout))
    {
        yLog.fatal() << "YARP network is not available";
        return -1;
    }

    yLog.info() << "Resource finder stuff";
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("walking-client.ini"); //default config file name.
    rf.setDefaultContext("walking-client"); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    int loopPeriod;
    if (!rf.check("period")) {
        OCRA_WARNING("Option 'period' [ms] was not specified. Using default 10ms ");
        loopPeriod = 10;
    } else {
        loopPeriod = rf.find("period").asInt();
    }
    
    yLog.info() << "Making model initializer";
    ocra_icub::ModelInitializer modelIni = ocra_icub::ModelInitializer();

    std::shared_ptr<ocra_recipes::ControllerClient> ctrlClient;
    yLog.info() << "Making controller client";

    if(!modelIni.getModel())
    {
        yLog.fatal() << "Model is not empty.";
    }

    ctrlClient = std::make_shared<WalkingClient>(modelIni.getModel(), loopPeriod);

    std::shared_ptr<ocra_recipes::ClientManager> clientManager;
    yLog.info() << "Making client manager";
    clientManager = std::make_shared<ocra_recipes::ClientManager>(ctrlClient);


    if (rf.check("help"))
    {
        OCRA_INFO("User requested help...");
        clientManager->printHelp();
        return 0;
    }

    yLog.info() << "Configuring";
    OCRA_WARNING("rf.MIQP_PARAMS.wu = " << rf.find("wu").asDouble());
    clientManager->configure(rf);

    yLog.info() << "Launching client";
    return clientManager->launchClient();
}
