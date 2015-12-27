#ifndef CONTROLTHREADBASE_H
#define CONTROLTHREADBASE_H value

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>

#include <string>
#include <sstream>
#include <iostream>

class controlThreadBase: public yarp::os::RateThread
{

public:
    // Constructor
    controlThreadBase(int period, const std::string& taskRpcPortName);
    ~controlThreadBase();

    static int threadId;


    // RateThread virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    // controlThreadBase pure virtual functions
    virtual bool ct_threadInit()=0;
    virtual void ct_threadRelease()=0;
    virtual void ct_run()=0;

    // controlThreadBase functions
    std::string getThreadType(){return controlThreadType;}

protected:

    void setThreadType(const std::string& _threadType = "controlThreadBase"){controlThreadType = _threadType;}

    std::string controlThreadType;

    // Yarp control ports
    std::string inputPortName, outputPortName;
    yarp::os::Port inputPort, outputPort;

    //Yarp RPC client
    std::string taskRpcServerName, threadRpcClientName;
    yarp::os::RpcClient threadRpcClient;

    //Yarp network
    yarp::os::Network yarp;

    bool openControlPorts();
    bool connectControlPorts();

};

#endif
