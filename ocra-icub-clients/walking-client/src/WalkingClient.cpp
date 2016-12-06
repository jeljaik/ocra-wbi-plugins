#include "walking-client/WalkingClient.h"

using namespace Eigen;

WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod),
_zmpParams(std::make_shared<ZmpControllerParams>(2e-3, model->getMass(), (model->getCoMPosition()).operator()(2), 9.8) ),
_zmpController(std::make_shared<ZmpController>(loopPeriod, modelPtr, _zmpParams)),
_rawLeftFootWrench(Eigen::VectorXd::Zero(6)),
_rawRightFootWrench(Eigen::VectorXd::Zero(6)),
_globalZMP(Eigen::Vector2d::Zero()),
_isTestRun(false)
{

}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

bool WalkingClient::initialize()
{
    //TODO: Remember to remove the hardcorded /walkingClient name
    // Connect to feet wrench ports
    bool ok = portWrenchLeftFoot.open("/walkingClient/left_foot/wrench:i");
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/left_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        if (!yarp::os::Network::connect("/icubGazeboSim/left_foot/analog:o", portWrenchLeftFoot.getName().c_str())) {
            OCRA_ERROR("Impossible to connect to /icubGazeboSim/left_foot/analog:o");
            return false;
        }
    }
    ok = portWrenchRightFoot.open("/walkingClient/right_foot/wrench:i");
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/right_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        if (!yarp::os::Network::connect("/icubGazeboSim/right_foot/analog:o", portWrenchRightFoot.getName().c_str()) ) {
            OCRA_ERROR("Impossible to connect to /icubGazeboSim/right_foot/analog:o");
            return false;
        }
    }
    
    
    // CoM TaskConnection object. This is the task object through which the CoM velocity will be set by this controller.
    std::string comTaskName("ComTask");
    _comTask = std::make_shared<ocra_recipes::TaskConnection>(comTaskName);
    _comTask->openControlPorts();
    
    
    // For testing and tuning purposes, generate ZMP trajectory that moves from one foot to the other.
    Eigen::Vector3d sep; sep.setZero();
    getFeetSeparation(sep);
    double feetSeparation = sep(1);
    int N = 4;
    int period = this->getExpectedPeriod();
    double tTrans = 4;
    _zmpTrajectory = generateZMPTrajectoryTEST(tTrans, feetSeparation, period, N);
    
    // Is the controller undergoing a test run?
    _isTestRun = true;
    
    
    // If ZMP tests are being run, publish ZMP on port
    if (_isTestRun) {
        _zmpPort.open("/walkingClient/zmpError:o");
        _dcomErrorPort.open("/walkingClient/dcomError:o");
        _dComDesPort.open("/walkingClient/dComDesired:o");
        _dComCurPort.open("/walkingClient/dComCurrent:o");
        _zmpDesPort.open("/walkingClient/zmpDesired:o");
        _zmpCurPort.open("/walkingClient/zmpCurrent:o");
        _comCurrent.open("/walkingClient/comCurrent:o");
        _ddcomCurrent.open("/walkingClient/ddcomCurrent:o");
        _ddcomFromZMP.open("/walkingClient/ddcomFromZMP:o");
    }
    
    Eigen::Vector3d initialCOMPosition = this->model->getCoMPosition();
    _zmpParams->cz = initialCOMPosition(2);
    std::cout << "ZMPController object will run with the following parameters: " << std::endl;
    std::cout << "Gain k_f: " << _zmpParams->kf << std::endl;
    std::cout << "Total mass of the robot: " << _zmpParams->m << std::endl;
    std::cout << "Constant height of the COM: " << _zmpParams->cz << std::endl;
    std::cout << "Gravity acceleration: " << _zmpParams->g << std::endl;
    
    return true;
}

void WalkingClient::release()
{
    
}

void WalkingClient::loop()
{
    static int el = 0;
    // Read measurements
    readFootWrench(LEFT_FOOT, _rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, _rawRightFootWrench);
    
    
    // Compute ZMP
    _zmpController->computeGlobalZMPFromSensors(_rawLeftFootWrench, _rawRightFootWrench, _globalZMP);
    
    
    // Testing
    Eigen::Vector2d dhd; dhd.setZero();
    //TODO: Write _zmpTrajectory to port
    if (_isTestRun) {
        _zmpController->computehd(_globalZMP, _zmpTrajectory[el], dhd);
//        Eigen::Vector2d error = _globalZMP - _zmpTrajectory.rbegin()[el];
//        publishZMPError(error);
        if ( el < _zmpTrajectory.size() ){
            el++;
        }
    }
    
    Eigen::Vector3d zeroAngVel = Eigen::Vector3d::Zero();
//    double yRefVel = 0.01;
//    Eigen::Vector3d refLinVel( 0, yRefVel, 0 );
//    Eigen::Vector3d refLinVel( dhd(0), dhd(1), 0 );
    Eigen::Vector3d refLinVel( 0, dhd(1), 0 );
    Eigen::Twistd refComVelocity( zeroAngVel, refLinVel );
    
    // Apply CONTROL
    _desiredComState.setVelocity(refComVelocity);
    _comTask->setDesiredTaskStateDirect(_desiredComState);
    
    Eigen::Vector3d currentComLinVel;
    currentComLinVel = _comTask->getTaskState().getVelocity().getLinearVelocity();
//    Eigen::Vector2d error = currentComLinVel.topRows(2) - refLinVel.topRows(2);
//    publishCOMError( error );
    
    Eigen::Vector3d currentComPos;
    currentComPos = _comTask->getTaskState().getPosition().getTranslation();
    
    Eigen::Vector3d currentddcom;
    currentddcom = _comTask->getTaskState().getAcceleration().getLinearVelocity();
    
    Eigen::Vector3d ddcomFromZMP;
    ddcomFromZMP = (_zmpParams->g/_zmpParams->cz)*(currentComPos.topRows(2) - _globalZMP);

    if (_isTestRun) {
        publish3dQuantity(_dComDesPort, refLinVel);
        publish3dQuantity(_dComCurPort, currentComLinVel);
        publish3dQuantity(_zmpDesPort, (Eigen::Vector3d() << _zmpTrajectory[el-1], 0).finished());
        publish3dQuantity(_zmpCurPort, (Eigen::Vector3d() << _globalZMP, 0).finished());
        publish3dQuantity(_comCurrent, currentComPos);
        publish3dQuantity(_ddcomCurrent, currentddcom);
        publish3dQuantity(_ddcomFromZMP, (Eigen::Vector3d() << ddcomFromZMP, 0).finished());
    }
    
    // Compute ZMPPreviewController optimal input
    
}

bool WalkingClient::readFootWrench(FOOT whichFoot, Eigen::VectorXd &rawWrench) {
    yarp::sig::Vector * yRawFootWrench;
    switch (whichFoot) {
        case LEFT_FOOT:
            yRawFootWrench = portWrenchLeftFoot.read();
            break;
        case RIGHT_FOOT:
            yRawFootWrench = portWrenchRightFoot.read();
            break;
        default:
            break;
    }
    
    if (yRawFootWrench == NULL)
        return false;
    
    rawWrench = Eigen::VectorXd::Map(yRawFootWrench->data(), 6);
    return true;
}

std::vector<Eigen::Vector2d> WalkingClient::generateZMPTrajectoryTEST(double tTrans,
                                                                      double feetSeparation,
                                                                      double timeStep,
                                                                      int    N)
{
    std::vector<Eigen::Vector2d> zmpTrajectory;
    Eigen::Vector2d sineVector = Eigen::Vector2d::Zero();
    double t = 0;
    double tmp;
    while (t < N*tTrans) {
        tmp = (-feetSeparation/3)*std::sin((M_PI/(2*tTrans))*t) - feetSeparation/2;
        sineVector(1) = tmp;
        zmpTrajectory.push_back(sineVector);
        t = t + timeStep/1000;
    }
    
    return zmpTrajectory;
}

bool WalkingClient::getFeetSeparation(Eigen::Vector3d &sep) {
    Eigen::Vector3d lFootPosition = this->model->getSegmentPosition("l_foot").getTranslation();
    Eigen::Vector3d rFootPosition = this->model->getSegmentPosition("r_foot").getTranslation();
    sep = (rFootPosition - lFootPosition).cwiseAbs();
    return true;
}

bool WalkingClient::publishZMPError(Eigen::Vector2d &zmpError) {
    yarp::os::Bottle &output = _zmpPort.prepare();
    output.clear();
    output.addDouble(zmpError(0));
    output.addDouble(zmpError(1));
    _zmpPort.write();
    return true;
}

bool WalkingClient::publishCOMError(Eigen::Vector2d &dcomError) {
    yarp::os::Bottle &output = _dcomErrorPort.prepare();
    output.clear();
    output.addDouble(dcomError(0));
    output.addDouble(dcomError(1));
    _dcomErrorPort.write();
    return true;
}

bool WalkingClient::publish3dQuantity(yarp::os::BufferedPort<yarp::os::Bottle> &port, Eigen::Vector3d &value) {
    yarp::os::Bottle &output = port.prepare();
    output.clear();
    output.addDouble(value(0));
    output.addDouble(value(1));
    output.addDouble(value(2));
    port.write();
    return true;
}