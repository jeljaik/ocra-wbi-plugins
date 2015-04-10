#include "ISIRWholeBodyController/ScenariosICub.h"
#include <cmath>

#ifndef PI
#define PI 3.1415926

#endif

ScenarioICub_01_Standing::ScenarioICub_01_Standing() : orcisir::ISIRTaskManagerCollectionBase()
{
}

ScenarioICub_01_Standing::~ScenarioICub_01_Standing()
{
}

void ScenarioICub_01_Standing::doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model)
{
    // Initialise full posture task
    // Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
    Eigen::VectorXd q_full = model.getJointPositions();
    
    std::cout << "\n\n q_full = \n"<< q_full <<" \n\n";    

    // q_full[model.getDofIndex("l_elbow")]      = PI/8.0;
    // q_full[model.getDofIndex("r_elbow")]      = PI/8.0;
    // q_full[model.getDofIndex("l_knee")]             = -0.05;
    // q_full[model.getDofIndex("r_knee")]             = -0.05;
    // q_full[model.getDofIndex("l_ankle_pitch")]      = -0.05;
    // q_full[model.getDofIndex("r_ankle_pitch")]      = -0.05;
    // q_full[model.getDofIndex("l_shoulder_roll")]    = PI/8.0;
    // q_full[model.getDofIndex("r_shoulder_roll")]    = PI/8.0;
    
    std::cout << "\n\n q_full = \n"<< q_full <<" \n\n";
    
    std::cout << "\n\n tmFull \n\n";
    taskManagers["tmFull"] = new orcisir::ISIRFullPostureTaskManager(ctrl, model, "fullPostureTask", orc::FullState::INTERNAL, 10.0, 2*sqrt(10.0), 0.0001, q_full);

    // Initialise waist pose
    
    // std::cout << "\n\n tmSegPoseWaist \n\n";
    // Eigen::Displacementd desiredWaistPose = Eigen::Displacementd(0.0,0.0,0.58,1.0,0.0,0.0,0.0);
    // taskManagers["tmSegPoseWaist"] = new orcisir::ISIRSegPoseTaskManager(ctrl, model, "waistPoseTask", "root_link", orc::XYZ, 36.0, 2*sqrt(36.0), 1.0, desiredWaistPose);
    
    // Initialise partial posture task

    
    // Eigen::VectorXi sdofs(3);
    // sdofs << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");
    // Eigen::VectorXd zero = Eigen::VectorXd::Zero(3);
    // std::cout << "\n\n tmPartialBack \n\n";
    // taskManagers["tmPartialBack"] = new orcisir::ISIRPartialPostureTaskManager(ctrl, model, "partialPostureBackTask", orc::FullState::INTERNAL, sdofs, 16.0, 2*sqrt(16.0), 0.001, zero);

    // double mu_sys = 0.5;
    // double margin = 0.0;
    // double sqrt2on2 = sqrt(2.0)/2.0;
    // Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, -sqrt2on2, 0.0) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
    // Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(0.0, sqrt2on2, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
    


    // // Initialise left foot contacts
    // Eigen::Displacementd LFContacts[4];
    // LFContacts[0] = Eigen::Displacementd(Eigen::Vector3d(-.039,-.027,-.031), rotLZdown);
    // LFContacts[1] = Eigen::Displacementd(Eigen::Vector3d(-.039, .027,-.031), rotLZdown);
    // LFContacts[2] = Eigen::Displacementd(Eigen::Vector3d(-.039, .027, .099), rotLZdown);
    // LFContacts[3] = Eigen::Displacementd(Eigen::Vector3d(-.039,-.027, .099), rotLZdown);
    // std::cout << "\n\n tmFootContactLeft \n\n";
    // taskManagers["tmFootContactLeft"] = new orcisir::ISIRContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_foot", LFContacts, 4, mu_sys, margin);

    // // Initailise right foot contacts
    // Eigen::Displacementd RFContacts[4];
    // RFContacts[0] = Eigen::Displacementd(Eigen::Vector3d(-.039,-.027, .031), rotRZdown);
    // RFContacts[1] = Eigen::Displacementd(Eigen::Vector3d(-.039, .027, .031), rotRZdown);
    // RFContacts[2] = Eigen::Displacementd(Eigen::Vector3d(-.039, .027,-.099), rotRZdown);
    // RFContacts[3] = Eigen::Displacementd(Eigen::Vector3d(-.039,-.027,-.099), rotRZdown);
    // std::cout << "\n\n tmFootContactRight \n\n";
    // taskManagers["tmFootContactRight"] = new orcisir::ISIRContactSetTaskManager(ctrl, model, "RightFootContactTask", "r_foot", RFContacts, 4, mu_sys, margin);
    std::cout << "\n\n Fully Initialized \n\n";
    
}

void ScenarioICub_01_Standing::doUpdate(double time, orcisir::ISIRModel& state, void** args)
{
}
