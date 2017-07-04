/*! \file       IcubControllerServer.h
 *  \brief      Module class implementation for the iCub controller server.
 *  \class      IcubControllerServer
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-recipes.
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

#ifndef ICUB_CONTROLLER_SERVER_H
#define ICUB_CONTROLLER_SERVER_H

#include <wbi/wbi.h>
#include <ocra-recipes/ControllerServer.h>
#include <Eigen/Dense>
#include <ocra-icub/OcraWbiModel.h>
#include <iDynTree/Estimation/SimpleLeggedOdometry.h>
#include <ocra/util/ErrorsHelper.h>
#include <ocra/util/KDLUtilities.h>

class IcubControllerServer : public ocra_recipes::ControllerServer
{
public:
    IcubControllerServer(   std::shared_ptr<wbi::wholeBodyInterface> robot,
                            std::string icubName,
                            const bool usingFloatingBase,
                            const ocra_recipes::CONTROLLER_TYPE ctrlType=ocra_recipes::WOCRA_CONTROLLER,
                            const ocra_recipes::SOLVER_TYPE solver=ocra_recipes::QUADPROG,
                            const bool usingInterprocessCommunication=true,
                            const bool useOdometry=false
                        );
    
    virtual ~IcubControllerServer();

    virtual ocra::Model::Ptr loadRobotModel();

    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root);

    virtual void getRobotStateKDL(Eigen::VectorXd& q, Eigen::VectorXd& qd, KDL::Frame& H_root, KDL::Twist& T_root);

    /**
     Performs initialization steps for leg odometry such as: 
     - Retrieve iCub's cannonical joints. 
     

     @param model_file Full path to URDF model file.
     @param initialFixedFrame Valid name of a frame that is initially fixed to the ground.
     @return True when initialization is successful. False otherwise.
     */
    bool initializeOdometry(std::string model_file, std::string initialFixedFrame);
    
    /**
     The URDF file has more joints than those used by the yarpWholeBodyInterface, and these two should match. Therefore, the following method creates a list of joints as those that constitute ROBOT_MAIN_JOINTS in yarpWholeBodyInterface.ini.

     @return Vector of cannonical joints.
     */
    std::vector<std::string> getCanonical_iCubJoints();

    /**
     Computes iCub's root frame velocity. This method is not inherited from the base class.

     @param q Current internal joint configuration.
     @param qd Current internal joint velocities.
     @param wbi_H_root_Transform Roto-translation matrix of iCub's root reference frame.
     @param regularization Regularization term for the computation of the root velocity.
     @param LEFT_FOOT_CONTACT 1 if the left foot is in contact with the ground, 0 otherwise.
     @param RIGHT_FOOT_CONTACT 1 if the right foot is in contact with the ground, 0 otherwise.
     @param[out] twist Reference to variable storing the resulting root frame velocity.
     */
    void rootFrameVelocity(Eigen::VectorXd& q,
                           Eigen::VectorXd& qd,
                           iDynTree::Transform& wbi_H_root_Transform,
                           double           regularization,
                           int              LEFT_FOOT_CONTACT,
                           int              RIGHT_FOOT_CONTACT,
                           Eigen::VectorXd& twist);

    void rootFrameVelocityPivLU(Eigen::VectorXd& q,
                                Eigen::VectorXd& qd,
                                iDynTree::Transform& wbi_H_root_Transform,
                                Eigen::VectorXd& twist);

    void rootFrameVelocityPivLU(Eigen::VectorXd& q,
                                Eigen::VectorXd& qd,
                                iDynTree::Transform& wbi_H_root_Transform,
                                int              LEFT_FOOT_CONTACT,
                                int              RIGHT_FOOT_CONTACT,
                                Eigen::VectorXd& twist);

    void pinv(Eigen::MatrixXd mat, Eigen::MatrixXd& pinvmat, double pinvtoler=1.0e-6) const;

    void velocityError(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd X);
private:
    std::shared_ptr<wbi::wholeBodyInterface> wbi; /*!< The WBI used to talk to the robot. */
    std::string robotName;
    bool isFloatingBase;
    bool useOdometry;
    static constexpr double ALL_JOINTS = -1.0;
    int nDoF;

    Eigen::VectorXd wbi_H_root_Vector;
    Eigen::VectorXd wbi_T_root_Vector;
    wbi::Frame wbi_H_root;

    iDynTree::SimpleLeggedOdometry odometry;

};

#endif
