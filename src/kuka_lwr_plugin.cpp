/*
 Copyright (c) 2018, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 Author: Dawid Seredynski
*/

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelKukaLwr : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model_ = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelKukaLwr::OnUpdate, this));
        std::cout << "plugin loaded" << std::endl;

        const std::string name = "lwr";
        for (int i = 0; i < 7; ++i) {
            std::string joint_name = std::string("lwr::") + name + "_arm_" + std::to_string(i) + "_joint";
            joints_.push_back(model_->GetJoint(joint_name));
        }

        for (int i = 0; i < 7; ++i) {
            std::string link_name = std::string("lwr::") + name + "_arm_" + std::to_string(i+1) + "_link";
            links_.push_back(model_->GetLink(link_name));
        }

    }

    void setForces(const std::array<double, 7 > &t) {
        for (int i=0; i<joints_.size(); i++) {
            joints_[i]->SetForce(0, t[i]);
        }
    }

    void getGravComp(std::array<double, 7 > &t) {
        ignition::math::Vector3d gr = gazebo::physics::get_world()->Gravity();

        ignition::math::Vector3d tool_com;
        double tool_mass = 0;

        gazebo::physics::LinkPtr link = links_[6];
        gazebo::physics::JointPtr joint = joints_[6];
        ignition::math::Pose3d T_W_L7 = link->WorldPose();
        ignition::math::Vector3d cog = T_W_L7.CoordPositionAdd( tool_com );
        ignition::math::Vector3d r = cog - joint->WorldPose().Pos();
        double mass = tool_mass;
        ignition::math::Vector3d torque = r.Cross(mass * gr);
        ignition::math::Vector3d axis = joint->GlobalAxis(0);
        t[6] = axis.Dot(torque);

        for (int i = 6; i > 0; i--) {
            link = links_[i-1];
            joint = joints_[i-1];
            cog = (cog * mass + link->WorldCoGPose().Pos() * link->GetInertial()->Mass()) / (mass+link->GetInertial()->Mass());
            mass += link->GetInertial()->Mass();
            r = cog - joint->WorldPose().Pos();
            torque = r.Cross(mass * gr);
            axis = joint->GlobalAxis(0);
            t[i-1] = axis.Dot(torque);
        }

        for (int i = 0; i < 7; ++i) {
            t[i] = -t[i];
        }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
        std::array<double, 7 > t;
        t.fill(0);      // initialize all torques to 0

        // get gravity compensation torques
        getGravComp(t);

        // declare equilibrium point
        std::array<double, 7 > eq({0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4});

        // calculate spring forces
        double k = 10.0;    // stiffness constant
        for (int i = 0; i < t.size(); ++i) {
            double diff = eq[i] - joints_[i]->Position(0);
            t[i] += k * diff;
        }

        // apply torques
        setForces(t);
    }

    // Pointer to the model
    private: physics::ModelPtr model_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private:

    std::vector<gazebo::physics::JointPtr > joints_;
    std::vector<gazebo::physics::LinkPtr > links_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelKukaLwr)
}
