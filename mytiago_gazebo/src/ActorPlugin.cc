/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "hrsi_gazebo/ActorPlugin.hh"

using namespace gazebo ;
using namespace std ;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"
#define PI 3.1416
#define CoM_Z 1.1

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem = _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Read in target list
  if (_sdf->HasElement("target_list"))
  {
    this->loop_traj = _sdf->GetElement("target_list")->Get<bool>("loop_traj");
    sdf::ElementPtr targetElem = _sdf->GetElement("target_list")->GetElement("target");
    while (targetElem)
    {
      ignition::math::Vector3d t = targetElem->Get<ignition::math::Vector3d>();
      t[2] = CoM_Z;
      this->target_list.push_back(t);
      targetElem = targetElem->GetNextElement("target");
    }
  }  

  // Read in the min_max X-coord
  if (_sdf->HasElement("min_max_X"))
  {
    ignition::math::Vector3d minmax_x = _sdf->Get<ignition::math::Vector3d>("min_max_X");
    this->min_x = minmax_x[0];
    this->max_x = minmax_x[1];
  }
  else
  {
    this->min_x = -15.0;
    this->max_x = 15.0;
  }

  // Read in the min_max y-coord
  if (_sdf->HasElement("min_max_Y"))
  {
    ignition::math::Vector3d minmax_y = _sdf->Get<ignition::math::Vector3d>("min_max_Y");
    this->min_y = minmax_y[0];
    this->max_y = minmax_y[1];
  }
  else
  {
    this->min_y = -25.0;
    this->max_y = 25.0;
  }

  this->Reset();

}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;
  this->target = this->target_list[0];

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{
  int index_current_target = this->find_index();
  int index_next_target = index_current_target;
  if (index_current_target == this->target_list.size() - 1 && this->loop_traj)
  {
    index_next_target = 0;
  }
  else if (index_current_target < this->target_list.size() - 1)
  {
    index_next_target = index_current_target + 1;
  }
  this->target = this->target_list[index_next_target];
}

/////////////////////////////////////////////////
int ActorPlugin::find_index()
{
  auto itr = find(this->target_list.begin(), this->target_list.end(), this->target);
  return distance(this->target_list.begin(), itr);
}

/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + PI/2 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(PI/2, 0, rpy.Z() + yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(PI/2, 0, rpy.Z() + yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(this->min_x, std::min(this->max_x, pose.Pos().X())));
  pose.Pos().Y(std::max(this->min_y, std::min(this->max_y, pose.Pos().Y())));
  pose.Pos().Z(CoM_Z);

  // Distance traveled is used to coordinate motion with the walking animation
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
