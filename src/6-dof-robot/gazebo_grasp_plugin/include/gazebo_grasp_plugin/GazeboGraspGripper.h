#ifndef GAZEBO_GAZEBOGRASPGRIPPER_H
#define GAZEBO_GAZEBOGRASPGRIPPER_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>
#include <vector>
#include <map>
#include <string>

namespace gazebo
{

/**
 * \brief Helper class for GazeboGraspFix which holds information for one arm.
 * Attaches /detaches objects to the palm of this arm.
 */
class GazeboGraspGripper
{
  public:
    GazeboGraspGripper();
    GazeboGraspGripper(const GazeboGraspGripper &o);
    virtual ~GazeboGraspGripper();

    bool Init(physics::ModelPtr &_model,
              const std::string &_gripperName,
              const std::string &palmLinkName,
              const std::vector<std::string> &fingerLinkNames,
              bool _disableCollisionsOnAttach,
              std::map<std::string, physics::CollisionPtr> &_collisions);

    const std::string &getGripperName() const;
    bool hasLink(const std::string &linkName) const;
    bool hasCollisionLink(const std::string &linkName) const;
    bool isObjectAttached() const;
    const std::string &attachedObject() const;

    bool HandleAttach(const std::string &objName);
    void HandleDetach(const std::string &objName);

  private:
    physics::ModelPtr model;
    std::string gripperName;
    std::vector<std::string> linkNames;
    std::map<std::string, physics::CollisionPtr> collisionElems;
    physics::JointPtr fixedJoint;
    physics::LinkPtr palmLink;
    bool disableCollisionsOnAttach;
    bool attached;
    std::string attachedObjName;
};

}
#endif
