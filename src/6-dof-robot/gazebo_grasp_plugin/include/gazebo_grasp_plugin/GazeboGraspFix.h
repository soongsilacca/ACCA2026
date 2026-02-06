#ifndef GAZEBO_GAZEBOGRASPFIX_H
#define GAZEBO_GAZEBOGRASPFIX_H

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>
#include <gazebo_grasp_plugin/GazeboGraspGripper.h>

namespace gazebo
{

class GazeboGraspFix : public ModelPlugin
{
  public:
    GazeboGraspFix();
    GazeboGraspFix(physics::ModelPtr _model);
    virtual ~GazeboGraspFix();
  
  private:
    void OnAttach(const std::string &objectName, const std::string &armName);
    void OnDetach(const std::string &objectName, const std::string &armName);

    virtual void Init();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate();
    void InitValues();
    void OnContact(const ConstContactsPtr &ptr);

    bool IsGripperLink(const std::string &linkName, std::string &gripperName) const;
    std::map<std::string, std::string> GetAttachedObjects() const;

    class ObjectContactInfo;
    bool ObjectAttachedToGripper(const ObjectContactInfo &objContInfo, std::string &attachedToGripper) const;
    bool ObjectAttachedToGripper(const std::string &gripperName, std::string &attachedToGripper) const;

    physics::WorldPtr world;
    std::map<std::string, GazeboGraspGripper> grippers;
    event::ConnectionPtr update_connection;
    transport::NodePtr node;
    transport::SubscriberPtr contactSub;

    float forcesAngleTolerance;
    bool disableCollisionsOnAttach;
    std::map<std::string, std::string> collisions;

    class CollidingPoint;
    std::map<std::string, std::map<std::string, CollidingPoint> > contacts;
    boost::mutex mutexContacts;

    std::map<std::string, std::map<std::string, CollidingPoint> > attachGripContacts;
    std::map<std::string, int> gripCounts;
    int maxGripCount;
    int gripCountThreshold;
    float releaseTolerance;
    common::Time updateRate;
    common::Time prevUpdateTime;
    std::string filter_name;
};

}

#endif
