#ifndef GAZEBO_VERSION_HELPERS_H
#define GAZEBO_VERSION_HELPERS_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    // Simple helpers for Gazebo 11 compatibility (assuming ROS 2 Humble)
    
    inline physics::PhysicsEnginePtr GetPhysics(physics::WorldPtr world)
    {
        return world->Physics();
    }

    inline physics::EntityPtr GetEntityByName(physics::WorldPtr world, const std::string &name)
    {
        return world->EntityByName(name);
    }

    inline ignition::math::Vector3d GetPos(const ignition::math::Pose3d &pose)
    {
        return pose.Pos();
    }
    
    // Gazebo 9/11 usually use ignition math, but some APIs still return gazebo::math or Ignite math.
    // For Gazebo 11, Link::GetWorldPose returns python-wrapped method result or ign-math.
    // Let's rely on standard API usage seen in Gazebo 11 examples.
}

#endif
