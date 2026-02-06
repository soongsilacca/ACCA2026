#include <gazebo_grasp_plugin/GazeboGraspGripper.h>
#include <gazebo_version_helpers/GazeboVersionHelpers.h>
#include <iostream>

using namespace gazebo;

GazeboGraspGripper::GazeboGraspGripper(): attached(false) {}

GazeboGraspGripper::GazeboGraspGripper(const GazeboGraspGripper &o):
  model(o.model),
  gripperName(o.gripperName),
  linkNames(o.linkNames),
  collisionElems(o.collisionElems),
  fixedJoint(o.fixedJoint),
  palmLink(o.palmLink),
  disableCollisionsOnAttach(o.disableCollisionsOnAttach),
  attached(o.attached),
  attachedObjName(o.attachedObjName)
{}

GazeboGraspGripper::~GazeboGraspGripper()
{
  this->model.reset();
}

bool GazeboGraspGripper::Init(physics::ModelPtr &_model,
                              const std::string &_gripperName,
                              const std::string &palmLinkName,
                              const std::vector<std::string> &fingerLinkNames,
                              bool _disableCollisionsOnAttach,
                              std::map<std::string, physics::CollisionPtr> &_collisionElems)
{
  this->gripperName = _gripperName;
  this->attached = false;
  this->disableCollisionsOnAttach = _disableCollisionsOnAttach;
  this->model = _model;
  physics::PhysicsEnginePtr physics = GetPhysics(this->model->GetWorld());
  this->fixedJoint = physics->CreateJoint("revolute");

  this->palmLink = this->model->GetLink(palmLinkName);
  if (!this->palmLink)
  {
    gzerr << "GazeboGraspGripper: Palm link " << palmLinkName << " not found." << std::endl;
    return false;
  }

  for (const std::string &linkName : fingerLinkNames)
  {
    physics::LinkPtr link = this->model->GetLink(linkName);
    if (!link)
    {
      gzerr << "GazeboGraspGripper ERROR: Link " << linkName << " can't be found." << std::endl;
      continue;
    }
    this->linkNames.push_back(linkName);
    for (unsigned int j = 0; j < link->GetChildCount(); ++j)
    {
      physics::CollisionPtr collision = link->GetCollision(j);
      if (!collision) continue; // Should not happen if index is valid
      std::string collName = collision->GetScopedName();
      if (this->collisionElems.find(collName) != this->collisionElems.end())
      {
        gzwarn << "GazeboGraspGripper: Adding collision " << collName << " multiple times." << std::endl;
        continue;
      }
      this->collisionElems[collName] = collision;
      _collisionElems[collName] = collision;
    }
  }
  return !this->collisionElems.empty();
}

const std::string &GazeboGraspGripper::getGripperName() const
{
  return gripperName;
}

bool GazeboGraspGripper::hasLink(const std::string &linkName) const
{
  for (const std::string &name : linkNames)
  {
    if (name == linkName) return true;
  }
  return false;
}

bool GazeboGraspGripper::hasCollisionLink(const std::string &linkName) const
{
  return collisionElems.find(linkName) != collisionElems.end();
}

bool GazeboGraspGripper::isObjectAttached() const
{
  return attached;
}

const std::string &GazeboGraspGripper::attachedObject() const
{
  return attachedObjName;
}

bool GazeboGraspGripper::HandleAttach(const std::string &objName)
{
  if (!this->palmLink) return false;
  
  physics::WorldPtr world = this->model->GetWorld();
  physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(GetEntityByName(world, objName));
  
  if (!obj) {
      // Try getting as model if collision not found (fallback, though usually it interacts with collisions)
      physics::ModelPtr modelObj = world->ModelByName(objName);
      if (modelObj && modelObj->GetLink()) {
          // If wefound a model, grab its first link's first collision? Or just attach to link?
           // The original code attaches to collision's link.
           // Let's assume objName is a scoped collision name as provided by ContactManager
      } 
      if (!obj)
      {
        // One retry: maybe objName is the model name, and we should attach to its canonical link?
        // But the grasp plugin works on collisions.
        std::cerr << "ERROR: Object " << objName << " not found in world, cannot attach." << std::endl;
        return false;
      }
  }

  // Calculate pose difference (Obj relative to Palm)
  ignition::math::Pose3d diff = this->palmLink->WorldPose().Inverse() * obj->GetLink()->WorldPose();
  
  this->fixedJoint->Load(this->palmLink, obj->GetLink(), diff);
  this->fixedJoint->Init();
  this->fixedJoint->SetUpperLimit(0, 0);
  this->fixedJoint->SetLowerLimit(0, 0);

  if (this->disableCollisionsOnAttach)
  {
    obj->GetLink()->SetCollideMode("none");
  }

  this->attached = true;
  this->attachedObjName = objName;
  return true;
}

void GazeboGraspGripper::HandleDetach(const std::string &objName)
{
  physics::WorldPtr world = this->model->GetWorld();
  if (!world) return;
  
  // Just detach, we don't strictly need to find the object if we just detach the joint, 
  // but we might want to restore collision mode.
  this->fixedJoint->Detach();

  if (this->disableCollisionsOnAttach && !objName.empty())
  {
     physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(GetEntityByName(world, objName));
     if (obj) {
         obj->GetLink()->SetCollideMode("all");
     }
  }

  this->attached = false;
  this->attachedObjName = "";
}
