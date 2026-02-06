#include <gazebo_grasp_plugin/GazeboGraspFix.h>
#include <gazebo_version_helpers/GazeboVersionHelpers.h>
#include <iostream>
#include <cmath>

using namespace gazebo;

#define DEFAULT_FORCES_ANGLE_TOLERANCE 120
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10
#define DEFAULT_RELEASE_TOLERANCE 0.005
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

GazeboGraspFix::GazeboGraspFix()
{
  InitValues();
}

GazeboGraspFix::GazeboGraspFix(physics::ModelPtr _model)
{
  InitValues();
}

GazeboGraspFix::~GazeboGraspFix()
{
  if (!filter_name.empty() && this->world)
  {
    physics::PhysicsEnginePtr physics = GetPhysics(this->world);
    physics::ContactManager *contactManager = physics->GetContactManager();
    if (contactManager)
        contactManager->RemoveFilter(filter_name);
  }
  this->update_connection.reset();
  if (this->node) this->node->Fini();
  this->node.reset();
}

void GazeboGraspFix::Init()
{
  this->prevUpdateTime = common::Time::GetWallTime();
}

void GazeboGraspFix::InitValues()
{
  this->prevUpdateTime = common::Time::GetWallTime();
  this->node = transport::NodePtr(new transport::Node());
}

void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  gzmsg << "Loading grasp-fix plugin" << std::endl;

  physics::ModelPtr model = _parent;
  this->world = model->GetWorld();

  if (!_sdf->HasElement("disable_collisions_on_attach")) {
    this->disableCollisionsOnAttach = DEFAULT_DISABLE_COLLISIONS_ON_ATTACH;
  } else {
    this->disableCollisionsOnAttach = _sdf->Get<bool>("disable_collisions_on_attach");
  }
  gzmsg << "GazeboGraspFix: disable_collisions_on_attach = " << (this->disableCollisionsOnAttach ? "true" : "false") << std::endl;

  if (!_sdf->HasElement("forces_angle_tolerance")) {
     this->forcesAngleTolerance = DEFAULT_FORCES_ANGLE_TOLERANCE * M_PI / 180;
  } else {
     this->forcesAngleTolerance = _sdf->Get<float>("forces_angle_tolerance") * M_PI / 180;
  }

  double _updateSecs;
  if (!_sdf->HasElement("update_rate")) {
    _updateSecs = 1.0 / DEFAULT_UPDATE_RATE;
  } else {
    int _rate = _sdf->Get<int>("update_rate");
    _updateSecs = 1.0 / _rate;
  }
  this->updateRate = common::Time(0, common::Time::SecToNano(_updateSecs));

  if (!_sdf->HasElement("max_grip_count")) {
    this->maxGripCount = DEFAULT_MAX_GRIP_COUNT;
  } else {
    this->maxGripCount = _sdf->Get<int>("max_grip_count");
  }

  if (!_sdf->HasElement("grip_count_threshold")) {
    this->gripCountThreshold = floor(this->maxGripCount / 2.0);
  } else {
    this->gripCountThreshold = _sdf->Get<int>("grip_count_threshold");
  }

  if (!_sdf->HasElement("release_tolerance")) {
     this->releaseTolerance = DEFAULT_RELEASE_TOLERANCE;
  } else {
     this->releaseTolerance = _sdf->Get<float>("release_tolerance");
  }

  // Load Arms
  sdf::ElementPtr armElem = _sdf->GetElement("arm");
  std::vector<std::string> collisionNames;
  
  if (!armElem)
  {
      gzerr << "GazeboGraspFix: No <arm> tag found." << std::endl;
      return;
  }

  while (armElem)
  {
      std::string armName = armElem->Get<std::string>("arm_name");
      std::string palmName = armElem->Get<std::string>("palm_link");
      
      std::vector<std::string> fingerLinkNames;
      if (armElem->HasElement("gripper_link")) {
          sdf::ElementPtr fingerElem = armElem->GetElement("gripper_link");
          while (fingerElem) {
              fingerLinkNames.push_back(fingerElem->Get<std::string>());
              fingerElem = fingerElem->GetNextElement("gripper_link");
          }
      }

      GazeboGraspGripper &gripper = grippers[armName];
      std::map<std::string, physics::CollisionPtr> _collisions;
      
      if (gripper.Init(model, armName, palmName, fingerLinkNames, disableCollisionsOnAttach, _collisions))
      {
           for (auto const& [name, val] : _collisions) {
               if (collisions.find(name) == collisions.end()) {
                   collisions[name] = armName;
                   collisionNames.push_back(name);
               }
           }
      }
      armElem = armElem->GetNextElement("arm");
  }

  if (grippers.empty()) {
      gzerr << "GazeboGraspFix: No valid arms configured." << std::endl;
      return;
  }

  // Init Transport
  this->node->Init(this->world->Name());
  physics::PhysicsEnginePtr physics = GetPhysics(this->world);
  physics::ContactManager *contactManager = physics->GetContactManager();
  
  filter_name = model->GetScopedName();
  std::string topic = contactManager->CreateFilter(filter_name, collisionNames);
  
  this->contactSub = this->node->Subscribe(topic, &GazeboGraspFix::OnContact, this);

  this->update_connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GazeboGraspFix::OnUpdate, this));
}

class GazeboGraspFix::CollidingPoint {
public:
    CollidingPoint() : sum(0) {}
    std::string gripperName;
    physics::CollisionPtr collLink, collObj;
    ignition::math::Vector3d force;
    ignition::math::Vector3d pos;
    ignition::math::Vector3d objPos;
    int sum;
};

class GazeboGraspFix::ObjectContactInfo {
public:
    std::vector<ignition::math::Vector3d> appliedForces;
    std::map<std::string, int> grippersInvolved;
    int maxGripperContactCnt;
    std::string maxContactGripper;
};

// ... Helper functions definitions ...

double AngularDistance(const ignition::math::Vector3d &_v1, const ignition::math::Vector3d &_v2) {
    ignition::math::Vector3d v1 = _v1;
    ignition::math::Vector3d v2 = _v2;
    v1.Normalize();
    v2.Normalize();
    return acos(v1.Dot(v2));
}

bool CheckGrip(const std::vector<ignition::math::Vector3d> &forces, float minAngleDiff, float lengthRatio) {
     for (size_t i = 0; i < forces.size(); ++i) {
         for (size_t j = i + 1; j < forces.size(); ++j) {
             double angle = AngularDistance(forces[i], forces[j]);
             if (angle > minAngleDiff) {
                 double l1 = forces[i].Length();
                 double l2 = forces[j].Length();
                 if (l1 < 1e-4 || l2 < 1e-4) continue;
                 double ratio = (l1 > l2) ? l2/l1 : l1/l2;
                 if (ratio >= lengthRatio) return true;
             }
         }
     }
     return false;
}

void GazeboGraspFix::OnUpdate()
{
   if ((common::Time::GetWallTime() - this->prevUpdateTime) < this->updateRate) return;
   this->prevUpdateTime = common::Time::GetWallTime();

   this->mutexContacts.lock();
   std::map<std::string, std::map<std::string, CollidingPoint>> contPoints = this->contacts;
   this->contacts.clear();
   this->mutexContacts.unlock();

   std::map<std::string, ObjectContactInfo> objectContactInfo;
   
   // Aggregate contacts
   for (auto &obj : contPoints) {
       std::string objName = obj.first;
       ObjectContactInfo &info = objectContactInfo[objName];
       
       for (auto &link : obj.second) {
           CollidingPoint &cp = link.second;
           if (cp.sum == 0) continue;
           ignition::math::Vector3d avgForce = cp.force / cp.sum;
           info.appliedForces.push_back(avgForce);
           
           int &cnt = info.grippersInvolved[cp.gripperName];
           cnt++;
           if (cnt > info.maxGripperContactCnt) {
               info.maxGripperContactCnt = cnt;
               info.maxContactGripper = cp.gripperName;
           }
       }
   }

   std::set<std::string> grippedObjects;

   // Check attachment
   for (auto &oc : objectContactInfo) {
       std::string objName = oc.first;
       ObjectContactInfo &info = oc.second;
       
       if (!CheckGrip(info.appliedForces, this->forcesAngleTolerance, 0.3)) continue;
       
       grippedObjects.insert(objName);
       int &counts = this->gripCounts[objName];
       if (counts < this->maxGripCount) counts++;

       if (counts <= this->gripCountThreshold) continue;

       std::string attachedTo;
       if (ObjectAttachedToGripper(info, attachedTo)) continue; // Already attached to one of the involved grippers?

       // Attach!
       std::string gripperName = info.maxContactGripper;
       GazeboGraspGripper &gripper = grippers[gripperName];
       if (gripper.isObjectAttached()) continue;

       gzmsg << "GazeboGraspFix: Attaching " << objName << " to " << gripperName << std::endl;
       
       // Store initial contacts
       this->attachGripContacts[objName].clear();
       const auto &pts = contPoints[objName];
       for (auto &pt : pts) {
           if (gripper.hasCollisionLink(pt.first)) {
               this->attachGripContacts[objName][pt.first] = pt.second;
           }
       }

       gripper.HandleAttach(objName);
       OnAttach(objName, gripperName);
   }

   // Check detachment
   std::map<std::string, std::string> attachedObjs = GetAttachedObjects();
   for (auto &gc : this->gripCounts) {
       std::string objName = gc.first;
       if (grippedObjects.count(objName)) continue; // Still gripped

       if (gc.second > 0) gc.second--;

       if (attachedObjs.find(objName) == attachedObjs.end()) continue; // Not attached
       if (gc.second > this->gripCountThreshold) continue; // Hysteresis

       // Check release tolerance (distance check)
       std::string gripperName = attachedObjs[objName];
       auto &initColls = this->attachGripContacts[objName];
       int releaseCnt = 0;

       for (auto &pt : initColls) {
           CollidingPoint &cp = pt.second;
           if (cp.sum == 0) continue; 
           ignition::math::Vector3d relContact = cp.pos / cp.sum;
           ignition::math::Vector3d relObj = cp.objPos / cp.sum;
           
           // Current World Poses
           ignition::math::Pose3d currObjPose = cp.collObj->GetLink()->WorldPose();
           ignition::math::Pose3d currLinkPose = cp.collLink->GetLink()->WorldPose();
           
           // Reconstruct world pos of contact
           ignition::math::Vector3d currContactWorld = currLinkPose.CoordPositionAdd(relContact);
           
           double oldDist = (relContact - relObj).Length(); // Distance in Link frame
           double newDist = (currContactWorld - currObjPose.Pos()).Length();
           
           if (fabs(oldDist - newDist) > this->releaseTolerance) {
               releaseCnt++;
               // gzmsg << "Detach check: " << objName << " dist diff: " << fabs(oldDist - newDist) << " > " << this->releaseTolerance << std::endl;
           }
       }

       if (releaseCnt > 0) {
           gzmsg << "GazeboGraspFix: Detaching " << objName << " from " << gripperName << std::endl;
           grippers[gripperName].HandleDetach(objName);
           OnDetach(objName, gripperName);
       }
   }
}

void GazeboGraspFix::OnContact(const ConstContactsPtr &ptr)
{
    // Need mutex? Yes.
    boost::mutex::scoped_lock lock(this->mutexContacts);
    
    for (int i = 0; i < ptr->contact_size(); ++i) {
        const msgs::Contact &contact = ptr->contact(i);
        std::string name1 = contact.collision1();
        std::string name2 = contact.collision2();
        
        std::string gripperName;
        std::string collLink, collObj;
        
        if (collisions.count(name1)) {
            gripperName = collisions[name1];
            collLink = name1; 
            collObj = name2;
        } else if (collisions.count(name2)) {
            gripperName = collisions[name2];
            collLink = name2;
            collObj = name1;
        } else {
            continue; // Not involving our gripper
        }

        // Get collision pointers
        physics::CollisionPtr c1 = boost::dynamic_pointer_cast<physics::Collision>(GetEntityByName(world, collLink));
        physics::CollisionPtr c2 = boost::dynamic_pointer_cast<physics::Collision>(GetEntityByName(world, collObj));
        
        if (!c1 || !c2) continue;

        // Process forces
        for (int j = 0; j < contact.position_size(); ++j) {
             CollidingPoint &p = contacts[collObj][collLink];
             p.gripperName = gripperName;
             p.collLink = c1;
             p.collObj = c2;
             
             ignition::math::Vector3d force(contact.wrench(j).body_1_wrench().force().x(),
                                            contact.wrench(j).body_1_wrench().force().y(),
                                            contact.wrench(j).body_1_wrench().force().z());
             // Orient force correctly? If body1 is the gripper, force is ON body1. We want force ON object.
             // But simpler: just magnitude and direction relative to object matter.
             // Assuming basic plugin logic here.
             p.force += force;
             p.sum++;
             
             ignition::math::Vector3d pos(contact.position(j).x(), contact.position(j).y(), contact.position(j).z());
             // rel pos to link (Global -> Local)
             p.pos += c1->GetLink()->WorldPose().Inverse().CoordPositionAdd(pos);
             // rel obj pos (Global -> Local)
             p.objPos += c1->GetLink()->WorldPose().Inverse().CoordPositionAdd(c2->GetLink()->WorldPose().Pos());
        }
    }
}

bool GazeboGraspFix::IsGripperLink(const std::string &linkName, std::string &gripperName) const
{
  for (auto const& [name, gripper] : grippers)
  {
      if (gripper.hasLink(linkName)) {
          gripperName = name;
          return true;
      }
  }
  return false;
}

std::map<std::string, std::string> GazeboGraspFix::GetAttachedObjects() const
{
    std::map<std::string, std::string> ret;
    for (auto const& [name, gripper] : grippers) {
        if (gripper.isObjectAttached()) {
            ret[gripper.attachedObject()] = name;
        }
    }
    return ret;
}

bool GazeboGraspFix::ObjectAttachedToGripper(const ObjectContactInfo &objContInfo, std::string &attachedToGripper) const
{
    for (auto const& [name, cnt] : objContInfo.grippersInvolved) {
        if (ObjectAttachedToGripper(name, attachedToGripper)) return true;
    }
    return false;
}

bool GazeboGraspFix::ObjectAttachedToGripper(const std::string &gripperName, std::string &attachedToGripper) const
{
    if (grippers.find(gripperName) != grippers.end()) {
        if (grippers.at(gripperName).isObjectAttached()) {
            attachedToGripper = gripperName;
            return true;
        }
    }
    return false;
}

void GazeboGraspFix::OnAttach(const std::string &objectName, const std::string &armName)
{
    gzmsg << "Event: Attached " << objectName << " to " << armName << std::endl;
}

void GazeboGraspFix::OnDetach(const std::string &objectName, const std::string &armName)
{
    gzmsg << "Event: Detached " << objectName << " from " << armName << std::endl;
}
