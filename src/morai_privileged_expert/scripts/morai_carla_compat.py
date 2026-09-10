"""CARLA API compatibility objects backed by MORAI state.

This module does not implement a simulator.  It provides the geometry and
actor surface consumed by the unmodified CARLA Garage ``team_code/autopilot``.
"""
import fnmatch
import math
import types
from enum import IntEnum, IntFlag

import numpy as np


class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        if hasattr(x, "x"):
            x, y, z = x.x, x.y, x.z
        self.x, self.y, self.z = float(x), float(y), float(z)
    def length(self): return math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
    def __add__(self, other): return self.__class__(self.x+other.x, self.y+other.y, self.z+other.z)
    def __sub__(self, other): return self.__class__(self.x-other.x, self.y-other.y, self.z-other.z)
    def __mul__(self, value): return self.__class__(self.x*value, self.y*value, self.z*value)
    __rmul__ = __mul__
    def dot(self, other): return self.x*other.x + self.y*other.y + self.z*other.z


class Vector2D(Vector3D): pass


class Location(Vector3D):
    def distance(self, other): return (self-other).length()


class Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0): self.pitch, self.yaw, self.roll = float(pitch), float(yaw), float(roll)
    def get_forward_vector(self):
        yaw, pitch = math.radians(self.yaw), math.radians(self.pitch)
        return Vector3D(math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), math.sin(pitch))
    def get_right_vector(self):
        yaw = math.radians(self.yaw + 90.0)
        return Vector3D(math.cos(yaw), math.sin(yaw), 0.0)
    def get_up_vector(self): return Vector3D(0.0, 0.0, 1.0)


class Transform:
    def __init__(self, location=None, rotation=None): self.location, self.rotation = location or Location(), rotation or Rotation()
    def transform(self, value):
        yaw = math.radians(self.rotation.yaw); c, s = math.cos(yaw), math.sin(yaw)
        return Location(self.location.x+c*value.x-s*value.y,
                        self.location.y+s*value.x+c*value.y,
                        self.location.z+value.z)
    def get_forward_vector(self): return self.rotation.get_forward_vector()
    def get_matrix(self):
        yaw, pitch, roll = map(math.radians, (self.rotation.yaw, self.rotation.pitch, self.rotation.roll))
        cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        matrix = np.array([[cp*cy, cy*sp*sr-sy*cr, -cy*sp*cr-sy*sr, self.location.x],
                           [cp*sy, sy*sp*sr+cy*cr, -sy*sp*cr+cy*sr, self.location.y],
                           [sp, -cp*sr, cp*cr, self.location.z], [0., 0., 0., 1.]])
        return matrix.tolist()
    def get_inverse_matrix(self): return np.linalg.inv(np.asarray(self.get_matrix())).tolist()


class BoundingBox:
    def __init__(self, location=None, extent=None):
        self.location, self.extent, self.rotation = location or Location(), extent or Vector3D(), Rotation()


class VehicleControl:
    def __init__(self, throttle=0.0, steer=0.0, brake=0.0):
        self.throttle, self.steer, self.brake = float(throttle), float(steer), float(brake)


class WalkerControl:
    def __init__(self, direction=None): self.direction = direction or Vector3D()


class TrafficLightState(IntEnum): Red=0; Yellow=1; Green=2; Off=3; Unknown=4
class VehicleLightState(IntFlag): NONE=0; Position=1; LowBeam=2
class LaneChange(IntFlag): NONE=0; Right=1; Left=2; Both=3
class LaneType(IntFlag): None_=0; Driving=1; Shoulder=2; Any=0xffffffff


class Actor:
    def __init__(self, actor_id=0, type_id="", transform=None, velocity=None, extent=None):
        self.id, self.type_id = int(actor_id), str(type_id)
        self._transform = transform or Transform(); self._velocity = velocity or Vector3D()
        self.bounding_box = BoundingBox(Location(), extent or Vector3D(.5, .5, .5))
        self._control = VehicleControl(); self.attributes = {}; self.is_alive = True
        self.trigger_volume = BoundingBox(Location(), Vector3D(1.5, 1.5, 1.0))
    def get_location(self): return self._transform.location
    def get_transform(self): return self._transform
    def get_velocity(self): return self._velocity
    def get_control(self): return self._control
    def get_world(self): return _Provider.world
    def destroy(self): self.is_alive = False


class Vehicle(Actor): pass
class Walker(Actor): pass
class TrafficLight(Actor):
    def __init__(self, *args, **kwargs): super().__init__(*args, **kwargs); self.state = TrafficLightState.Red
class TrafficSign(Actor): pass
class StopSign(TrafficSign): pass


class ActorList(list):
    def filter(self, pattern): return ActorList([actor for actor in self if fnmatch.fnmatch(actor.type_id, pattern)])


class Waypoint:
    def __init__(self, transform=None, road_id=0, lane_id=0, is_junction=False, lane_width=3.5):
        self.transform, self.road_id, self.lane_id = transform or Transform(), road_id, lane_id
        self.is_junction = bool(is_junction); self.lane_type = LaneType.Driving
        self.lane_change = LaneChange.Both; self.lane_width = float(lane_width)
    def next(self, _distance): return []
    def previous(self, _distance): return []
    def _side_lane(self, sign):
        yaw = math.radians(self.transform.rotation.yaw)
        loc = self.transform.location
        shifted = Location(loc.x-sign*math.sin(yaw)*self.lane_width,
                           loc.y+sign*math.cos(yaw)*self.lane_width, loc.z)
        return Waypoint(Transform(shifted, self.transform.rotation), self.road_id,
                        self.lane_id+sign, self.is_junction, self.lane_width)
    def get_left_lane(self): return self._side_lane(1)
    def get_right_lane(self): return self._side_lane(-1)


class Map:
    def __init__(self): self.waypoint_resolver = None
    def get_waypoint(self, location, *args, **kwargs):
        return self.waypoint_resolver(location) if self.waypoint_resolver else Waypoint(Transform(location))


class _Debug:
    def __getattr__(self, _name): return lambda *args, **kwargs: None


class World:
    def __init__(self): self.actors, self.map, self.debug = ActorList(), Map(), _Debug()
    def get_actors(self): return self.actors
    def get_actor(self, actor_id): return next((a for a in self.actors if a.id == actor_id), None)
    def get_map(self): return self.map


class _Provider:
    world = World(); hero = None; active_scenarios = []
    @classmethod
    def get_map(cls): return cls.world.map
    @classmethod
    def get_hero_actor(cls): return cls.hero
    @classmethod
    def get_velocity(cls, actor): return actor.get_velocity().length()
    @classmethod
    def get_client(cls): return types.SimpleNamespace(get_world=lambda: cls.world)


def carla_module():
    module = types.ModuleType("carla")
    for name, value in globals().copy().items():
        if name[0].isupper() and name not in ("IntEnum", "IntFlag"):
            setattr(module, name, value)
    module.libcarla = module
    module.Color = lambda *args, **kwargs: None
    module.Vector = Vector3D
    return module


Provider = _Provider
