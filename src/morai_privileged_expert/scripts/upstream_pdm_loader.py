"""Load the vendored CARLA Garage AutoPilot without modifying upstream."""
import enum
import os
import sys
import types

from morai_carla_compat import Provider, carla_module


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "third_party", "carla_garage"))
for path in (os.path.join(ROOT, "team_code"), os.path.join(ROOT, "scenario_runner"),
             os.path.join(ROOT, "leaderboard")):
    if path not in sys.path: sys.path.insert(0, path)
sys.modules.setdefault("carla", carla_module())

# CARLA's agents package only contributes the route-command enum here.
agents = types.ModuleType("agents"); navigation = types.ModuleType("agents.navigation")
local_planner = types.ModuleType("agents.navigation.local_planner")
class RoadOption(enum.IntEnum):
    VOID=-1; LEFT=1; RIGHT=2; STRAIGHT=3; LANEFOLLOW=4; CHANGELANELEFT=5; CHANGELANERIGHT=6
local_planner.RoadOption = RoadOption
global_route_planner = types.ModuleType("agents.navigation.global_route_planner")
class GlobalRoutePlanner:
    def __init__(self, *args, **kwargs): pass
    def trace_route(self, *args, **kwargs): return []
global_route_planner.GlobalRoutePlanner = GlobalRoutePlanner
sys.modules.setdefault("agents", agents)
sys.modules.setdefault("agents.navigation", navigation)
sys.modules.setdefault("agents.navigation.local_planner", local_planner)
sys.modules.setdefault("agents.navigation.global_route_planner", global_route_planner)

# Inject only the two framework surfaces used by the upstream class. MORAI
# owns the lifecycle; ScenarioRunner/Leaderboard do not run a CARLA server.
srunner = types.ModuleType("srunner")
scenario_manager = types.ModuleType("srunner.scenariomanager")
provider_module = types.ModuleType("srunner.scenariomanager.carla_data_provider")
provider_module.CarlaDataProvider = Provider
sys.modules["srunner"] = srunner
sys.modules["srunner.scenariomanager"] = scenario_manager
sys.modules["srunner.scenariomanager.carla_data_provider"] = provider_module

leaderboard_pkg = types.ModuleType("leaderboard")
autoagents_pkg = types.ModuleType("leaderboard.autoagents")
autonomous_agent = types.ModuleType("leaderboard.autoagents.autonomous_agent")
autonomous_agent_local = types.ModuleType("leaderboard.autoagents.autonomous_agent_local")
class Track(enum.IntEnum): SENSORS=1; MAP=2
class AutonomousAgent:
    def __init__(self, *args, **kwargs):
        self._global_plan = []; self._global_plan_world_coord = []
        self.org_dense_route_world_coord = []
autonomous_agent.Track = Track; autonomous_agent.AutonomousAgent = AutonomousAgent
autonomous_agent_local.AutonomousAgent = AutonomousAgent
autoagents_pkg.autonomous_agent = autonomous_agent
autoagents_pkg.autonomous_agent_local = autonomous_agent_local
sys.modules["leaderboard"] = leaderboard_pkg
sys.modules["leaderboard.autoagents"] = autoagents_pkg
sys.modules["leaderboard.autoagents.autonomous_agent"] = autonomous_agent
sys.modules["leaderboard.autoagents.autonomous_agent_local"] = autonomous_agent_local

from autopilot import AutoPilot

__all__ = ["AutoPilot", "Provider", "RoadOption"]
