"""Multimodal learning models and ROS runtime helpers.

Keep package import lightweight: inference nodes only needing preprocessing or
one runtime model must not import the complete V17 training dependency graph.
"""

__all__ = ["GoalSpatialCandidateDataset", "GoalSpatialCandidatePlannerV17"]


def __getattr__(name):
    if name == "GoalSpatialCandidateDataset":
        from .data import GoalSpatialCandidateDataset
        return GoalSpatialCandidateDataset
    if name == "GoalSpatialCandidatePlannerV17":
        from .model import GoalSpatialCandidatePlannerV17
        return GoalSpatialCandidatePlannerV17
    raise AttributeError(name)
