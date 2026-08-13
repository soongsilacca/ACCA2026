import numpy as np


def mgeo_tokens_from_msg(msg, max_tokens, feature_dim, require_v2=True):
    """Convert Float32MultiArray-like bag message to padded [N,D] tokens."""
    values = np.asarray(msg.data, dtype=np.float32)
    dimensions = getattr(getattr(msg, "layout", None), "dim", [])
    label = dimensions[0].label if dimensions else ""
    if require_v2 and label != "mgeo_v2_points":
        raise ValueError("MGeo schema must be mgeo_v2_points, got '{}'".format(label or "<empty>"))
    if require_v2 and values.size != max_tokens * feature_dim:
        raise ValueError(
            "V2 MGeo must be {}x{}, got {} floats".format(max_tokens, feature_dim, values.size)
        )
    if values.size % feature_dim:
        raise ValueError("MGeo token data length is not divisible by feature_dim")
    values = values.reshape(-1, feature_dim)[:max_tokens]
    output = np.zeros((max_tokens, feature_dim), dtype=np.float32)
    mask = np.zeros(max_tokens, dtype=np.bool_)
    output[:len(values)] = values
    # V2 topic contract is a dense, fixed 64-row tensor.
    mask[:len(values)] = True
    return output, mask


def route_tokens_from_msg(msg, max_tokens, normalization=60.0, expected_frame="base_link", require_v2=True):
    """Convert nav_msgs/Path to ego-relative x/y and unit direction tokens."""
    frame_id = getattr(getattr(msg, "header", None), "frame_id", "").lstrip("/")
    expected_frame = expected_frame.lstrip("/")
    if frame_id != expected_frame:
        raise ValueError(
            "local_route frame must be '{}', got '{}'".format(expected_frame, frame_id or "<empty>")
        )
    points = np.asarray([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=np.float32)
    if require_v2 and len(points) != max_tokens:
        raise ValueError("V2 local_route must contain {} poses, got {}".format(max_tokens, len(points)))
    output = np.zeros((max_tokens, 4), dtype=np.float32)
    mask = np.zeros(max_tokens, dtype=np.bool_)
    points = points[:max_tokens]
    if len(points):
        output[:len(points), :2] = points / normalization
        delta = np.diff(points, axis=0)
        norm = np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-6)
        output[:len(delta), 2:] = delta / norm
        mask[:len(points)] = True
    return output, mask
