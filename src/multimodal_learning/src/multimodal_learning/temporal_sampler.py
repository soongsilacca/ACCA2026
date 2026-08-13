from .synchronizer import nearest_index


def history_indices(stamps, current, history_frames, interval=0.25, tolerance=0.08):
    """Select frames at fixed timestamp offsets over a one-second history."""
    current_stamp = stamps[current]
    indices = []
    for offset in reversed(range(history_frames)):
        index = nearest_index(stamps, current_stamp - offset * interval, tolerance)
        if index is None or index > current:
            return None
        indices.append(index)
    return indices
