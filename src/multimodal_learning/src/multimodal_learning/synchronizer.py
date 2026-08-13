from bisect import bisect_left


def nearest_index(stamps, target, tolerance):
    """Return index of the nearest timestamp, or None outside tolerance."""
    if not stamps:
        return None
    pos = bisect_left(stamps, target)
    candidates = [i for i in (pos - 1, pos) if 0 <= i < len(stamps)]
    idx = min(candidates, key=lambda i: abs(stamps[i] - target))
    return idx if abs(stamps[idx] - target) <= tolerance else None


def synchronize(reference, streams, tolerance):
    """Synchronize sorted (timestamp, value) streams to a reference stream."""
    stamp_tables = {name: [x[0] for x in values] for name, values in streams.items()}
    for stamp, value in reference:
        row = {"stamp": stamp, "reference": value}
        for name, values in streams.items():
            idx = nearest_index(stamp_tables[name], stamp, tolerance)
            if idx is None:
                break
            row[name] = values[idx][1]
        else:
            yield row
