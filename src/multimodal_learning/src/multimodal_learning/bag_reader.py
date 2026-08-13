def read_bag(path, topics):
    """Read selected topics into timestamp-sorted lists (ROS import is lazy)."""
    import rosbag

    result = {name: [] for name in topics}
    reverse = {topic: name for name, topic in topics.items()}
    with rosbag.Bag(path, "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=list(reverse)):
            header = getattr(msg, "header", None)
            msg_stamp = header.stamp.to_sec() if header and header.stamp.to_sec() > 0 else stamp.to_sec()
            result[reverse[topic]].append((msg_stamp, msg))
    return result
