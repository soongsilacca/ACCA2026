import torch


def trajectory_metrics(output, target):
    prediction = output["selected_trajectory"] if isinstance(output, dict) else output
    distance = torch.linalg.norm(prediction[..., :2] - target[..., :2], dim=-1)
    yaw_error = prediction[..., 2] - target[..., 2]
    yaw_error = torch.atan2(torch.sin(yaw_error), torch.cos(yaw_error)).abs()
    speed_error = (prediction[..., 3] - target[..., 3]).abs()
    result = {
        "ade": distance.mean().item(),
        "fde": distance[:, -1].mean().item(),
        "yaw_mae": yaw_error.mean().item(),
        "speed_mae": speed_error.mean().item(),
    }
    for seconds, index in ((1, 4), (2, 9), (4, 19)):
        result["horizon_{}s".format(seconds)] = distance[:, index].mean().item()
    return result
