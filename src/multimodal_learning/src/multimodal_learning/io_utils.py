from pathlib import Path
import yaml


def load_yaml(path):
    with open(Path(path).expanduser(), "r") as f:
        return yaml.safe_load(f)


def package_config(name):
    import rospkg
    return str(Path(rospkg.RosPack().get_path("multimodal_learning")) / "config" / name)
