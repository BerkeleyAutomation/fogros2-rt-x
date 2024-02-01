import sys
import pkgutil
import importlib
import subprocess

sys.path.append("../../fogros2_rt_x")
from plugins.orchestrator_base import BaseTopicOrchestrator


def import_submodules(package_name):
    package = importlib.import_module(package_name)
    package_path = package.__path__
    for _, name, is_pkg in pkgutil.walk_packages(package_path):
        full_name = f"{package_name}.{name}"
        importlib.import_module(full_name)
        if is_pkg:
            import_submodules(full_name)


def get_subclass_names(base_class):
    return [cls.__name__ for cls in base_class.__subclasses__()]


def get_orchestrators():
    import_submodules("plugins")
    return get_subclass_names(BaseTopicOrchestrator)


def export_rlds(observation_topics, action_topics, step_topics, dataset_name, destination):
    try:
        print([
            "ros2",
            "fgr",
            "export",
            "-o", *observation_topics,
            "-a", *action_topics,
            "-s", *step_topics,
            "--dataset_name", dataset_name,
            "--destination", destination 
        ])
        
        proc = subprocess.run(
            [
                "ros2",
                "fgr",
                "export",
                "-o", *observation_topics,
                "-a", *action_topics,
                "-s", *step_topics,
                "--dataset_name", dataset_name,
                "--destination", destination 
            ],
            stdout=subprocess.PIPE,
            text=True,
            check=True
        )
        print(proc.stdout)
        return proc.returncode == 0
    except subprocess.CalledProcessError as e:
        print("Command failed with return code:", e.returncode)
        print("Error message:", e.stderr)
        return False
    
