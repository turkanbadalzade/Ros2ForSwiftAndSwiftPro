from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # Build MoveIt config with OMPL as the default planner
    moveit_config = (
        MoveItConfigsBuilder("swiftpro", package_name="pro_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)


