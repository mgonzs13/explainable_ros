import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from llama_bringup.utils import create_llama_launch_from_yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_directory = get_package_share_directory("explainable_ros_bringup")

    embbedings_model = GroupAction(
        [
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "bge-base-en-v1.5.yaml")
            ),
        ]
    )

    reranker_model = GroupAction(
        [
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "jina-reranker.yaml")
            ),
        ]
    )

    base_model = GroupAction(
        [
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "Qwen2.yaml")
            ),
        ]
    )

    vlm_model = GroupAction(
        [
            PushRosNamespace("llava"),
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "SpaceOm.yaml")
            ),
        ],
    )

    explainability_node_cmd = Node(
        package="explainable_ros",
        executable="explainability_node",
        name="explainability_node",
        output="screen",
    )

    vexp_node_cmd = Node(
        package="explainable_ros",
        executable="vexp_node",
        name="vexp_node",
        namespace="llava/llama",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(embbedings_model)
    ld.add_action(reranker_model)
    ld.add_action(base_model)
    ld.add_action(vlm_model)
    ld.add_action(explainability_node_cmd)
    ld.add_action(vexp_node_cmd)

    return ld
