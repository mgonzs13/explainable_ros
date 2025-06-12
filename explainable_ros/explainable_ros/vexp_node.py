import math
import rclpy
import message_filters
from simple_node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from llama_msgs.action import GenerateChatCompletions
from llama_msgs.msg import ChatMessage
from llama_ros.llama_client_node import LlamaClientNode


class VisualExplainabilityNode(Node):
    def __init__(self):
        super().__init__("vexp_node")

        self._llama_client = LlamaClientNode.get_instance()

        self.previous_distance = float("inf")
        self.declare_parameter("distance_threshold", 5.0)
        self.distance_threshold = (
            self.get_parameter("distance_threshold").get_parameter_value().double_value
        )

        # subscribers
        camera_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw", qos_profile=10
        )
        plan_sub = message_filters.Subscriber(self, Path, "/plan", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (camera_sub, plan_sub), 10, 0.5
        )
        self._synchronizer.registerCallback(self.obstacle_detection_callback)

    def calculate_distance(self, p1, p2):
        distance = math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2 + (p2.z - p1.z) ** 2)
        return distance

    def obstacle_detection_callback(self, data: Image, plan: Path):
        total_distance = 0

        for i in range(len(plan.poses) - 1):
            p1 = plan.poses[i].pose.position
            p2 = plan.poses[i + 1].pose.position
            distance = self.calculate_distance(p1, p2)
            total_distance += distance

        if (
            self.previous_distance > 0
            and abs(self.previous_distance - total_distance) > self.distance_threshold
        ):
            self.get_logger().info(
                "Possible obstacle detected: Distance to the goal increase from {:.2f} meters to {:.2f} meters".format(
                    self.previous_distance, total_distance
                )
            )

            # VLM Code for Image-To-Text
            goal = GenerateChatCompletions.Goal()
            goal.messages = [
                ChatMessage(
                    role="system",
                    content="You are an AI assistant for a mobile robot. Your task is to analyze the camera data and provide information about obstacles in the environment.",
                ),
                ChatMessage(
                    role="user",
                    content="<__image__>Is there any obstacle in the image? If yes, describe it.",
                ),
            ]
            goal.images.append(data)
            goal.sampling_config.temp = 0.0
            goal.add_generation_prompt = True

            result, _ = self._llama_client.generate_chat_completions(goal)

            self.get_logger().info(
                f"Obstacle detection in camera: {result.choices[0].message.content}"
            )

        self.previous_distance = total_distance


def main(args=None):
    rclpy.init(args=args)
    node = VisualExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
