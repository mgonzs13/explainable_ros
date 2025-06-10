import rclpy
from explainable_ros.explainability_client_node import ExplainabilityClientNode


def main(args=None):
    rclpy.init(args=args)
    client_node = ExplainabilityClientNode()

    questions = [
        "How many waypoints were received during the navigation task?",
        "What were the IDs of the waypoints received during the navigation task?",
        "Were all the waypoints received successfully reached?",
        "What happened during navigation to waypoint with ID 6?",
        "Why was the route replanned during navigation to waypoint with ID 6?",
        "Have any relevant events occurred during navigation?",
        "What is the task that the robot had to perform?",
        "Did the robot avoid any obstacle during the navigation?",
    ]

    markdown_table = """
| **User Question** | **LLM Answer** |
|-------------------|----------------|
"""
    for question in questions:
        print(f"Question: {question}")
        response = client_node.sed_request(question)
        print(f"Response: {response.answer}")

        answer_text = response.answer
        answer_text = answer_text.replace("\n", " ")
        answer_text = " ".join(answer_text.split())
        markdown_table += f"| {question} | {answer_text} |\n"

    # Save the markdown table to a file
    with open("explainability_responses.md", "w") as file:
        file.write(markdown_table)

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
