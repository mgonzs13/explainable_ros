import rclpy
from explainable_ros.explainability_client_node import ExplainabilityClientNode


def main(args=None):
    rclpy.init(args=args)
    client_node = ExplainabilityClientNode()

    questions = [
        "Did the robot encounter any obstacles during navigation? What type of obstacleis.?"
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
