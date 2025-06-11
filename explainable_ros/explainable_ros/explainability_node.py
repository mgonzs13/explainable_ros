import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from explainable_ros_msgs.srv import Question

from langchain_chroma import Chroma
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate, HumanMessagePromptTemplate
from langchain.retrievers import ContextualCompressionRetriever

from llama_ros.langchain import ChatLlamaROS, LlamaROSEmbeddings, LlamaROSReranker


class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__("explainability_node")

        self.logs_number = 0
        self.total_time = 0
        self.embedding_number = 0
        self.msg_queue = []
        self.previous_msg = ""

        # To manage a queue of 10 most recent messages and avoid high amount of duplicated messages
        self.recent_msgs = []
        self.recent_msgs_conter = 0

        # Create vector database and retriever
        self.vector_db = Chroma(embedding_function=LlamaROSEmbeddings())
        self.retriever = self.vector_db.as_retriever(search_kwargs={"k": 20})

        # Create prompt
        prompt = ChatPromptTemplate.from_messages(
            [
                SystemMessage(
                    "You are an explainability AI tool for ROS 2 mobile robots. Your goal is to interpret the robot's data and answer questions."
                ),
                HumanMessagePromptTemplate.from_template(
                    "Taking into account the following logs:\n{context}\nAnswer the following question: {question}"
                ),
            ]
        )

        compressor = LlamaROSReranker(top_n=10)
        compression_retriever = ContextualCompressionRetriever(
            base_compressor=compressor, base_retriever=self.retriever
        )

        def format_docs(documents):
            logs = ""
            sortered_list = self.order_retrievals(documents)

            for l in sortered_list:
                logs += f"\t{l}"

            return logs

        # Create the chain
        self.rag_chain = (
            {
                "context": compression_retriever | format_docs,
                "question": RunnablePassthrough(),
            }
            | prompt
            | ChatLlamaROS(temp=0.0)
            | StrOutputParser()
        )

        # Create subscription for /rosout topic
        self.subscription = self.create_subscription(
            Log,
            "/rosout",
            self.listener_callback,
            1000,
        )

        # Create a ROS 2 Service to make question to de model
        self.srv = self.create_service(
            Question,
            "question",
            self.question_server_callback,
        )

        self.emb_timer = self.create_timer(0.001, self.emb_cb)

    def listener_callback(self, log: Log) -> None:
        self.logs_number += 1

        if log.name == self.get_logger().name:
            # Avoiding the node's own logs
            return

        # For not considering llama_ros logs
        if "llama" in log.name and "llava" not in log.name:
            return

        # Save logs
        self.msg_queue.append(log)
        self.get_logger().info(f"Log {self.logs_number}: {log.msg}")

    def emb_cb(self) -> None:

        if self.msg_queue:
            log = self.msg_queue.pop(0)

            # Eliminar solo el mensaje anterior
            if log.msg != self.previous_msg:
                start = time.time()

                msg_sec = log.stamp.sec
                msg_nanosec = log.stamp.nanosec
                unix_timestamp = msg_sec + msg_nanosec / 1e9

                self.vector_db.add_texts([f"{unix_timestamp} - {log.msg}"])

                self.previous_msg = log.msg
                self.embedding_number += 1

                emb_time = time.time() - start
                self.total_time += emb_time
                self.get_logger().info(
                    f"Time to create embedding {self.embedding_number}: {emb_time} | Total time: {self.total_time}"
                )

    def order_retrievals(self, docuemnt_list):

        aux_list = []

        for d in docuemnt_list:
            aux_list.append(d.page_content + "\n")

        return sorted(aux_list)

    def question_server_callback(
        self, request: Question.Request, response: Question.Response
    ) -> Question.Response:

        question_text = str(request.question)
        answer = self.rag_chain.invoke(question_text)
        response.answer = answer
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExplainabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
