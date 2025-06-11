from setuptools import find_packages, setup

package_name = "explainable_ros_demos"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    description="Explainability demos",
    license="MIT",
    entry_points={
        "console_scripts": [
            "explainable_demo_node = explainable_ros_demos.explainable_demo_node:main",
            "vlm_explainable_demo_node = vlm_explainable_ros_demos.explainable_demo_node:main",
        ],
    },
)
