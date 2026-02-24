from setuptools import find_packages, setup

package_name = "my_robot_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Robot controller nodes (goal avoid)",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run my_robot_controller goal_avoid
            "avoidance_node = my_robot_controller.avoidance_node:main",
            "avoidance_node_v2 = my_robot_controller.avoidance_node_v2:main",
            "goal_avoid_node = my_robot_controller.goal_avoid_node:main",
        ],
    },
)