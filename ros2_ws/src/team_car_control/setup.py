from setuptools import find_packages, setup


package_name = "team_car_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/control.launch.py",
                "launch/bringup.launch.py",
                "launch/actea_bringup.launch.py",
                "launch/actea_control.launch.py",
            ],
        ),
        (f"share/{package_name}/routes", ["routes/mbgazworld_route.json"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="ENPM661 Group 4",
    maintainer_email="TODO@email.com",
    description="Phase 2 planner action server and Gazebo executor for the team car.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigate_action_server = team_car_control.navigate_action_server:main",
            "send_navigate_goal = team_car_control.send_navigate_goal:main",
            "actea_route_follower = team_car_control.actea_route_follower:main",
        ],
    },
)
