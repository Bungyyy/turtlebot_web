from setuptools import setup, find_packages

setup(
    name="turtlebot_web",
    version="2.0.0",
    packages=find_packages(),
    install_requires=[
        "flask>=2.3.0",
        "flask-socketio>=5.3.0",
        "flask-cors>=4.0.0",
        "eventlet>=0.33.0",
    ],
    data_files=[
        ("share/turtlebot_web/launch", [
            "launch/web_interface.launch.py",
            "launch/web_interface.launch",
        ]),
    ],
    entry_points={
        "console_scripts": [
            "turtlebot_web=app:main",
        ],
    },
)
