# Humanoid Robot Simulation

Table of Contents
- [Project Description](#project-description)
- [Core Architecture / Key Design Decisions](#core-architecture-key-design-decisions)
    - [File Structure](#file-structure)
    - [Class Structure](#class-structure)
- [Usage](#usage)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
    - [Running the Simulation](#running-the-simulation)
    - [CLI Options](#cli-options)
- [Testing](#testing)
    - [Running Tests](#running-tests)
    - [Test Structure](#test-structure)
- [Future Enhancements](#future-enhancements)
- [References](#references)

## [Project Description](#project-description)
This project simulates a simplified humanoid robot and its interaction with its environment. It includes functionalities such as sensor simulation, data processing, action planning, and task learning. The simulation is designed to provide a basic framework for understanding robot behavior and decision-making.

## [Core Architecture / Key Design Decisions](#core-architecture-key-design-decisions)
The robot simulation is structured around a modular design, with clear separation of concerns between different components.

### [File Structure](#file-structure)
The project has the following file structure:

- `humanoid.py`: Contains all the robot simulation code, including the classes for sensors, perception, planning, actuation, task learning, and the main robot class.
- `test_humanoid.py`: Contains the unit tests for the robot simulation.

### [Class Structure](#class-structure)
The code is organized into the following classes:

- `Sensor` (ABC): An abstract base class for all sensors. Defines the common interface for all sensors, including the abstract methods read_data() and calibrate(), and the abstract property sensor_type.
- `VisionSensor` (`Sensor`): Simulates a camera/vision system for object detection.
- `ProximitySensor` (`Sensor`): Simulates a proximity sensor for human detection.
- `EnvironmentSensor` (`Sensor`): Simulates environmental scanning sensors.
- `Perception`: Manages data from all sensors (`VisionSensor`, `ProximitySensor`, `EnvironmentSensor`).
- `Planning`: Handles the robot's decision-making processes.
- `Actuation`: Controls the robot's physical movements.
- `TaskLearner`: Learns tasks through observation and demonstration.
- `HumanoidRobot`: Integrates all robot functionality.

### [Key Design Decisions](#key-design-decisions)

- **Abstraction**: The Sensor abstract base class provides a common interface for all sensors, making it easy to add new sensor types in the future.
- **Modularity**: The system is divided into classes with specific responsibilities, improving maintainability and scalability.
- **Layered Architecture**: The code follows a layered architecture (Perception, Planning, Actuation) that mirrors the functional architecture of a typical robot.
- **Task Learning**: The TaskLearner class introduces a basic task learning capability, allowing the robot to learn and execute new tasks.
- **Simplified Simulation**: The ProximitySensor uses a simplified simulation for human detection, directly generating x, y coordinates to avoid complex calculations. This was a design choice to keep the focus on the overall system architecture.
- **Configuration File**: The use of `robot_tasks.json` to store task definitions allows for easy modification and extension of robot behaviors without modifying the core code.
- **Logging**: The inclusion of a logging mechanism provides better visibility into the robot's internal state and decision-making process.

## Usage

### [Prerequisites](#prerequisites)

- Python 3.10 or higher (A virtual environment is highly recommended)
- Faker library (`pip install Faker`)

### [Installation](#installation)

Clone the repository:
```
git clone https://github.com/ImraanMeyer/humanoid_oop.git
cd humanoid_oop
```

It is highly recommended to create a virtual environment with Python 3.10 or higher. For example:
```
python3.10 -m venv venv
source venv/bin/activate  # On Linux/macOS
venv\Scripts\activate  # On Windows
```

Install the required dependencies (`Faker`):
```
pip install -r requirements.txt
```

Running the Simulation
To run the simulation, execute the main program:
```
python humanoid_robot.py
```

### [CLI Options](#cli-options)

The CLI provides the following options:

- **Power On**: Powers on the robot.
- **Power Off**: Powers off the robot.
- **Execute Action**: Executes the robot's next decided action.
- **Learn Task**: Allows the user to teach the robot a new task by providing a task name and a sequence of steps.
- **Run Autonomous Cycles**: Executes a specified number of action cycles.
- **List Tasks**: Lists all learned tasks.
- **Get Perception Data**: Displays the data currently perceived by the robot's sensors.
- **Get Status**: Displays the current status of the robot.
- **Exit**: Exits the simulation.

### [Testing](#testing)

The project includes a comprehensive suite of unit tests to ensure the correct functionality of each module.

### [Running Tests](#running-tests)

To run the tests, execute the following command:
```
python -m unittest
```

### [Test Structure](#test-structure)

The tests are organized as follows:
- `test_humanoid.py`: Contains unit tests for all classes in `humanoid.py`, including `Sensor`, `VisionSensor`, `ProximitySensor`, `EnvironmentSensor`, `Perception`, `Planning`, `Actuation`, `TaskLearner`, and `HumanoidRobot`.

### [Future Enhancements](#future-enhancements)
- Incorporate real world data to implement more sophisticated task learning and sensor models.
- Develop a more advanced planning algorithm.

### [References](#references)
- [Faker](https://faker.readthedocs.io/en/master/)
- [Python Documentation](https://docs.python.org/3/)
- [Python Object-Oriented Programming](https://realpython.com/python3-object-oriented-programming/) (Real Python)
- [Logging in Python](https://realpython.com/python-logging/) (Real Python)
- [Getting Started With Testing in Python](https://realpython.com/python-testing/) (Real Python)
- [Robotic Sensors - Interface between robots and world](https://howtorobot.com/expert-insight/robotic-sensors-interface-between-robots-and-world/) (How to Robot)

