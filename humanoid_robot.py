from abc import ABC, abstractmethod
from typing import Dict, List, Any
import random
import os
import json
from faker import Faker
import math

from collections import deque

from utils.logger.logging import logging


class Sensor(ABC):
    """Abstract base class for all robot sensors"""

    @abstractmethod
    def read_data(self) -> Dict[str, Any]:
        """Read and return sensor data"""
        pass

    @abstractmethod
    def calibrate(self):
        """Calibrate the sensor"""
        pass

    @property
    @abstractmethod
    def sensor_type(self) -> str:
        """Return the type of sensor"""
        pass


class VisionSensor(Sensor):
    """Simulates a camera/vision system for object detection"""

    def __init__(self):
        self.objects = ["cup", "book", "pen", "chair", "box",
                        "table", "bottle", "mug", "pencil"]
        self.calibrated = False

    def read_data(self) -> Dict[str, Any]:
        if not self.calibrated:
            logging.warning("Vision sensor not calibrated!")

        return {
            "object_status": random.choice(["Detected", "Not Detected"]),
            "objects": random.sample(self.objects, k=random.randint(0, len(self.objects))),
            "object_distance": [random.randint(1, 10) for _ in range(len(self.objects))],
            "object_count": len(self.objects),
        }

    def calibrate(self):
        self.calibrated = True
        logging.info("Vision sensor calibrated")

    @property
    def sensor_type(self) -> str:
        return "Vision"


class ProximitySensor(Sensor):
    """
    Simulates a proximity sensor for human detection.
    """

    def __init__(self, detection_range=5.0):
        self.fake = Faker()
        self.range = detection_range
        self.calibrated = False

    def read_data(self) -> Dict[str, Any]:
        """
        Generates simplified synthetic sensor data for human detection for the Robotics Assignment.

        This function simulates a simplified human detection scenario. It generates synthetic data by directly 
        assigning random x, y positions within a square area defined by the sensor range. The function returns a dictionary containing
        information about detected humans, including their names, positions, distances, and movement.
        """
        human_count = random.randint(0, 4)
        humans = []

        for _ in range(human_count):
            # Generate random x, y positions within the sensor's range.
            x = round(random.uniform(-self.range, self.range), 2)
            y = round(random.uniform(-self.range, self.range), 2)
            distance = round((x**2 + y**2)**0.5, 2)  # calculate distance
            direction_degrees = round(math.degrees(
                math.atan2(y, x)), 1) if (x != 0 or y != 0) else 0
            humans.append({
                "name": self.fake.first_name(),
                "distance": distance,
                "direction_degrees": direction_degrees,
                "position": (x, y),
                "movement": random.choice(["walking", "standing", "running"])
            })

        return {
            "detected_humans": humans,
            "human_status": "Detected" if human_count > 0 else "Not Detected",
            "human_count": human_count,
            "nearest_human": min((h["distance"] for h in humans), default=None),
            "sensor_status": "active" if self.calibrated else "uncalibrated"
        }

    def calibrate(self):
        self.calibrated = True
        logging.info("Proximity sensor calibrated")

    @property
    def sensor_type(self) -> str:
        return "Proximity"


class EnvironmentSensor(Sensor):
    """Simulates environmental scanning sensors"""

    def __init__(self):
        self.calibrated = False
        self.obstacles = ["wall", "table", "chair"]
        self.free_space = ["area1", "area2", "area3"]

    def read_data(self) -> Dict[str, Any]:
        if not self.calibrated:
            logging.warning("Environment sensor not calibrated!")

        return {
            "environment_status": random.choice(["Safe", "Unsafe"]),
            "obstacles": random.sample(self.obstacles, k=random.randint(0, 3)),
            "free_space": random.sample(self.free_space, k=random.randint(1, 3)),
            "obstacle_count": len(self.obstacles) if self.obstacles else 0,
            "free_space_count": len(self.free_space) if self.free_space else 0,
        }

    def calibrate(self):
        self.calibrated = True
        logging.info("Environment sensor calibrated")

    @property
    def sensor_type(self) -> str:
        return "Environment"


class Perception:
    """Manages data from all robot sensors"""

    def __init__(self):
        self.vision = VisionSensor()
        self.proximity = ProximitySensor()
        self.environment = EnvironmentSensor()
        self.calibrate_sensors()

    def calibrate_sensors(self):
        """Calibrate all sensors on initialization"""
        self.vision.calibrate()
        self.proximity.calibrate()
        self.environment.calibrate()
        logging.info("All sensors calibrated")

    def identify_objects(self) -> Dict[str, Any]:
        """Get data from vision sensor"""
        return self.vision.read_data()

    def track_humans(self) -> Dict[str, Any]:
        """Get data from proximity sensor"""
        return self.proximity.read_data()

    def map_environment(self) -> Dict[str, Any]:
        """Get data from environment sensor"""
        return self.environment.read_data()

    def get_perception_data(self) -> Dict[str, Any]:
        """Get data from all sensors"""
        return {
            "vision": self.identify_objects(),
            "proximity": self.track_humans(),
            "environment": self.map_environment()
        }


class Planning:
    """Handles the robot's decision-making processes."""

    def __init__(self, perception: Perception):
        self.perception = perception
        self.action_queue = deque()  # Queue for managing actions
        self.action_stack = []       # Stack for tracking recent actions

    def calculate_probabilities(self, object_data, human_data, environment_data):
        """Calculate the probabilities of different actions based on sensory data. Returns the action with the highest probability.
            TODO:These probabilities are currently hardcoded. They should be loaded from a configuration file: robot_tasks.json
        """

        action = "move_forward"

        # 70% chance to avoid human
        if human_data["human_status"] == "Detected" and random.random() < 0.7:
            action = "avoid_human"

        # 50% chance to pick up cup
        if object_data["object_status"] == "Detected" and "cup" in object_data["objects"] and random.random() < 0.5:
            action = "pick_up_cup"

        # 30% chance to pick up cup
        if object_data["object_status"] == "Detected" and "cup" in object_data["objects"] and random.random() < 0.3:
            action = "pick_up_cup"

        # 20% chance to avoid obstacle
        if environment_data["environment_status"] == "Unsafe" and random.random() < 0.2:
            action = "avoid_obstacle"

        # 10% chance to pick up book
        if object_data["object_status"] == "Detected" and "book" in object_data["objects"] and random.random() < 0.1:
            action = "pick_up_book"

        # 10% chance to pick up pen
        if object_data["object_status"] == "Detected" and "pen" in object_data["objects"] and random.random() < 0.1:
            action = "pick_up_pen"

        # 10% chance to pick up pencil
        if object_data["object_status"] == "Detected" and "pencil" in object_data["objects"] and random.random() < 0.1:
            action = "pick_up_pencil"

        # 10% chance to pick up paper
        if object_data["object_status"] == "Detected" and "paper" in object_data["objects"] and random.random() < 0.1:
            action = "pick_up_paper"

        # 10% chance to pick up notebook
        if object_data["object_status"] == "Detected" and "notebook" in object_data["objects"] and random.random() < 0.1:
            action = "pick_up_notebook"

        return action

    def decide_action(self) -> str:
        """Decides the next action based on sensory data."""
        logging.info("Deciding action...")
        object_data = self.perception.identify_objects()
        human_data = self.perception.track_humans()
        environment_data = self.perception.map_environment()

        # Clear the action queue and populate it with potential actions
        self.action_queue.clear()
        self.action_queue.append(self.calculate_probabilities(
            object_data, human_data, environment_data))

        # Randomly shuffle the action queue to add variability
        random.shuffle(self.action_queue)

        # Get the next action from the queue
        next_action = self.action_queue.popleft()

        # Push the action to the stack for tracking
        self.action_stack.append(next_action)

        logging.info(f"Object data: {object_data}")
        logging.info(f"Human data: {human_data}")
        logging.info(f"Environment data: {environment_data}")
        logging.info(f"Next action: {next_action}")
        logging.info(f"Recent actions: {self.action_stack}")

        return next_action


class Actuation:
    """Controls the robot's physical movements."""

    def __init__(self, planning: Planning):
        self.planning = planning
        self.holding_object = None
        self.movement_stack = []  # Stack for tracking movements

    def execute_action(self):
        """Executes the decided action.
            TODO:These actions are currently hardcoded. They should be loaded from a configuration file: robot_tasks.json
        """
        action = self.planning.decide_action()

        if action == "pick_up_cup":
            self.pick_up_object("cup")
        elif action == "avoid_human":
            self.avoid_human()
        elif action == "move_forward":
            self.move_forward()
        elif action == "pick_up_book":
            self.pick_up_object("book")
        elif action == "avoid_obstacle":
            self.avoid_obstacle()
        elif action == "pick_up_pen":
            self.pick_up_object("pen")
        elif action == "pick_up_pencil":
            self.pick_up_object("pencil")
        elif action == "pick_up_paper":
            self.pick_up_object("paper")
        elif action == "pick_up_notebook":
            self.pick_up_object("notebook")

    def pick_up_object(self, object_name: str):
        """Picks up an object."""
        logging.info(f"Picking up {object_name}...")
        self.holding_object = object_name
        self.movement_stack.append(
            f"picked_up_{object_name}")  # Track action in stack

    def release_object(self):
        """Releases the held object."""
        logging.info("Releasing object...")
        self.holding_object = None
        self.movement_stack.append("released_object")  # Track action in stack

    def avoid_human(self):
        """Avoids a human."""
        logging.info("Avoiding human...")
        self.movement_stack.append("avoided_human")  # Track action in stack

    def move_forward(self):
        """Moves forward."""
        logging.info("Moving forward...")
        self.movement_stack.append("moved_forward")  # Track action in stack

    def avoid_obstacle(self):
        """Avoids an obstacle."""
        logging.info("Avoiding obstacle...")
        self.movement_stack.append("avoided_obstacle")  # Track action in stack


class TaskLearner:
    """Learns tasks through observation and demonstration."""

    def __init__(self, tasks_file: str = "robot_tasks.json"):
        self.learned_tasks = {}
        self.tasks_file = tasks_file
        self.task_stack = []  # Stack for tracking learned tasks
        self.load_tasks()

    def load_tasks(self):
        """Loads tasks from a file."""
        if os.path.exists(self.tasks_file):
            with open(self.tasks_file, "r") as f:
                self.learned_tasks = json.load(f)
            logging.info(
                f"Loaded {len(self.learned_tasks)} tasks from storage.")

    def save_tasks(self):
        """Saves tasks to a file."""
        with open(self.tasks_file, "w") as f:
            json.dump(self.learned_tasks, f, indent=2)
        logging.info(f"Saved {len(self.learned_tasks)} tasks to storage.")

    def learn_task(self, task_name: str, steps: List[str]):
        """Learns a new task."""
        if task_name in self.learned_tasks:
            logging.info(f"Task '{task_name}' already exists.")
            return False

        self.learned_tasks[task_name] = steps
        self.task_stack.append(task_name)  # Track learned task in stack
        self.save_tasks()
        logging.info(f"Task '{task_name}' learned successfully.")
        return True

    def execute_task(self, task_name: str):
        """Executes a learned task."""
        if task_name not in self.learned_tasks:
            logging.info(f"Task '{task_name}' not found.")
            return False

        logging.info(f"Executing task: '{task_name}'")
        for step in self.learned_tasks[task_name]:
            logging.info(f"Performing step: {step}")
        logging.info(f"Task '{task_name}' completed.")
        return True

    def list_tasks(self) -> List[str]:
        """Lists all learned tasks."""
        return list(self.learned_tasks.keys())


class HumanoidRobot:
    """Main class that integrates all robot functionality."""

    def __init__(self):
        self.perception = Perception()
        self.planning = Planning(self.perception)
        self.actuation = Actuation(self.planning)
        self.task_learner = TaskLearner()
        self.power_state = "off"

    def power_on(self):
        """Powers on the robot."""
        self.power_state = "on"
        logging.info("Robot powered on.")

    def power_off(self):
        """Powers off the robot."""
        self.power_state = "off"
        logging.info("Robot powered off.")

    def get_status(self) -> Dict[str, Any]:
        """Gets the current status of the robot."""
        return {
            "power_state": self.power_state,
            "holding_object": self.actuation.holding_object,
            "tasks_learned": len(self.task_learner.list_tasks()),
            # Show last 3 movements
            "recent_movements": self.actuation.movement_stack[-3:],
            # Show last 3 learned tasks
            "recent_tasks": self.task_learner.task_stack[-3:],
        }


def main():
    """Main CLI interface for the humanoid robot."""
    robot = HumanoidRobot()

    while True:
        print("\n--- Humanoid Robot CLI ---")
        print("1. Power On")
        print("2. Power Off")
        print("3. Execute Action")
        print("4. Learn Task")
        print("5. Run Autonomus Cycles")
        print("6. List Tasks")
        print("7. Get Perception Data")
        print("8. Get Status")
        print("9. Exit")

        choice = input("Enter your choice: ")

        match choice:
            case "1":
                robot.power_on()
            case "2":
                robot.power_off()
            case "3":
                if robot.power_state == "off":
                    logging.info(
                        "Robot is powered off. Please power on first.")
                else:
                    robot.actuation.execute_action()
            case "4":
                task_name = input("Enter task name: ")
                steps = input("Enter steps (comma-separated): ").split(",")
                robot.task_learner.learn_task(task_name, steps)
            case "5":
                cycles = int(input("Enter number of cycles: "))
                for _ in range(cycles):
                    print('\n')
                    robot.actuation.execute_action()
            case "6":
                print("Learned tasks:", robot.task_learner.list_tasks())
            case "7":
                print("Robot status:", robot.get_status())
            case "8":
                print("Perception data:", robot.perception.get_perception_data())
            case "9":
                logging.info("Exiting...")
                break
            case _:
                logging.error("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
