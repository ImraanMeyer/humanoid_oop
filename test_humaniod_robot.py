import unittest
import os
from unittest.mock import patch
from humanoid_robot import (
    HumanoidRobot, Perception, Planning, Actuation, TaskLearner, VisionSensor, ProximitySensor, EnvironmentSensor)


class TestSensor(unittest.TestCase):
    def test_sensor_calibration(self):
        # Setup
        vision_sensor = VisionSensor()
        proximity_sensor = ProximitySensor()
        environment_sensor = EnvironmentSensor()

        # Assert
        self.assertFalse(vision_sensor.calibrated)
        self.assertFalse(proximity_sensor.calibrated)
        self.assertFalse(environment_sensor.calibrated)

        # Act
        vision_sensor.calibrate()
        proximity_sensor.calibrate()
        environment_sensor.calibrate()

        # Assert
        self.assertTrue(vision_sensor.calibrated)
        self.assertTrue(proximity_sensor.calibrated)
        self.assertTrue(environment_sensor.calibrated)

    def test_vision_sensor_read_data(self):
        # Setup
        vision_sensor = VisionSensor()
        # Act
        data = vision_sensor.read_data()
        # Assert
        self.assertIn("object_status", data)
        self.assertIn("objects", data)
        self.assertIn("object_distance", data)
        self.assertIn("object_count", data)
        self.assertIsInstance(data["objects"], list)
        self.assertIsInstance(data["object_distance"], list)
        self.assertIsInstance(data["object_count"], int)

    def test_proximity_sensor_read_data(self):
        # Setup
        proximity_sensor = ProximitySensor()
        # Act
        data = proximity_sensor.read_data()
        # Assert
        self.assertIn("human_status", data)
        self.assertIn("detected_humans", data)
        self.assertIn("human_count", data)
        self.assertIn("nearest_human", data)
        self.assertIsInstance(data["detected_humans"], list)
        self.assertIsInstance(data["human_count"], int)
        if data["human_count"] > 0:
            self.assertIsInstance(data["nearest_human"], (int, float))

    def test_environment_sensor_read_data(self):
        # Setup
        environment_sensor = EnvironmentSensor()
        # Act
        data = environment_sensor.read_data()
        # Assert
        self.assertIn("environment_status", data)
        self.assertIn("obstacles", data)
        self.assertIn("free_space", data)
        self.assertIn("obstacle_count", data)
        self.assertIn("free_space_count", data)
        self.assertIsInstance(data["obstacles"], list)
        self.assertIsInstance(data["free_space"], list)
        self.assertIsInstance(data["obstacle_count"], int)
        self.assertIsInstance(data["free_space_count"], int)


class TestPerception(unittest.TestCase):
    def setUp(self):
        self.perception = Perception()

    def test_calibrate_sensors(self):
        # Act
        self.perception.calibrate_sensors()
        # Assert
        self.assertTrue(self.perception.vision.calibrated)
        self.assertTrue(self.perception.proximity.calibrated)
        self.assertTrue(self.perception.environment.calibrated)

    @patch.object(VisionSensor, 'read_data')
    def test_identify_objects(self, mock_read):
        # Setup mock return values
        mock_read.return_value = {
            "object_status": "Detected",
            "objects": ["cup", "book"],
            "object_distance": [5, 5],
            "object_count": 2
        }

        # Call method
        result = self.perception.identify_objects()

        # Assert results
        self.assertEqual(result["object_status"], "Detected")
        self.assertEqual(result["objects"], ["cup", "book"])
        self.assertEqual(result["object_distance"], [5, 5])

    @patch('random.choice')
    @patch('random.randint')
    def test_tracking_humans_not_found(self, mock_randint, mock_choice):
        # Setup mock return values
        mock_randint.return_value = 0  # human_count = 0
        mock_choice.return_value = "Not Detected"

        # Call method
        result = self.perception.track_humans()

        # Assert results
        self.assertEqual(result["human_status"], "Not Detected")
        self.assertEqual(result["detected_humans"], [])
        self.assertEqual(result["human_count"], 0)
        self.assertEqual(result["nearest_human"], None)
        self.assertEqual(result["sensor_status"], "active")

    @patch('random.choice')
    @patch('random.randint')
    @patch('random.uniform')
    def test_tracking_humans_found(self, mock_uniform, mock_randint, mock_choice):
        # Setup mock return values
        mock_randint.return_value = 1  # human_count = 1
        mock_uniform.side_effect = [
            2.0,  # x
            2.0,  # y
            2.83,  # distance
            45.0,  # direction_degrees
        ]
        mock_choice.return_value = "walking"

        # Act
        result = self.perception.track_humans()

        # Assert results
        self.assertEqual(result["human_status"], "Detected")
        self.assertEqual(len(result["detected_humans"]), 1)
        self.assertEqual(result["human_count"], 1)
        self.assertAlmostEqual(result["nearest_human"], 2.83, places=2)
        self.assertEqual(result["sensor_status"], "active")
        self.assertAlmostEqual(
            result["detected_humans"][0]["distance"], 2.83, places=2)
        self.assertAlmostEqual(
            result["detected_humans"][0]["direction_degrees"], 45.0, places=2)
        self.assertEqual(result["detected_humans"][0]["position"], (2.0, 2.0))
        self.assertEqual(result["detected_humans"][0]["movement"], "walking")

    @patch('random.choice')
    @patch('random.sample')
    def test_map_environment(self, mock_sample, mock_choice):
        # Setup mock return values
        mock_choice.return_value = "Safe"
        mock_sample.side_effect = [["wall"], ["area1", "area2"]]

        # Act
        result = self.perception.map_environment()

        # Assert results
        self.assertEqual(result["environment_status"], "Safe")
        self.assertEqual(result["obstacles"], ["wall"])
        self.assertEqual(result["free_space"], ["area1", "area2"])


class TestPlanning(unittest.TestCase):
    def setUp(self):
        self.perception = Perception()
        self.planning = Planning(self.perception)

    @patch.object(Perception, 'identify_objects')
    @patch.object(Perception, 'track_humans')
    @patch.object(Perception, 'map_environment')
    @patch('random.random')
    def test_decide_action(self, mock_random, mock_map, mock_track, mock_identify):
        # Setup mock return values
        mock_identify.return_value = {
            "object_status": "Detected",
            "objects": ["cup"],
            "object_distance": 2
        }
        mock_track.return_value = {
            "human_status": "Detected",
            "humans": ["human1"],
            "human_distance": 3
        }
        mock_map.return_value = {
            "environment_status": "Safe",
            "obstacles": [],
            "free_space": ["area1"]
        }
        mock_random.return_value = 0.4  # Below threshold to pick up cup

        # Act
        action = self.planning.decide_action()

        # Assert results
        self.assertIn(action, ["pick_up_cup", "avoid_human"])
        self.assertEqual(len(self.planning.action_stack), 1)
        self.assertEqual(self.planning.action_stack[0], action)


class TestActuation(unittest.TestCase):
    def setUp(self):
        perception = Perception()
        planning = Planning(perception)
        self.actuation = Actuation(planning)

    @patch.object(Planning, 'decide_action')
    def test_execute_action(self, mock_decide):
        mock_decide.return_value = "pick_up_cup"
        self.actuation.execute_action()
        self.assertEqual(self.actuation.holding_object, "cup")
        self.assertEqual(self.actuation.movement_stack[-1], "picked_up_cup")

    def test_pick_up_object(self):
        self.actuation.pick_up_object("book")
        self.assertEqual(self.actuation.holding_object, "book")
        self.assertEqual(self.actuation.movement_stack[-1], "picked_up_book")

    def test_avoid_human(self):
        self.actuation.avoid_human()
        self.assertEqual(self.actuation.movement_stack[-1], "avoided_human")

    def test_move_forward(self):
        self.actuation.move_forward()
        self.assertEqual(self.actuation.movement_stack[-1], "moved_forward")


class TestTaskLearner(unittest.TestCase):
    def setUp(self):
        self.task_learner = TaskLearner(tasks_file="test_tasks.json")
        # Clean up any existing test file
        if os.path.exists(self.task_learner.tasks_file):
            os.remove(self.task_learner.tasks_file)

    def tearDown(self):
        # Clean up after each test
        if os.path.exists(self.task_learner.tasks_file):
            os.remove(self.task_learner.tasks_file)

    def test_learn_and_execute_task(self):
        # Test learning a new task
        result = self.task_learner.learn_task(
            "make_tea", ["boil_water", "add_tea", "serve"])
        self.assertTrue(result)
        self.assertIn("make_tea", self.task_learner.learned_tasks)

        # Test executing the task
        result = self.task_learner.execute_task("make_tea")
        self.assertTrue(result)

    def test_list_tasks(self):
        self.task_learner.learn_task("task1", ["step1"])
        self.task_learner.learn_task("task2", ["step1", "step2"])

        tasks = self.task_learner.list_tasks()
        self.assertEqual(len(tasks), 2)
        self.assertIn("task1", tasks)
        self.assertIn("task2", tasks)

    def test_task_persistence(self):
        # Test that tasks are saved to and loaded from file
        self.task_learner.learn_task("persistent_task", ["step1"])
        self.task_learner.save_tasks()

        self.assertIn("persistent_task", self.task_learner.learned_tasks)


class TestHumanoidRobot(unittest.TestCase):
    def setUp(self):
        self.robot = HumanoidRobot()

    def test_power_states(self):
        self.assertEqual(self.robot.power_state, "off")
        self.robot.power_on()
        self.assertEqual(self.robot.power_state, "on")
        self.robot.power_off()
        self.assertEqual(self.robot.power_state, "off")

    @patch.object(Actuation, 'execute_action')
    def test_execute_action_when_powered_on(self, mock_execute):
        self.robot.power_on()
        self.robot.actuation.execute_action()
        mock_execute.assert_called_once()

    def test_execute_action_when_powered_off(self):
        self.robot.power_off()
        # We need to test the CLI behavior, not direct actuation call
        with patch('builtins.print') as mock_print:
            # Simulate what happens in the CLI when powered off
            if self.robot.power_state == "off":
                print("Robot is powered off. Please power on first.")
            else:
                self.robot.actuation.execute_action()

            mock_print.assert_called_once_with(
                "Robot is powered off. Please power on first.")

    def test_get_status(self):
        self.robot.power_on()
        self.robot.actuation.pick_up_object("pen")

        status = self.robot.get_status()
        self.assertEqual(status["power_state"], "on")
        self.assertEqual(status["holding_object"], "pen")
        self.assertEqual(status["recent_movements"], ["picked_up_pen"])

    def test_get_status_when_powered_off(self):
        self.robot.power_off()
        status = self.robot.get_status()
        self.assertEqual(status["power_state"], "off")
        self.assertEqual(status["holding_object"], None)
        self.assertEqual(status["recent_movements"], [])


if __name__ == "__main__":
    unittest.main()
