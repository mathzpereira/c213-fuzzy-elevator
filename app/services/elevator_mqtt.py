import os
import paho.mqtt.client as mqtt
import json
import time
import threading
from typing import Optional, Callable
from app.services.fuzzy_control import ElevatorFuzzyController
from dotenv import load_dotenv

load_dotenv()


class ElevatorMQTT:
    def __init__(self):
        self.broker_host = os.getenv("HIVEMQ_BROKER")
        self.broker_port = int(os.getenv("HIVEMQ_PORT"))
        self.client = mqtt.Client()
        self.client.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(
            os.getenv("HIVEMQ_USERNAME"),
            os.getenv("HIVEMQ_PASSWORD"),
        )
        self.controller = ElevatorFuzzyController()

        self.current_floor = "terreo"
        self.current_position = 4.0
        self.target_floor = None
        self.target_position = None
        self.is_moving = False
        self.direction = 0
        self.previous_error = 0.0

        self.simulation_thread = None
        self.stop_simulation = False

        self.position_callback: Optional[Callable] = None
        self.status_callback: Optional[Callable] = None

        self.topics = {
            "floor_request": "elevator/floor_request",
            "position_update": "elevator/position_update",
            "status_update": "elevator/status_update",
        }

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            client.subscribe(self.topics["floor_request"])
            print(f"Subscribed to topic: {self.topics['floor_request']}")
        else:
            print(f"Failed to connect to MQTT broker. Return code: {rc}")

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())

            if topic == self.topics["floor_request"]:
                self._handle_floor_request(payload)

        except json.JSONDecodeError:
            print(f"Invalid JSON received on topic {msg.topic}: {msg.payload}")
        except Exception as e:
            print(f"Error processing message on topic {msg.topic}: {e}")

    def _on_disconnect(self, client, userdata, rc):
        print(f"Disconnected from MQTT broker. Return code: {rc}")

    def _handle_floor_request(self, payload):
        try:
            requested_floor = payload.get("floor")
            if requested_floor and not self.is_moving:
                print(f"Floor request received: {requested_floor}")
                self.move_to_floor(requested_floor)
            elif self.is_moving:
                print(f"Elevator is moving. Request for {requested_floor} ignored.")
        except Exception as e:
            print(f"Error handling floor request: {e}")

    def connect(self):
        try:
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")
            return False

    def disconnect(self):
        self.stop_simulation = True
        if self.simulation_thread and self.simulation_thread.is_alive():
            self.simulation_thread.join()
        self.client.loop_stop()
        self.client.disconnect()

    def move_to_floor(self, target_floor: str):
        if self.is_moving:
            print("Elevator is already moving")
            return False

        try:
            target_position = self.controller.get_floor_position(target_floor)

            if abs(target_position - self.current_position) < 0.1:
                print(f"Already at floor {target_floor}")
                return False

            self.target_floor = target_floor
            self.target_position = target_position
            self.direction = 1 if target_position > self.current_position else -1
            self.is_moving = True
            self.previous_error = target_position - self.current_position

            self.stop_simulation = False
            self.simulation_thread = threading.Thread(
                target=self._run_movement_simulation
            )
            self.simulation_thread.start()

            self._publish_status_update()

            return True

        except ValueError as e:
            print(f"Invalid floor request: {e}")
            return False

    def _run_movement_simulation(self):
        tolerance = 0.1
        max_iterations = 300
        iteration = 0

        print(f"Starting movement from {self.current_floor} to {self.target_floor}")

        while (
            not self.stop_simulation
            and iteration < max_iterations
            and abs(self.target_position - self.current_position) > tolerance
        ):
            try:
                motor_power, current_error = self.controller.compute_control(
                    self.current_position, self.target_position, self.previous_error
                )
                self.current_position = self.controller.update_position(
                    self.current_position, motor_power, self.direction
                )

                self.previous_error = current_error

                position_data = {
                    "timestamp": time.time(),
                    "current_position": self.current_position,
                    "target_position": self.target_position,
                    "current_floor": self._get_nearest_floor(),
                    "target_floor": self.target_floor,
                    "motor_power": motor_power,
                    "error": current_error,
                    "direction": "up" if self.direction > 0 else "down",
                    "is_moving": True,
                }

                self._publish_position_update(position_data)
                if self.position_callback:
                    self.position_callback(position_data)

                iteration += 1
                time.sleep(self.controller.sampling_time)

            except Exception as e:
                print(f"Error in movement simulation: {e}")
                break

        self.is_moving = False
        self.direction = 0
        self.current_floor = self._get_nearest_floor()

        final_data = {
            "timestamp": time.time(),
            "current_position": self.current_position,
            "target_position": self.target_position,
            "current_floor": self.current_floor,
            "target_floor": self.target_floor,
            "motor_power": 0,
            "error": self.target_position - self.current_position,
            "direction": "stopped",
            "is_moving": False,
            "movement_completed": True,
        }

        self._publish_position_update(final_data)
        self._publish_status_update()

        if self.position_callback:
            self.position_callback(final_data)

        print(f"Movement completed. Current floor: {self.current_floor}")
        print(f"Final position: {self.current_position:.2f}m")
        print(
            f"Final error: {abs(self.target_position - self.current_position) * 1000:.1f}mm"
        )

    def _get_nearest_floor(self) -> str:
        min_distance = float("inf")
        nearest_floor = "terreo"

        for floor_name, position in self.controller.floor_positions.items():
            distance = abs(position - self.current_position)
            if distance < min_distance:
                min_distance = distance
                nearest_floor = floor_name

        return nearest_floor

    def _publish_position_update(self, data: dict):
        try:
            self.client.publish(self.topics["position_update"], json.dumps(data))
        except Exception as e:
            print(f"Error publishing position update: {e}")

    def _publish_status_update(self):
        try:
            status_data = {
                "timestamp": time.time(),
                "current_floor": self.current_floor,
                "target_floor": self.target_floor,
                "is_moving": self.is_moving,
                "direction": "up"
                if self.direction > 0
                else ("down" if self.direction < 0 else "stopped"),
                "current_position": self.current_position,
            }

            self.client.publish(self.topics["status_update"], json.dumps(status_data))

            if self.status_callback:
                self.status_callback(status_data)

        except Exception as e:
            print(f"Error publishing status update: {e}")

    def get_current_status(self) -> dict:
        return {
            "current_floor": self.current_floor,
            "current_position": self.current_position,
            "target_floor": self.target_floor,
            "target_position": self.target_position,
            "is_moving": self.is_moving,
            "direction": "up"
            if self.direction > 0
            else ("down" if self.direction < 0 else "stopped"),
        }

    def set_position_callback(self, callback: Callable):
        self.position_callback = callback

    def set_status_callback(self, callback: Callable):
        self.status_callback = callback
