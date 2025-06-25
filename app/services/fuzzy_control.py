import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from typing import Tuple
import time


class ElevatorFuzzyController:
    def __init__(self):
        self.sampling_time = 0.2
        self.startup_duration = 2.0
        self.startup_max_power = 31.5
        self.floor_positions = self._get_floor_positions()
        self._setup_fuzzy_system()

    def _get_floor_positions(self) -> dict:
        position = {
            "ground": 4.0,
            "floor_1": 8.0,
            "floor_2": 11.0,
            "floor_3": 14.0,
            "floor_4": 17.0,
            "floor_5": 20.0,
            "floor_6": 23.0,
            "floor_7": 26.0,
            "floor_8": 29.0,
        }
        return position

    def _setup_fuzzy_system(self):
        self.error = ctrl.Antecedent(np.arange(0, 37, 0.1), "error")
        self.delta_error = ctrl.Antecedent(np.arange(-0.3, 0.3, 0.01), "delta_error")
        self.motor_power = ctrl.Consequent(np.arange(0, 101, 1), "motor_power")

        self.error["zero"] = fuzz.trimf(self.error.universe, [0, 0, 0.1])
        self.error["small"] = fuzz.trimf(self.error.universe, [0.05, 1.5, 3.0])
        self.error["medium"] = fuzz.trimf(self.error.universe, [2.5, 4.5, 6.0])
        self.error["large"] = fuzz.trimf(self.error.universe, [5.0, 10.5, 18.0])
        self.error["very_large"] = fuzz.trimf(self.error.universe, [15.0, 25.5, 36.0])

        self.delta_error["decreasing_fast"] = fuzz.trimf(
            self.delta_error.universe, [-0.3, -0.2, -0.1]
        )
        self.delta_error["decreasing_slow"] = fuzz.trimf(
            self.delta_error.universe, [-0.15, -0.05, -0.01]
        )
        self.delta_error["stable"] = fuzz.trimf(
            self.delta_error.universe, [-0.02, 0, 0.02]
        )
        self.delta_error["increasing"] = fuzz.trimf(
            self.delta_error.universe, [0.01, 0.15, 0.3]
        )

        self.motor_power["zero"] = fuzz.trimf(self.motor_power.universe, [0, 0, 5])
        self.motor_power["low"] = fuzz.trimf(self.motor_power.universe, [4, 25, 40])
        self.motor_power["medium"] = fuzz.trimf(self.motor_power.universe, [35, 50, 65])
        self.motor_power["high"] = fuzz.trimf(self.motor_power.universe, [60, 75, 85])
        self.motor_power["maximum"] = fuzz.trimf(
            self.motor_power.universe, [80, 90, 95]
        )

        self._setup_fuzzy_rules()

        self.control_system = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.control_system)

    def _setup_fuzzy_rules(self):
        self.rules = [
            ctrl.Rule(self.error["very_large"], self.motor_power["maximum"]),
            ctrl.Rule(self.error["large"], self.motor_power["high"]),
            ctrl.Rule(self.error["medium"], self.motor_power["medium"]),
            ctrl.Rule(
                self.error["small"] & self.delta_error["decreasing_fast"],
                self.motor_power["medium"],
            ),
            ctrl.Rule(
                self.error["small"] & self.delta_error["decreasing_slow"],
                self.motor_power["low"],
            ),
            ctrl.Rule(
                self.error["small"] & self.delta_error["stable"],
                self.motor_power["low"],
            ),
            ctrl.Rule(self.error["zero"], self.motor_power["zero"]),
            ctrl.Rule(self.delta_error["increasing"], self.motor_power["zero"]),
        ]

    def get_floor_position(self, floor_name: str) -> float:
        if floor_name in self.floor_positions:
            return self.floor_positions[floor_name]
        else:
            raise ValueError(f"Unknown floor: {floor_name}")

    def compute_startup_power(self, elapsed_time: float) -> float:
        if elapsed_time >= self.startup_duration:
            return None
        startup_power = (self.startup_max_power / self.startup_duration) * elapsed_time
        return startup_power

    def compute_control(
        self, current_position: float, target_position: float, previous_error: float
    ) -> Tuple[float, float]:
        current_error = target_position - current_position
        error_magnitude = abs(current_error)
        delta_abs_error = abs(current_error) - abs(previous_error)

        try:
            self.simulation.input["error"] = error_magnitude
            self.simulation.input["delta_error"] = delta_abs_error
            self.simulation.compute()
            motor_power = self.simulation.output["motor_power"]
        except Exception:
            motor_power = 0.0

        motor_power = min(90.0, motor_power)
        return motor_power, current_error

    def update_position(
        self,
        current_position: float,
        motor_power_percent: float,
        direction: int,
        elapsed_time: float = None,
    ) -> float:
        k1 = 1 if direction > 0 else -1
        motor_power_fraction = motor_power_percent / 100.0

        if elapsed_time is not None and elapsed_time <= 2.0:
            new_position_raw = (
                k1 * current_position * 0.999 + motor_power_fraction * 0.251287
            )
        else:
            new_position_raw = (
                k1 * current_position * 0.9995 + motor_power_fraction * 0.212312
            )

        new_position = abs(new_position_raw)
        return new_position

    def simulate_movement(
        self, start_floor: str, target_floor: str, max_time: float = 80.0
    ) -> dict:
        start_position = self.get_floor_position(start_floor)
        target_position = self.get_floor_position(target_floor)

        direction = 1 if target_position > start_position else -1
        current_position = start_position
        previous_error = target_position - start_position

        time_data = []
        position_data = []
        error_data = []
        motor_power_data = []
        simulation_time = 0.0

        distance = abs(target_position - start_position)
        if distance >= 20:
            tolerance = 0.05
        elif distance >= 10:
            tolerance = 0.04
        else:
            tolerance = 0.03

        max_iterations = int(max_time / self.sampling_time)

        for i in range(max_iterations):
            current_error = target_position - current_position
            if abs(current_error) <= tolerance:
                break

            startup_power = self.compute_startup_power(simulation_time)
            if startup_power is not None:
                motor_power = startup_power
            else:
                motor_power, _ = self.compute_control(
                    current_position, target_position, previous_error
                )

            current_position = self.update_position(
                current_position, motor_power, direction, simulation_time
            )

            time_data.append(simulation_time)
            position_data.append(current_position)
            error_data.append(current_error)
            motor_power_data.append(motor_power)

            previous_error = current_error
            simulation_time += self.sampling_time

        final_error_mm = abs(target_position - current_position) * 1000

        if not position_data:
            peak_position = start_position
        elif direction > 0:
            peak_position = max(position_data)
        else:
            peak_position = min(position_data)

        overshoot = abs(peak_position - target_position)
        travel_distance = abs(target_position - start_position)
        overshoot_percent = (
            (overshoot / travel_distance) * 100 if travel_distance > 0 else 0
        )

        return {
            "time": time_data,
            "position": position_data,
            "error": error_data,
            "motor_power": motor_power_data,
            "final_time": simulation_time,
            "final_error_mm": final_error_mm,
            "peak_position": peak_position,
            "overshoot_percent": overshoot_percent,
            "start_position": start_position,
            "target_position": target_position,
            "direction": "Subida" if direction > 0 else "Descida",
        }
