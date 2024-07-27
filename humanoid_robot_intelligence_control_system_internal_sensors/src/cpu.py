#!/usr/bin/env python3

# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import time
from typing import Dict, List, Tuple

class PIDController:
    def __init__(self, gains: Tuple[float, float, float], name: str, output_limits: Tuple[float, float] = (-float('inf'), float('inf'))):
        self.kp, self.ki, self.kd = gains
        self.name = name
        self.output_limits = output_limits
        self.reset()

    def reset(self):
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint: float, process_variable: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        error = setpoint - process_variable
        
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.output_limits[1]), self.output_limits[0])

        self.last_error = error
        self.last_time = current_time

        return output

    def update_config(self, gains: Tuple[float, float, float]):
        self.kp, self.ki, self.kd = gains
