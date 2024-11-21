# /*
#  * Copyright 2024 Myeong Jin Lee
#  *
#  * Licensed under the Apache License, Version 2.0 (the "License");
#  * you may not use this file except in compliance with the License.
#  * You may obtain a copy of the License at
#  *
#  *     http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing, software
#  * distributed under the License is distributed on an "AS IS" BASIS,
#  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  * See the License for the specific language governing permissions and
#  * limitations under the License.
#  */
 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    e2box_imu_node = Node(
        package='e2box_imu', 
        executable='e2box_imu', 
        name='e2box_imu_node',  
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port_name': '/dev/ttyUSB0',
            'baudrate': 115200,
            'loop_rate': 100,
            'angular_velocity_threshold': 0.3,
            'linear_acceleration_threshold': 2.0
        }]
    )

    return LaunchDescription([e2box_imu_node])