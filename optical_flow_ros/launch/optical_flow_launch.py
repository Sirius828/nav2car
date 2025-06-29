# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    ld = LaunchDescription()
    
    optical_flow_params_path = PathJoinSubstitution(
        [FindPackageShare("optical_flow_ros"), "config", "sensor_params.yaml"])
           
    optical_flow_node = LifecycleNode(
        package='optical_flow_ros',
        executable='optical_flow_publisher',
        name='optical_flow',
        namespace='', # Do not change, else config params and remappings need to be updated
        output='screen',
        parameters=[optical_flow_params_path],
        remappings=[('odom', 'flow_odom')])

    # 配置事件 - 延迟2秒启动以确保节点完全初始化
    emit_configure_event = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(optical_flow_node),
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            )
        ]
    )

    # 激活事件处理器 - 当节点配置成功后自动激活
    register_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=optical_flow_node, 
            start_state='configuring', 
            goal_state='inactive',
            entities=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(optical_flow_node),
                                transition_id=Transition.TRANSITION_ACTIVATE
                            )
                        )
                    ]
                )
            ]
        )
    )

    ld.add_action(optical_flow_node)
    ld.add_action(emit_configure_event)
    ld.add_action(register_activate_handler)

    return ld
