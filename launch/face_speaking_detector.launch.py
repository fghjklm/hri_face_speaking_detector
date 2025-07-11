# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_pal import get_pal_configuration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = 'hri_face_speaking_detector'
    node = 'hri_face_speaking_detector'
    ld = LaunchDescription()
    config = get_pal_configuration(pkg=pkg, node=node, ld=ld)

    face_speaking_detector_node = LifecycleNode(
        package=pkg,
        executable='hri_face_speaking_detector',
        namespace='',
        name=node,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both', emulate_tty=True,
        # prefix="xterm -e ",
    )

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(face_speaking_detector_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=face_speaking_detector_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(face_speaking_detector_node),
            transition_id=Transition.TRANSITION_ACTIVATE))], handle_once=True))

    ld.add_action(face_speaking_detector_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    return ld
