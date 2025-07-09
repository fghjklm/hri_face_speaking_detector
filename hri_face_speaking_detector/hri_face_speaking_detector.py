#! /usr/bin/python3
# -*- coding: utf-8 -*-

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

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from hri import HRIListener
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.lifecycle import Node, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.msg import State
from functools import partial
from hri_msgs.msg import FacialLandmarks
from std_msgs.msg import Bool
import pickle
import os
import cv2
from ament_index_python.packages import get_package_share_directory
from sklearn.ensemble import RandomForestClassifier # type: ignore

# Drawing parameters definition
PASTEL_YELLOW = (174, 239, 238)
BLACK = (0, 0, 0)
TEXT_BLACK = (0, 0, 0, 255)
BOX_THICKNESS = 3
THICKNESS_CORNERS = 2
LABEL_DISTANCE = 50
LABEL_WIDTH = 80
LABEL_HEIGHT = 30
SPACE_PER_CHARACTER = 14
LABEL_LINE_THICKNESS = 1
JOINT_RADIUS = 10
FILLED = -1


class NodeFaceSpeakingDetector(Node):
    def __init__(self):
        super().__init__('hri_face_speaking_detector')

        self.last_prediction = {}

        self.declare_parameter(
            'sequence_length',
            16,
            ParameterDescriptor(
                description='length of the sequence of images classified')
        )

        # Start publishing diagnostics
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 1)
        self.diag_timer = self.create_timer(1., self.publish_diagnostics)

        self.current_diagnostics_status = DiagnosticStatus.WARN
        self.current_diagnostics_message = 'Face speaking detector is unconfigured'
        self.get_logger().info('State: Unconfigured.')

    def __del__(self):
        state = self._state_machine.current_state
        self.on_shutdown(LifecycleState(state_id=state[0], label=state[1]))

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.internal_cleanup()

        self.get_logger().info('State: Unconfigured.')
        return super().on_cleanup(state)
    
    def load_classifier(self):

        classifier_path =  os.path.join(
            get_package_share_directory("hri_face_speaking_detector"),
            "models/RF_16_1.pkl")
        try:
            with open(classifier_path, 'rb') as f:
                self.classifier = pickle.load(f)
        except Exception as e:
            # Log the error and handle it gracefully
            self.get_logger().error(
                f"Failed to load classifier from {classifier_path}: {e}")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:

        self.load_classifier()



        self.SEQUENCE_LENGTH = int(self.get_parameter("sequence_length").value) # type: ignore
        
        
        self.current_diagnostics_status = DiagnosticStatus.WARN
        self.current_diagnostics_message = (
            'Face speaking detector is configured, yet inactive '
            '(lifecycle: inactive).'
        )
        self.get_logger().info('State: Inactive.')

        return super().on_configure(state)

    def internal_cleanup(self):

        self.destroy_timer(self.diag_timer)
        self.destroy_publisher(self.diag_pub)

    def internal_deactivate(self):

        for face_id in self.face_publishers:
            self.destroy_publisher(self.face_publishers[face_id])
        for face_id in self.faces_subscribed:
            self.destroy_subscription(self.faces_subscribed[face_id])

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.internal_deactivate()

        self.current_diagnostics_status = DiagnosticStatus.WARN
        self.current_diagnostics_message = \
            'Face speaking detector is configured, yet inactive (lifecycle: inactive).'

        self.get_logger().info('State: Inactive.')
        return super().on_deactivate(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:

        self.hri_listener = HRIListener('Face_speaking_hri_detector')

        self.landmarks_indexes = []

        self.faces_subscribed = {}
        self.faces_landmarks = {}
        self.face_publishers = {}


        self.on_face_listener = self.hri_listener.on_face(self.on_face_callback)

        for face_id, face in self.hri_listener.faces.items():
            self.faces_subscribed[face_id] = self.create_subscription(
                                        FacialLandmarks, 
                                        f'humans/faces/{face_id}/landmarks',
                                        partial(self.landmarks_callback, face_id=face_id),
                                        qos_profile=qos_profile_sensor_data)
            self.faces_landmarks[face_id] = []
        self.on_face_lost_listener = self.hri_listener.on_face_lost(self.on_face_lost_callback)
        self.current_diagnostics_status = DiagnosticStatus.OK
        self.current_diagnostics_message = \
            'Face speaking detector is active'
        self.get_logger().info('State: Active.')
        return super().on_activate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        if state.state_id == State.PRIMARY_STATE_ACTIVE:
            self.internal_deactivate()
        if state.state_id in [State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE]:
            self.internal_cleanup()
        self.get_logger().info('State: Finalized.')
        return super().on_shutdown(state)

    
    def on_face_callback(self, face):
        face_id = face.id
        subscription = self.create_subscription(
                                        FacialLandmarks, 
                                        f'humans/faces/{face_id}/landmarks',
                                        partial(self.landmarks_callback, face_id=face_id),
                                        qos_profile=qos_profile_sensor_data)
        self.faces_subscribed[face_id] = subscription
        self.faces_landmarks[face_id] = []

    def landmarks_callback(self, Faciallandmarks : FacialLandmarks, face_id):
        
        self.faces_landmarks[face_id].append(Faciallandmarks.landmarks)
        
        if len(self.faces_landmarks[face_id])>= self.SEQUENCE_LENGTH:

            sequence = self.faces_landmarks[face_id][-(self.SEQUENCE_LENGTH):]
            if len(self.faces_landmarks[face_id]) > self.SEQUENCE_LENGTH:
                self.faces_landmarks[face_id].pop(0)
                
            self.is_speaking(face_id, sequence)

            

    def process_landmarks(self, input):
        
        landmarks_list = []
        for landmarks in input:
            for i in range(48, 68):  # only using the 20 landmarks around the mouth from the 70 broadcasted by hri_face_detect
                landmarks_list.append(landmarks[i].x)
                landmarks_list.append(landmarks[i].y)

        return [landmarks_list]

    def is_speaking(self, face_id, input):

        input = self.process_landmarks(input)
        predict = self.classifier.predict_proba(input)
        label, confidence = np.argmax(predict[0]), np.max(predict[0]) #confidence is processed but not used in this case
        self.last_prediction[face_id] = label
        self.publish_is_speaking(face_id, label)


    def on_face_lost_callback(self, face_id):

        if face_id in self.faces_subscribed:
            self.destroy_subscription(self.faces_subscribed[face_id])
            del self.faces_subscribed[face_id]
        if face_id in self.faces_landmarks:
            del self.faces_landmarks[face_id]
        self.last_prediction.pop(face_id, None)

        if face_id in self.face_publishers:
            self.destroy_subscription(self.face_publishers[face_id])
            del self.face_publishers[face_id]

    def publish_is_speaking(self, face_id, label):
        try:
            if face_id not in self.face_publishers:
                self.face_publishers[face_id] = self.create_publisher(
                    Bool, f'/humans/faces/{face_id}/is_speaking', 10)

            msg = Bool()
            msg.data = bool(label)

            self.face_publishers[face_id].publish(msg)

        except Exception as e:
            # Log the error and handle it gracefully
            self.get_logger().error(
                f"Failed to publish label for face_id {face_id}: {e}")

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=self.current_diagnostics_status,
            name='/social_perception/faces/hri_face_speaking_detector',
            message=self.current_diagnostics_message,
            values=[
                KeyValue(key="Module name", value="hri_face_speaking_detector"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
                KeyValue(key="Last recognised label",
                         value="; ".join(
                             f"face <{face_id}>: {label}"
                             for face_id, label
                             in self.last_prediction.items())),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self.diag_pub.publish(arr)



def main(args=None):
    rclpy.init(args=args)
    node = NodeFaceSpeakingDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
