import rclpy
from rclpy.node import Node
from rclpy.client import SrvTypeResponse

from idl_definitions.msg import (
    MindroveArmBandEightChannelMsg,
    UserInputMsg
)

from ros2_mindrove.mindrove_configs import MINDROVE_ROS_TOPIC_NAME
from pi_user_input_node.user_input_node_config import USER_INPUT_TOPIC_NAME, USER_INPUT_QOS_PROFILE
from ur10e_custom_control.ur10e_typedefs import URService
from ur_msgs.srv._set_speed_slider_fraction import SetSpeedSliderFraction_Request

import threading
import numpy as np

from collections import deque

class SimpleSpeedDecoder:
    def __init__(self, node: Node, refresh_rate: float):
        self._node = node
        self._lock = threading.RLock()
        self._mindrove_subscriber = self._node.create_subscription(
            MindroveArmBandEightChannelMsg,
            MINDROVE_ROS_TOPIC_NAME,
            self.update_armband_info,
            0
        )

        # For storing last requested scaling factor, to avoid overwhelming the
        # service.
        #
        # TODO: only publish changes or decimate service? it is not viable to
        # publish service at high frequency (i.e. 100 Hz)
        self._last_scaling_factor = -1

        # Setup raw and filtered buffers
        # Use 100 data points, because we will be performing a LPF of 5 Hz
        # for filtering the data, which is effectively equivalent to averaging
        # a buffer of length=100.
        self._raw_emg_data = [
            deque(maxlen = 100) for _ in range(8)
        ] # using 125 data-points

        self._filt_emg_data = [
            deque(maxlen = 100) for _ in range(8)
        ] # using 125 data-points

        # FFC offset
        self._ffc_offset = -(500 // 60)

        # TODO: tune this...
        self._emg_activation_threshold = 0.05

        # Initialize fatigue input to zero
        self._fatigue_input: int = 0

        self._user_input_subscriber = self._node.create_subscription(
            UserInputMsg,
            USER_INPUT_TOPIC_NAME,
            self.update_user_info,
            USER_INPUT_QOS_PROFILE
        )

        self._ur_speed_scaling_service = URService.init_service(
            self._node,
            URService.IOAndStatusController.SRV_SET_SPEED_SLIDER_FRACTION,
            timeout = 10
        )

        self._timer = self._node.create_timer(
            refresh_rate,
            self.run_decode
        )

    @property
    def is_emg_data_present(self):
        with self._lock:
            _ret = len(self._raw_emg_data[0]) > 0

        return _ret

    def update_armband_info(self, msg: MindroveArmBandEightChannelMsg):
        with self._lock:
            for i in range(8):
                # Iterate across 8 EMG channels

                for dp in msg.__getattribute__(f"c{i+1}"):
                    # Get data points from each channel

                    # Apply FFC filter + full-wave rectification
                    # The low-pass filter (moving average) will be 
                    # applied at the point of analysis
                    self._filt_emg_data[i].append(
                        abs(dp - (self._raw_emg_data[-self._ffc_offset] if self._ffc_offset <= len(self._raw_emg_data) else 0.0))
                    )

                    self._raw_emg_data[i].append(dp)

    def update_user_info(self, msg: UserInputMsg):
        with self._lock:
            self._fatigue_input = msg.fatigue_percentage

    def run_decode(self):
        # Initialize on/off factor
        factor = 0

        if self.is_emg_data_present:
            avg_magnitude = np.average([x for x in np.average(self._filt_emg_data)])
            if avg_magnitude >= self._emg_activation_threshold:
                # If we are above the desired threshold, activate the fatigue input
                factor = 1
        else:
            # If there is no armband active, just always set the on/off factor to "on",
            # in order to remove the dependency of the armband
            factor = 1
        
        # Compute scaling factor. Currently is simply a function of manual user fatigue input
        # and if there is muscle activation detected.
        _scaling_factor = float(factor * 0.01 * self._fatigue_input)

        if abs(_scaling_factor - self._last_scaling_factor) > 1e-5:
            # If a new scaling factor has been sent from user input node, request the
            # speed scaling service
            self._node.get_logger().info(f"Updating scaling to {_scaling_factor}")
            self._last_scaling_factor = _scaling_factor

            # Populate request
            request: SetSpeedSliderFraction_Request = URService.get_service_type(
                URService.IOAndStatusController.SRV_SET_SPEED_SLIDER_FRACTION
            ).Request()
            request.speed_slider_fraction = _scaling_factor
            
            # Send request asynchronously
            future = self._ur_speed_scaling_service.call_async(request)
            future.add_done_callback(self._speed_scaling_callback)

    def _speed_scaling_callback(self, response: SrvTypeResponse):
        self._node.get_logger().debug(f"Future resp: {response}")

def main():
    rclpy.init()
    
    node = Node("decoder_node")
    speed_decoder = SimpleSpeedDecoder(node, refresh_rate = 0.01)

    rclpy.spin(node)

if __name__ == "__main__":
    main()