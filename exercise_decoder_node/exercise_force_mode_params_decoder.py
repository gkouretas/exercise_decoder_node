import threading
import numpy as np
import rclpy
from rclpy.node import Node

from fatigue_classifier.fatigue_classifier_configs import (
    FATIGUE_OUTPUT_TOPIC_NAME, FATIGUE_OUTPUT_QOS
)

from idl_definitions.msg import FatigueState
from ur10e_custom_control.ur10e_typedefs import URService

from ur_msgs.srv import (
    SetForceModeParams
)

_USE_PREDICTED = False

# Lower/upper bound damping factors
# At lower bound fatigue (0%), damping should be at the maximum,
# and at upper bound fatigue (100%), damping should be at the minimum
_DAMPING_LOWER_BOUND = 0.5
_DAMPING_UPPER_BOUND = 0.01

def map(x, in_min: float, in_max: float, out_min: float, out_max: float):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class ExerciseForceModeParamsDecoder:
    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.RLock()
        self._fatigue_monitor = self._node.create_subscription(
            FatigueState,
            FATIGUE_OUTPUT_TOPIC_NAME,
            self.update_force_mode_params,
            FATIGUE_OUTPUT_QOS
        )

        self._mapped_fatigue = _DAMPING_UPPER_BOUND

        self._signal = threading.Event()
        self._signal.clear()

        self._force_mode_params_service = self._node.create_client(
            srv_type=URService.get_service_type(
                URService.IOAndStatusController.SRV_SET_FORCE_MODE_PARAMS
            ),
            srv_name=URService.IOAndStatusController.SRV_SET_FORCE_MODE_PARAMS.value
        )

        # Run service thread
        threading.Thread(target=self.force_mode_params_service_thread, daemon=True).start()

    def force_mode_params_service_thread(self):
        while True:
            self._signal.wait()

            # Get the damping factor to use
            factor = self._mapped_fatigue
            
            self._signal.clear()

            try:
                self._node.get_logger().info(f"Sending {factor}")

                response = self._force_mode_params_service.call(
                    request=SetForceModeParams.Request(damping_factor=factor)
                )

                if not response.success:
                    self._node.get_logger().error("Resposne from sending force mode params unsuccessful")
            except Exception as e:
                self._node.get_logger().fatal(f"Failed to send force mode params: [exc: {e}]")


    def update_force_mode_params(self, msg: FatigueState):
        if _USE_PREDICTED:
            fatigue = msg.predicted
        else:
            fatigue = msg.actual

        self._mapped_fatigue = map(
            fatigue,
            0.0,
            1.0,
            _DAMPING_LOWER_BOUND,
            _DAMPING_UPPER_BOUND
        )

        self._node.get_logger().info(f"Fatigue mapping: {fatigue} -> {self._mapped_fatigue}")

        # Set the signal as ready
        self._signal.set()

def main():
    rclpy.init()
    
    node = Node("decoder_node")
    force_mode_params_decoder = ExerciseForceModeParamsDecoder(node)

    rclpy.spin(node)