import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/georgekouretas/ros2_ws/src/exercise_decoder_node/install/exercise_decoder_node'
