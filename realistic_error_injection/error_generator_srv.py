from error_injection_interfaces.srv import ObtainError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np 
from kincalib.Learning.NoiseGenerator import NetworkNoiseGenerator
from realistic_error_injection.utils import load_noise_generator, get_path_to_torch_model

def msg2numpy(msg: JointState)->np.ndarray:
    return np.array([msg.position])

def numpy2msg(arr: np.ndarray)->JointState:
    joint_state_msg = JointState()
    
    joint_state_msg.position = arr.tolist() 
    return JointState(position=arr) 

class ErrorGeneratorService(Node):

    def __init__(self):
        super().__init__('error_generator_service')

        torch_model_path = get_path_to_torch_model()
        self.noise_generator, self.cfg = load_noise_generator(torch_model_path)

        self.srv = self.create_service(ObtainError, 'ObtainError', self.obtain_error_callback)


    def obtain_error_callback(self, request, response):
        '''
        Service definition for reference

        request.current_js: JointState
        request.previous_js: JointState
        response.error: JointState 
        response.success: bool
        ''' 

        previous_jp = [[0.2, 0.2, 0.125, 0.2, 0.2, 0.2]]
        sample_jp = [[0.2, 0.2, 0.125, 0.2, 0.2, 0.2]]
        corrupted_jp, offset = self.noise_generator.corrupt_jp_batch(sample_jp, previous_jp, self.cfg)

        offset = offset[0] # remove batch dimension

        self.get_logger().info(f'Incoming request\n: Output offset{offset}')
        response.joint_error = numpy2msg(offset)
        response.success = True

        return response


def main():
    rclpy.init()

    minimal_service = ErrorGeneratorService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()