from error_injection_interfaces.srv import ObtainError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np 
from kincalib.Learning.NoiseGenerator import NetworkNoiseGenerator
from realistic_error_injection.utils import load_noise_generator, get_path_to_torch_model
from realistic_error_injection.utils import numpy2msg, msg2numpy


class ErrorGeneratorService(Node):

    def __init__(self):
        super().__init__('error_generator_service')

        torch_model_path = get_path_to_torch_model()
        self.noise_generator, self.cfg = load_noise_generator(torch_model_path)

        self.srv = self.create_service(ObtainError, 'ObtainError', self.obtain_error_callback)


    def obtain_error_callback(self, request: ObtainError.Request, response: ObtainError.Response):
        '''
        Service definition for reference

        request.current_js: JointState
        request.previous_js: JointState
        response.error: JointState 
        response.success: bool
        ''' 

        # previous_jp = [[0.2, 0.2, 0.125, 0.2, 0.2, 0.2]]
        # sample_jp = [[0.2, 0.2, 0.125, 0.2, 0.2, 0.2]]

        previous_js = msg2numpy(request.previous_js)
        current_js = msg2numpy(request.current_js)

        # Add batch dimension
        previous_js = np.expand_dims(previous_js, axis=0)
        current_js =  np.expand_dims(current_js, axis=0)

        corrupted_js, offset = self.noise_generator.corrupt_jp_batch(current_js, previous_js, self.cfg)

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