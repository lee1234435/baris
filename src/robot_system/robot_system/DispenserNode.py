import time
import rclpy as rp
from library.Constants import Constants, DispenseCommand, Service
from message.srv import DispenseService
from rclpy.node import Node
from robot_system.DIODispenser import DIODispenser


class DispenserNode(Node):
    """
        커피 DP, water DP 노드 클래스
    """

    # 서비스 체크 시간
    CHECK_INTERVAL_SEC = 0.1

    def __init__(self):

        super().__init__(Constants.DISPENSER_NODE)

        # 아카데미 서비스
        self.xyz_dispenser_service = self.create_service(DispenseService, Service.SERVICE_DISPENSER,
                                                         self.xyz_callback_service)
        # 디스펜서 생성
        self.dispenser = DIODispenser()

        self.get_logger().info('Dispenser Node Init!!')

    def xyz_callback_service(self, request, response):
        self.get_logger().info(f' Request Cmd : {request.command}')
        if request.command == DispenseCommand.COFFEE_ON:
            response = self.dispenser.dispense(DispenseCommand.COFFEE_ON)
            time.sleep(0.05)

            response = self.dispenser.dispense(DispenseCommand.COFFEE_PIN_RESET)
            time.sleep(0.5)

            response = self.dispenser.dispense(DispenseCommand.COFFEE_OFF)
            time.sleep(0.05)

            response = self.dispenser.dispense(DispenseCommand.COFFEE_PIN_RESET)
    

        elif request.command == DispenseCommand.WATER_TOGGLE:
            response = self.dispenser.dispense(DispenseCommand.WATER_TOGGLE)
            time.sleep(0.05)

            response = self.dispenser.dispense(DispenseCommand.WATER_PIN_RESET)
            time.sleep(9.0)

            response = self.dispenser.dispense(DispenseCommand.WATER_TOGGLE)
            time.sleep(0.05)

            response = self.dispenser.dispense(DispenseCommand.WATER_PIN_RESET)
        else:
            response = DispenseService.Response()
            response.seq_no = ''
            response.response_cd = 9999

        return response


def main(args=None):

    rp.init(args=args)

    dispenserNode = DispenserNode()
    rp.spin(dispenserNode)

    dispenserNode.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
