import datetime
import traceback

import rclpy as rp
from library.Constants import Service, Topic, Constants, DeviceCode, DeviceStatus, ResponseCode
from message.msg import DispenserStatus, ComponentStatus
from message.srv import RobotService
from rclpy.node import Node
from rclpy.qos import QoSProfile
from robot_system.RobotSystem import RobotSystem


class RobotSystemNode(Node):
    GRIPPER_INIT = 'gripper_init'
    SHAKE = 'shake'
    HOME = 'home'
    RAIL_BASE = 'rail_base'

    def __init__(self):
        super().__init__(Constants.ROBOTS_SYSTEM)

        # 로거 설정

        qos_profile = QoSProfile(depth=Constants.QOS_DEFAULT)

        # 로봇 시스템 요청 서비스
        self.service = self.create_service(RobotService, Service.SERVICE_ROBOT, self.callback_robot_service)

        # 로봇 시스템 상태 정보 송신
        self.publisher = self.create_publisher(DispenserStatus, Topic.ROBOT_STATUS, qos_profile=qos_profile)

        # 로봇 시스템 토픽 타이머 변수
        self.timer = self.create_timer(timer_period_sec=Constants.TIMER_PERIOD, callback=self.timer_execute)

        self.robot_system = RobotSystem()
        self.node_status = DeviceStatus.STANDBY

    def set_status(self, status):
        self.node_status = status
        return self.node_status

    def get_status(self):
        return self.node_status

    def callback_robot_service(self, request, response):
        """
        로봇 서비스 요청 받으면 실행되는 메소드
        :param request : 서비스 콜을 받은 메세지
        :param response : 서비스 콜을 처리 후 전달할 메세지
        :return 서비스 응답
        """
        try:

            return_value = self.robot_system.execute(request=request)
            response.seq_no = request.seq_no
            response.component_cd = return_value.component_cd
            response.response_cd = return_value.response_cd
            response.status_cd = return_value.status_cd
            response.result = return_value.result

        except Exception as error:
            print(f"RobotSystemNode callback_robot_service {error=}, {type(error)=}")

            response.seq_no = request.seq_no
            response.component_cd = DeviceCode.ROBOT
            response.response_cd = ResponseCode.ERROR
            response.status_cd = DeviceStatus.ERROR
            response.result = request.cmd

        finally:
            return response

    def timer_execute(self):
        """
        일정 주기로 실행되는 메소드(상태 토픽 발행용)
        """
        response = None
        component_list = []
        try:
            # Msg 변수 선언
            if self.robot_system.request_cnt > 15:
                self.get_logger().info(f"DRIP DONE")

            topic_msg = DispenserStatus()
            self.get_logger().info(f"Current Cmd {self.robot_system.get_cur_cmd()}")

            topic_msg.seq_no = str(datetime.datetime.now())
            component_list.append(self.component_status())

            topic_msg.component = component_list
            topic_msg.node_status = self.robot_system.get_cur_cmd()

            self.publisher.publish(topic_msg)

        except Exception as error:
            print(f"RobotSystemNode timer_execute {error=}, {type(error)=}")

    def component_status(self):
        status_msg = ComponentStatus()
        try:
            status_msg.status = DeviceStatus.STANDBY
            status_msg.status_code = ResponseCode.SUCCESS
            status_msg.stock = Constants.ZERO

        except Exception as error:
            print(f"RobotSystemNode component_status {error=}, {type(error)=}")
            self.get_logger().error(traceback.format_exc())
            status_msg.status = DeviceStatus.ERROR
            status_msg.status_code = ResponseCode.ERROR
            status_msg.stock = Constants.ZERO
        finally:
            return status_msg


def main(args=None):
    """
    Node 실행 메인 함수
    """
    rp.init(args=args)
    node = RobotSystemNode()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')

    except SystemExit:  # <--- process the exception
        print("RobotSystemNode System Exiting")

        topic_msg = DispenserStatus()
        component_msg = ComponentStatus()
        component_msg.status = DeviceStatus.OFF
        component_msg.stock = Constants.ZERO
        component_msg.status_code = ResponseCode.ROB_STATE_OFF.value
        topic_msg.node_status = node.get_status()
        topic_msg.component = [component_msg]
        topic_msg.seq_no = str(datetime.datetime.now())

        node.publisher.publish(topic_msg)

    except Exception as error:
        print(f"RobotSystemNode main {error=}, {type(error)=}")
        print(traceback.format_exc())
    finally:
        print("RobotSystemNode System Deinit")
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
