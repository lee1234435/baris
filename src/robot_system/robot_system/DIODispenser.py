from message.srv import DispenseService
from library.Constants import ResponseDataList, ResponseCode, DeviceStatus, DeviceCode
import datetime

class DIODispenser():

    """ DIO 관련 처리 디스펜서 """

    def __init__(self):
        """
        클래스 생성자
        """
        self.dio_component = None
        self.status = DeviceStatus.STANDBY
        self.before_cmd = ""

    def get_before_cmd(self):
        return self.before_cmd

    def dispense(self, command):
        """
           디스펜싱  처리 메소드
        """
        response = DispenseService.Response()
        self.before_cmd = command
        
        response.seq_no = str(datetime.datetime.now())
        response.response_cd = ResponseCode.SUCCESS

        return response
