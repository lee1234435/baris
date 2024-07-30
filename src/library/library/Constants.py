class Command:
    """ 커맨드 관련 상수  """
    INIT = "INIT"
    RUN = 'RUN'
    STOP = 'STOP'
    OFF = 'OFF'
    SHUTDOWN = 'SHUTDOWN'
    RESTART = 'RESTART'

    GET_STATUS = "get_status"
    CLEAN = "clean"
    CLEAN_ALL = "clean_all"
    CLEAN_CHANNEL = 'clean_channel'
    DISPENSE = "dispense"
    TEST = "test"
    EXTRACT = "extract"
    HOT_WATER = "hot_water"
    SET_STOCK_MIN = "set_stock_min"
    SET_STOCK_MAX = "set_stock_max"
    GET_MENU = 'get_menu'
    WATER = 'water'
    ICE = 'ice'
    GET_STATUS_MODULE = 'get_status_module'
    GET_INFO = 'get_info'
    RAIL_BASE = 'rail_base'
    REBOOT = 'reboot'
    PRESET = 'preset'
    CLOSE = 'close'
    UNLOCK_PROTECT = 'unlock_protect'
    GRIPPER_INIT = 'gripper_init'
    HOME = 'home'
    PLACE_SYRUP = 'place_syrup'
    PICKUP_SYRUP = 'pickup_syrup'
    TEACH = 'TEACH'
    ENDTEACH = 'ENDTEACH'
    RESET = 'reset'


class RobotParameter:
    ZERO = '0'
    ONE = '1'
    TWO = '2'
    DSP = 'DSP'
    ZON = 'ZON'
    KET = 'KET'
    DPO = 'DPO'
    PIC = 'PIC'
    ETC = 'ETC'
    HOT = 'HOT'
    SOL = 'SOL'
    BIN = 'BIN'
    COF = 'COF'


class RobotCommand:
    DRAIN_ALL = 'DRAIN_ALL'
    DRAIN_FIT = 'DRAIN_FIT'
    DRIP = 'DRIP'
    FLATTENING = 'FLATTENING'
    GESTURE = 'GESTURE'
    HOLD = 'HOLD'
    HOME_KETTLE = 'HOME_KETTLE'
    PLACE = 'PLACE'
    PLACE_CUP = 'PLACE_CUP'
    PLACE_DRIP = 'PLACE_DRIP'
    UNHOLD = 'UNHOLD'
    GRIPPER_INIT = 'GRIPPER_INIT'
    HOME_NORMAL = 'HOME_NORMAL'
    HOME_KETTLE = 'HOME_KETTLE'
    PICKUP = 'PICKUP'
    PICKUP_CUP = 'PICKUP_CUP'
    PICKUP_DRIP = 'PICKUP_DRIP'
    SHUTDOWN = 'SHUTDOWN'
    STOP = 'STOP'
    RUN = 'RUN'
    RESTART = 'RESTART'
    HOME = 'HOME'
    RESET = 'RESET'


class DispenseCommand:
    WATER_TOGGLE = 'WATER_TOGGLE'
    WATER_PIN_RESET = 'WATER_PIN_RESET'
    COFFEE_PIN_RESET = 'COFFEE_PIN_RESET'
    COFFEE_TOGGLE = 'COFFEE_TOGGLE'
    COFFEE_ON = 'COFFEE_ON'
    COFFEE_OFF = 'COFFEE_OFF'
    WATER = 'WATER'
    COFFEE = 'COFFEE'


class ResponseDataList:
    response_cd= 999
    status_cd = ""
    component_cd = ""
    data_list = []
    cmd = ""


class ResponseCode:
    SUCCESS = 0  # 정상
    OFF = 1  # OFF 상태
    ON = 2  # ON 상태
    STANDBY = 3  # STANDBY 상태
    WORKING = 4  # WORKING 상태
    ERROR = 9


class Constants:
    ZERO = 0
    ONE = 1
    TWO = 2
    THREE =3
    FOUR = 4
    TIMER_PERIOD = 0.1 #0.1
    ROBOTS_SYSTEM = 'RobotSystemNode'
    QOS_DEFAULT = 20
    DISPENSER_NODE = 'DispenserNode'


class Service:
    SERVICE_ROBOT = 'XYZ_robot/service'
    SERVICE_DISPENSER = 'XYZ_dispenser/service'

class Topic:
    ROBOT_STATUS = 'XYZ_robot/status'

class DeviceCode:
    ROBOT = 'ROBOT'
    WATER = 'WATER'
    COFFEE= 'COFFEE'


class DeviceStatus:
    STANDBY = 'STANDBY'
    WORKING = 'WORKING'
    ERROR = 'ERROR'
    RESET = 'RESET'
