#!/usr/bin/python
# -*- coding: utf-8 -*-

import hakopy
import hako_pdu
import infra_sensor
import robot_pdu_info as pdu_info
import sys
from enum import Enum

pdu_manager = None
robot_controller = None

class RobotStatus(Enum):
    INIT = 1
    MOVE = 2
    STOP = 3

class RobotEvent(Enum):
    MOVE = 1
    STOP = 2


class RobotController:
    def __init__(self):
        global pdu_manager
        self.target_stop_pos_y = 9.8
        self.attached_duration_time_usec = 0
        self.attached_start_time_usec = 0
        self.status = RobotStatus.INIT
        self.pdu_baggage = pdu_manager.get_pdu(pdu_info.AVATAR_NAME, pdu_info.PDU_BAGGAGE_CHANNEL_ID)
        self.pdu_motor = pdu_manager.get_pdu(pdu_info.MY_ROBOT_NAME, pdu_info.PDU_MOTOR_CHANNEL_ID)
        self.pdu_pos = pdu_manager.get_pdu(pdu_info.AVATAR_NAME, pdu_info.PDU_POS_CHANNEL_ID)
        self.d_motor = self.pdu_motor.get()
        self.d_baggage = self.pdu_baggage.get()
        self.infra = infra_sensor.InfraSensor(pdu_manager)

    def event(self, event: RobotEvent):
        if (self.status == RobotStatus.INIT):
            if (event == RobotEvent.MOVE):
                print("INFO: EVENT DO MOVE")
                self.status = RobotStatus.MOVE
        elif (self.status == RobotStatus.MOVE):
            if (event == RobotEvent.STOP):
                print("INFO: EVENT DO STOP")
                self.status = RobotStatus.STOP

    def do_read(self):
        self.d_baggage = self.pdu_baggage.read()
        self.d_pos = self.pdu_pos.read()
    
    def do_write(self):
        #print("motor: ", self.pdu_motor.get()['linear']['x'])
        self.pdu_motor.write()

    def run_init(self):
        #motor
        self.d_motor['linear']['x'] = 0
        self.d_motor['linear']['y'] = 0
        self.d_motor['linear']['z'] = 0
        #touch sensor
        is_pressed = self.d_baggage['data']
        if is_pressed:
            if self.attached_start_time_usec > 0:
                self.attached_duration_time_usec = hakopy.simulation_time() - self.attached_start_time_usec
            else:
                self.attached_start_time_usec = hakopy.simulation_time()
        else:
            self.attached_start_time_usec = 0
            self.attached_duration_time_usec = 0

        # 3sec
        if self.attached_duration_time_usec >= 3000000:
            self.event(RobotEvent.MOVE)

    def run_move(self):
        #motor
        pos_y = self.d_pos['linear']['y']
        #print("pos_y: ", pos_y)
        if pos_y >= self.target_stop_pos_y:
            self.d_motor['linear']['x'] = 0.0
        else:
            self.d_motor['linear']['x'] = 0.1

    def run_stop(self):
        #motor
        self.d_motor['linear']['x'] = 0

    def run(self):
        self.infra.run()
        self.do_read()

        if (self.status == RobotStatus.INIT):
            self.run_init()
        elif (self.status == RobotStatus.MOVE):
            self.run_move()
        elif (self.status == RobotStatus.STOP):
            self.run_stop()
        
        self.do_write()

def my_on_initialize(context):
    return 0

def my_on_reset(context):
    return 0

def my_on_simulation_step(context):
    global robot_controller
    robot_controller.run()

    return 0

my_callback = {
    'on_initialize': my_on_initialize,
    'on_simulation_step': my_on_simulation_step,
    'on_manual_timing_control': None,
    'on_reset': my_on_reset
}
def main():
    global pdu_manager
    global robot_controller
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <config_path> [asset_only]")
        return 1
    
    enable_master = True
    if len(sys.argv) == 3:
        enable_master = False

    asset_name = 'RobotController'
    config_path = sys.argv[1]
    delta_time_usec = 20000

    pdu_manager = hako_pdu.HakoPduManager('/usr/local/lib/hakoniwa/hako_binary/offset', config_path)

    if enable_master:
        hakopy.conductor_start(delta_time_usec, delta_time_usec)

    robot_controller = RobotController()

    ret = hakopy.asset_register(asset_name, config_path, my_callback, delta_time_usec, hakopy.HAKO_ASSET_MODEL_CONTROLLER)
    if ret == False:
        print(f"ERROR: hako_asset_register() returns {ret}.")
        return 1

    ret = hakopy.start()

    if enable_master:
        hakopy.conductor_stop()

    return 0

if __name__ == "__main__":
    sys.exit(main())
