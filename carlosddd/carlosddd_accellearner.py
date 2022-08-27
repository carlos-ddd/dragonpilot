import numpy
import math
import datetime
from common.op_params import opParams


'''
module to passivly learn acceleration for gas and brake
'''

class Carlosddd_Timer:
    def __init__(self, timer_limit):
        self.timer_limit = timer_limit
        self.timer_value = 0
    def update(self, condition):
        if not condition:
            self.timer_value = 0
        else:
            self.timer_value += 1
        timer_elapsed = (self.timer_value > self.timer_limit)
        return timer_elapsed

class Carlosddd_Accellearner:

    def __init__(self):
        self.num_gear = 6
        self.min_gas_val = 227
        self.max_gas_val = 1245
        self.min_vEgo_kmh = 1
        self.max_vEgo_kmh = 210
        self.min_rpm = 800
        self.max_rpm = 4500
        self.min_brake = 1
        self.max_brake = 8192

        self.clutch_timer = Carlosddd_Timer(100)
        self.leergas_timer = Carlosddd_Timer(100)
        self.declutch_timer = Carlosddd_Timer(100)

        self.mode = 'init'
        self.mode_old = 'init'

    def init_gas(self):
        self.gas_data = []              # [gear][gas_val][rpm] = aEgo
        for i in range(self.num_gear):
            self.gas_data.append([])
            
    def init_brake(self):
        self.brake_data = []            #
    
    def init_coasting(self):
        self.coasting_data = []          #

    def init_rolling(self):
        self.coasting_data = []          #

    def update(self, op_enabled, vEgo, aEgo, clutchPressed, gearDetected, rpmDeteced, gasVal, brakeVal, pedalVal, leergas, brakePressed):
        v_Ego = int(vEgo*3.6)   # km/h
        v_Ego_inrange = self.min_vEgo_kmh <= v_Ego <= self.max_vEgo_kmh
        gear = int(gearDetected)
        gear_valid = (1 <= gear <= self.num_gear)
        rpm = int(round(rpmDeteced/10)*10)
        rpm_inrange = (self.min_rpm <= rpm <= self.max_rpm)
        gas_val = int(gasVal)       # OP commanding
        gas_inrange = (self.min_gas_val <= gas_val <= self.max_gas_val)
        brake_val = int(brakeVal)   # OP commanding
        brake_inrange = (self.min_brake <= brake_val <= self.max_brake)
        pedal_val = int(pedalVal)   # driver commanding (gas pedal)
        pedal_inrange = (self.min_gas_val <= pedal_val <= self.max_gas_val)
        a_Ego = round(aEgo, 2) # x.xx
        no_override = pedal_val < gas_val

        clutch_timer_elapsed = self.clutch_timer.update(clutchPressed)
        leergas_timer_elapsed = self.leergas_timer.update(leergas)
        declutch_timer_elapsed = self.declutch_timer.update(not clutchPressed)

        clutch_released_validated = clutch_timer_elapsed and not clutchPressed
        leergas_validated = leergas_timer_elapsed and leergas
        clutch_pressed_validated = declutch_timer_elapsed and clutchPressed

        self.mode = 'unknown'

        if not brakePressed:
            if clutch_released_validated and gear_valid:    # we only want data when we're clutched in some time (anti jerk etc.)
                if op_enabled:
                    # we can have data from OP commanded gas / brake
                    if leergas_validated and brake_inrange: # no gas is commanded and we're activly braking
                        self.update_brake(v_Ego, a_Ego, gear, brake_val)
                    elif gas_inrange and no_override and not brake_inrange: # gas is commanded, no braking commanded 
                        self.update_gas(v_Ego, a_Ego, gear, gas_val, pedal_val, sourceDriver=False)
                    elif not gas_inrange and leergas_validated and not brake_inrange:
                        self.update_coasting(v_Ego, a_Ego, gear)
                elif leergas_validated:
                    self.update_coasting(v_Ego, a_Ego, gear)
                elif not leergas and pedal_inrange:
                    self.update_gas(v_Ego, a_Ego, gear, gas_val, pedal_val, sourceDriver=True)
            elif clutch_pressed_validated:
                self.update_rolling(v_Ego, a_Ego)

        if self.mode != self.mode_old:
            print(">>> carlosddd_accellearner.py: mode change", self.mode_old, "->" self.mode)
        self.mode_old = self.mode   # update mode

    def update_brake(self, v_Ego, a_Ego, gear, brake_val):
        self.mode = 'brake'

    def update_gas(self, v_Ego, a_Ego, gear, gas_val, pedal_val, sourceDriver):
        self.mode = 'gas'

    def update_coasting(self, v_Ego, a_Ego, gear):
        self.mode = 'coasting'

    def update_rolling(self, v_Ego, a_Ego):
        self.mode = 'rolling'
