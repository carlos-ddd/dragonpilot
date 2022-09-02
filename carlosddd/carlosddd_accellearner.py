import numpy
import math
import datetime
from common.op_params import opParams
import os.path

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

        self.abs_path = "/data/openpilot/carlosddd/"
        self.log_path = self.abs_path + "logs/learn/"
        self.startup_ts = self.get_ts_now(with_date=True)

        self.log_filename_csv_brake = self.unique_filename(self.log_path, "learn_brake", "csv", self.startup_ts)
        self.log_filename_csv_gas = self.unique_filename(self.log_path, "learn_gas", "csv", self.startup_ts)
        self.log_filename_csv_coast = self.unique_filename(self.log_path, "learn_coast", "csv", self.startup_ts)
        self.log_filename_csv_roll = self.unique_filename(self.log_path, "learn_roll", "csv", self.startup_ts)

        # will be set to True as soon as the header has been written (once)
        self.log_csv_header_written_gas = False
        self.log_csv_header_written_brake = False
        self.log_csv_header_written_coast = False
        self.log_csv_header_written_roll = False

        self.json_path = self.abs_path + "config.json"
        json_data = self.load_json(self.json_path)
        self.parse_json(json_data)

    def load_json(self, json_path):
        with open(json_path, 'r') as f:
            json_data = json.load(f)
        return json_data

    def parse_json(self, json_data):
        self.config_learn =  bool(json_data['learn'])
        self.config_learn_gas =  bool(json_data['learn_gas']) and self.config_learn
        self.config_learn_brake =  bool(json_data['learn_brake']) and self.config_learn
        self.config_learn_coast =  bool(json_data['learn_coast']) and self.config_learn
        self.config_learn_roll =  bool(json_data['learn_roll']) and self.config_learn
        if not any( [self.config_learn_gas, self.config_learn_brake, self.config_learn_coast, self.config_learn_roll] ):
            self.skip_learn = True
        else:
            self.skip_learn = False

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
        if not self.skip_learn:     # do nothing if we're not interested in the log (save performance)
            v_Ego = int(vEgo*3.6)   # km/h
            v_Ego_inrange = self.min_vEgo_kmh <= v_Ego <= self.max_vEgo_kmh
            gear = int(gearDetected)
            gear_valid = (1 <= gear <= self.num_gear)
            rpm = int(round(rpmDeteced/10)*10)
            rpm_inrange = (self.min_rpm <= rpm <= self.max_rpm)
            if numpy.isnan( float(gasVal) ):
                gasVal = 0
            gas_val = int(gasVal)       # OP commanding
            gas_inrange = (self.min_gas_val <= gas_val <= self.max_gas_val)
            brake_val = int(brakeVal)   # OP commanding
            if numpy.isnan( float(brakeVal) ):
                brake_val = 0
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
                print(">>> carlosddd_accellearner.py: mode change", self.mode_old, "->", self.mode)
            self.mode_old = self.mode   # update mode

    def update_brake(self, v_Ego, a_Ego, gear, brake_val):
        self.mode = 'brake'
        if self.config_learn_brake:
            self.log_csv_header_written_brake = self.write_csv(self.log_filename_csv_brake, [v_Ego, a_Ego, gear, brake_val], ["v_Ego", "a_Ego", "gear", "brake_val"], write_header=not self.log_csv_header_written_brake)

    def update_gas(self, v_Ego, a_Ego, gear, gas_val, pedal_val, sourceDriver):
        self.mode = 'gas'
        if self.config_learn_gas:
            self.log_csv_header_written_gas = self.write_csv(self.log_filename_csv_brake, [v_Ego, a_Ego, gear, gas_val, pedal_val, sourceDriver], ["v_Ego", "a_Ego", "gear", "gas_val", "pedal_val", "sourceDriver"], write_header=not self.log_csv_header_written_gas)

    def update_coasting(self, v_Ego, a_Ego, gear):
        self.mode = 'coasting'
        if self.config_learn_coast:
            self.log_csv_header_written_coast = self.write_csv(self.log_filename_csv_brake, [v_Ego, a_Ego, gear], ["v_Ego", "a_Ego", "gear"], write_header=not self.log_csv_header_written_coast)

    def update_rolling(self, v_Ego, a_Ego):
        self.mode = 'rolling'
        if self.config_learn_roll:
            self.log_csv_header_written_roll = self.write_csv(self.log_filename_csv_brake, [v_Ego, a_Ego], ["v_Ego", "a_Ego"], write_header=not self.log_csv_header_written_roll)

    def write_csv(self, file_name, var_lst, var_name_lst, write_header=False):
        header_written = False
        log_file = open(file_name, 'a')
        if write_header:
            out_str = ""
            for var_name in var_name_lst:
                out_str += var_name
                out_str += ","
            out_str = out_str[:-1] # remove last comma
            log_file.write( out_str + "\n" )
        out_str = ""
        header_written = True
        for var in var_lst:
            out_str += str(var)
            out_str += ","
        out_str = out_str[:-1] # remove last comma
        log_file.write( out_str + "\n" )
        log_file.close()
        return header_written

    def unique_filename(self, path, log_type, filetype, time_stamp_str):
        org_path = path + log_type + "_" + time_stamp_str + "." + filetype
        addon = 0
        addon_str = ""
        while True:
            org_path = path + log_type + "_" + time_stamp_str + addon_str + "." + filetype
            if os.path.exists(org_path):
                addon_str = "__" + str(addon)
            else:
                break
            addon += 1
        print(">>> carlosddd_accellearner.py::unique_filename():", org_path)
        return org_path

    def get_ts_now(self, with_date=False):
        if with_date:
            return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        else:
            return datetime.datetime.now().strftime("%H:%M:%S.%f")
