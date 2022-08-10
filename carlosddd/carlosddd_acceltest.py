import numpy
import math
import datetime
from common.op_params import opParams


'''
module to perform constant acceleration / deceleration tests
to be used with https://github.com/sshane/opParams

in order to use this test, set the allowance variables on opParams: 
    * accel_test_allowable (global allow tests from this module)
    * accel_test_gas_schedulable (allow gas tests)
    * accel_test_brake_schedulable (allow brake tests)

the corresponding tests must be allowed an can the be triggered (while OP is disengaged !!!) by changing
    * accel_test_target_speed_gas or accel_test_gas_val for gas tests
    * accel_test_target_speed_brake or accel_test_brake_val
    -> a change in above values will automatically "trigger" the corresponding test
    -> please not that triggering is only allowed accepted whilst OP is NOT engaged, if OP is engaged it will be ignored!

logs will be written to /data/openpilot/carlosddd/logs/gas_break/ in an GNU octave readable format
use with wrk_pushpull/K2_Octavescripts/OP/ plot_brake_test.m or plot_gas_test.m for plotting or further processing

usage:
    init ->      CdddA = Carlosddd_Acceltest()
    loop ->      CdddA.update(self, op_enabled, vEgo, aEgo, clutchPressed, gasPressed, gear, rpm)
'''

class Carlosddd_Acceltest:
    def __init__(self, test_dict=None):

        if test_dict is None:
            # no testing -> read normal params
            self.op_params = opParams()
            print(">>> carlosddd_acceltest.py: using OPparams")

        # initial values to detect change from
        self.update_op_params(gas_tests_allowable=True, brake_tests_allowable=True, initial_run=True, test_dict=test_dict)

        self.gas_test_wait_for_clutch = False
        self.gas_test_active = False
        self.brake_test_active = False
        if test_dict is None:
            self.logPath = "/data/openpilot/carlosddd/logs/gas_break/"
        else:
            self.logPath = "logs/gas_break/"
        self.gas_log_open = False
        self.brake_log_open = False
        self.gas_log_idx = 0
        self.brake_log_idx = 0
    
    def get_ts_now(self, with_date=False):
        if with_date:
            return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        else:
            return datetime.datetime.now().strftime("%H:%M:%S.%f")

    def update_op_params(self, gas_tests_allowable, brake_tests_allowable, initial_run=False, test_dict=None):
        #print("test_dict:", test_dict, ";initial_run:", initial_run)
        if test_dict is None:
            # no testing -> fetch them for OPparam
            tests_allowable = self.op_params.get('accel_test_allowable')
            gas_test_schedulable = self.op_params.get('accel_test_gas_schedulable')
            brake_test_schedulable = self.op_params.get('accel_test_brake_schedulable')
            target_speed_gas = self.op_params.get('accel_test_target_speed_gas')/3.6    # m/s -> km/h
            target_speed_brake = self.op_params.get('accel_test_target_speed_brake')/3.6    # m/s -> km/h
            gas_val = int(self.op_params.get('accel_test_gas_val'))
            brake_val = int(self.op_params.get('accel_test_brake_val'))
        else:
            # test dict provided -> use these values instead
            print(">>> carlosddd_acceltest.py: ATTENTION !!! using test_dict values instead of those from OPparams!")
            tests_allowable = test_dict['accel_test_allowable']
            gas_test_schedulable = test_dict['accel_test_gas_schedulable']
            brake_test_schedulable = test_dict['accel_test_brake_schedulable']
            target_speed_gas = test_dict['accel_test_target_speed_gas']/3.6    # m/s -> km/h
            target_speed_brake = test_dict['accel_test_target_speed_brake']/3.6    # m/s -> km/h
            gas_val = int(test_dict['accel_test_gas_val'])
            brake_val = int(test_dict['accel_test_brake_val'])

        # see if a value has been changed to start a new test
        # only update if allowable (aka no current test / staging for test present)
        if not initial_run:
            if ((target_speed_gas != self.target_speed_gas) or (gas_val != self.gas_val)) and tests_allowable and gas_test_schedulable:
                if gas_tests_allowable:
                    self.gas_test_scheduled = True
                    #print("gas_test_scheduled (change) set to", self.gas_test_scheduled)
            elif gas_tests_allowable:
                self.gas_test_scheduled = False
                #print("gas_test_scheduled (not initial run) set to", self.gas_test_scheduled)
            if ((target_speed_brake != self.target_speed_brake) or (brake_val != self.brake_val)) and tests_allowable and brake_test_schedulable:
                if brake_tests_allowable:
                    self.brake_test_scheduled = True
                    #print("brake_test_scheduled (change) set to", self.brake_test_scheduled)
            elif brake_tests_allowable:
                self.brake_test_scheduled = False
                #print("brake_test_scheduled (not initial run) set to", self.brake_test_scheduled)
        else:
            if gas_tests_allowable or initial_run:  # inital run -> need to create memeber
                self.gas_test_scheduled = False
            if brake_tests_allowable or initial_run:
                self.brake_test_scheduled = False
        
        self.tests_allowable = tests_allowable
        self.target_speed_gas = target_speed_gas
        self.target_speed_brake = target_speed_brake
        self.gas_val = gas_val
        self.brake_val = brake_val

    def update(self, op_enabled, vEgo, aEgo, clutchPressed, gasPressed, gear, rpm, test_dict=None):
        active = False
        gas_tests_allowable = (not op_enabled) and (not self.gas_test_active) and (not self.gas_test_scheduled) and (not self.gas_test_wait_for_clutch)
        brake_tests_allowable = (not op_enabled) and (not self.brake_test_active) and (not self.brake_test_scheduled)
        #print("op_enabled", op_enabled, "brake_test_active", self.brake_test_active, "brake_test_scheduled", self.brake_test_scheduled)
        #print("update_op_params(", gas_tests_allowable, "," , brake_tests_allowable, ")")
        self.update_op_params(gas_tests_allowable, brake_tests_allowable, initial_run=False, test_dict=test_dict)
        apply_gas = 0
        apply_brake = 0
        if self.tests_allowable and op_enabled:
            if any([self.gas_test_active, self.brake_test_active]):
                apply_gas = self.update_gas(vEgo, aEgo, clutchPressed, gear, rpm)
                apply_brake = self.update_brake(vEgo, aEgo, clutchPressed, gasPressed)
            elif ((clutchPressed and self.gas_test_scheduled) or self.gas_test_wait_for_clutch) and not self.brake_test_active:
                if self.gas_test_scheduled:
                    print(">>> carlosddd_acceltest.py: Waiting for gas test to begin, release clutch!")
                    # preactive (wait for clutch release)
                    self.gas_test_wait_for_clutch = True
                    self.gas_test_scheduled = False
                elif self.gas_test_wait_for_clutch:
                    # become active
                    if not clutchPressed:
                        print(">>> carlosddd_acceltest.py: gas test beginning")
                        self.gas_test_active = True
                        self.gas_test_wait_for_clutch = False
                        self.gas_log = open(self.logPath+"gasLog_"+str(self.get_ts_now(with_date=True))+"__"+str(int(vEgo))+"_to_"+str(int(self.target_speed_gas))+".m", 'a')
                        self.gas_log_open = True
                        self.gas_log_idx = 0
                        self.gas_log.write("# log of accerleration test" + "\n")
                        self.gas_log.write("apply_gas = " + str(self.gas_val) + "\n")
                        self.gas_log.write("inp_data = [" + "\n")
            elif clutchPressed and self.brake_test_scheduled and not self.gas_test_active and not self.gas_test_wait_for_clutch:
                print(">>> carlosddd_acceltest.py: brake test beginning")
                self.brake_test_active = True
                self.brake_test_scheduled = False
                self.brake_log = open(self.logPath+"brakeLog_"+str(self.get_ts_now(with_date=True))+"__"+str(int(vEgo))+"_to_"+str(int(self.target_speed_brake))+".m", 'a')
                self.brake_log_open = True
                self.brake_log_idx = 0
                self.brake_log.write("# log of accerleration test" + "\n")
                self.brake_log.write("apply_brake = " + str(self.brake_val) + "\n")
                self.brake_log.write("inp_data = [" + "\n")
            active = self.gas_test_active or self.brake_test_active
        else:
            # disengage deactivates all active tests and all conditions for safety reasons!
            self.gas_test_active = False
            self.brake_test_active = False
            self.gas_test_wait_for_clutch = False


            if self.brake_log_open:
                self.brake_log.close()
                print(">>> carlosddd_acceltest.py: brake log closed() unexpectedly")
                self.brake_log_open = False
            if self.gas_log_open:
                self.gas_log.close()
                print(">>> carlosddd_acceltest.py: gas log closed() unexpectedly")
                self.gas_log_open = False
            apply_gas = 0
            apply_brake = 0
        return int(apply_gas), int(apply_brake), active

    def update_gas(self, vEgo, aEgo, clutchPressed, gear, rpm):
        apply_gas = 0
        if self.gas_test_active and not self.brake_test_active:
            if not clutchPressed:   # allow for shifiting
                apply_gas = self.gas_val
                # log here
                self.gas_log.write(str(self.gas_log_idx)+","+str(vEgo)+","+str(aEgo)+","+str(int(gear))+","+str(int(rpm))+";"+"\n")
            else:
                apply_gas = 0
                self.gas_log.write(str(self.gas_log_idx)+","+str(vEgo)+","+"nan"+","+"nan"+","+str(int(rpm))+";"+"\n")
            if vEgo >= self.target_speed_gas:
                self.gas_test_active = False    # done
                apply_gas = 0
                self.gas_log.write("];" + "\n")
                self.gas_log.write("idx = inp_data(:,1);" + "\n")
                self.gas_log.write("vEgo = inp_data(:,2);" + "\n")
                self.gas_log.write("aEgo = inp_data(:,3);" + "\n")
                self.gas_log.write("gear = inp_data(:,4);" + "\n")
                self.gas_log.write("rpm = inp_data(:,5);" + "\n")
                self.gas_log.close()
                self.gas_log_open = False
                print(">>> carlosddd_acceltest.py: gas test done, log closed()")
            self.gas_log_idx += 1
        else:
            apply_gas = 0
        return apply_gas

    def update_brake(self, vEgo, aEgo, clutchPressed, gasPressed):
        if self.brake_test_active and not self.gas_test_active:
            if clutchPressed and not gasPressed:
                apply_brake = self.brake_val
                # log here
                self.brake_log.write(str(self.brake_log_idx)+","+str(vEgo)+","+str(aEgo)+";"+"\n")
            else:
                apply_brake = 0
                self.brake_log.write(str(self.brake_log_idx)+","+str(vEgo)+","+"nan"+";"+"\n")
            if vEgo <= self.target_speed_brake:
                self.brake_test_active = False
                apply_brake = 0
                self.brake_log.write("];" + "\n")
                self.brake_log.write("idx = inp_data(:,1);" + "\n")
                self.brake_log.write("vEgo = inp_data(:,2);" + "\n")
                self.brake_log.write("aEgo = inp_data(:,3);" + "\n")
                self.brake_log.close()
                self.brake_log_open = False
                print(">>> carlosddd_acceltest.py: brake test done, log closed()")
            self.brake_log_idx += 1
        else:
            apply_brake = 0
        return apply_brake
        
