import numpy
import math
import datetime
import json
import os.path


from selfdrive.car.volkswagen.PQbap_module import PQbap
from selfdrive.car.volkswagen.raw_can_receiver import Raw_CAN_Receiver

class PQtsr():
    def __init__(self):
        self.bap = PQbap()
        self.BAP_VZA_CAN_ID = 1691  # dlc=8, 0x69B; Edgy-info: 0x69B FSG camera to cluster; Datasource: PQ-Systembeschreibung on CANs:Extended,Infotainment,Kombi
        self.BAP_VZA_CAN_BUS = 2
        self.BAP_VZA_LSG_ID = 33
        self.raw_can_receiver = Raw_CAN_Receiver(self.BAP_VZA_CAN_BUS, self.BAP_VZA_CAN_ID)

        self.startup_ts = self.get_ts_now(with_date=True)
        self.abs_path = "/data/openpilot/carlosddd/"
        self.log_path = self.abs_path + "logs/"

        self.log_filename_csv = self.unique_filename(self.log_path, "tsrLog", "csv", self.startup_ts)
        self.log_filename_csv_raw = self.unique_filename(self.log_path, "tsrLogRaw", "csv", self.startup_ts)

        self.cnt = 0

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
        print(">>> PQtrafficsign_module.py::unique_filename():", org_path)
        return org_path

    def update(self):
        speed_ms = 0.0  # looks like zero = no limit detected, [m/s]
        detected_signs_lst = self.update_bap(log=True)
        if not detected_signs_lst:  # list not empty
            speed_kph = detected_signs_lst[0]   # so far we only evaluate the first traffic sign,
                                                #  what ever that would mean !? (see parse_bap_vza())
            speed_ms = speed_kph / 3.6
            print(">>>", detected_signs_lst "->", speed_kph)
        return speed_ms

    def update_bap(self, log=False):

        detected_signs_lst = []

        raw_can_data_lst = self.raw_can_receiver.update()

        for raw_can_data in raw_can_data_lst:
            self.update_log_raw( raw_can_data )
            #print(">>>", raw_can_data)
            bap_pkg = self.bap.receive_can(self.BAP_VZA_CAN_ID, raw_can_data)
            if bap_pkg != None:
                self.update_log(bap_pkg, decoded=True)
                #print(">>>", bap_pkg)
                detected_signs_lst = self.parse_bap_vza(bap_pkg)

        return detected_signs_lst

    def get_ts_now(self, with_date=False):
        if with_date:
            return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        else:
            return datetime.datetime.now().strftime("%H:%M:%S.%f")

    def update_log(self, bap_pkg, decoded=True):
        log_file = open(self.log_filename_csv, 'a')
        can_id, op_code, lsg_id, fct_id, bap_data = bap_pkg
        log_file.write( str(can_id) + ";" )
        log_file.write( str(lsg_id) + ";" )
        log_file.write( str(fct_id) + ";" )
        log_file.write( str(op_code) + ";" )
        log_file.write( str(bap_data) + ";" )
        log_file.write( str(bap_pkg) + "\n" )
        log_file.close()

    def update_log_raw(self, to_log):
        #if bool(to_log):  # check if dict to_log is empty
        log_file = open(self.log_filename_csv_raw, 'a')
        log_file.write( str(to_log) + "\n" )
        log_file.close()


    def parse_bap_vza(self, bap_pkg):
        detected_signs_lst = []

        can_id, op_code, lsg_id, fct_id, bap_data = bap_pkg
        if can_id == self.BAP_VZA_CAN_ID:
            if lsg_id == self.BAP_VZA_LSG_ID:
                if fct_id==16 and op_code in [3,4] and bap_data[0]==1:  # see explanations below
                    for itm in bap_data[1:]:
                        if itm != 0:
                            if 1 <= itm <= 210:
                                detected_signs_lst.append(itm)
                    

        return detected_signs_lst



# CANid 1691
# lsg_id 33

# fct_id 16

# bap data (dec):
# 1 120 0 0 0 0 0 0 0 0 0 0 # 120km/h
# 1 100 0 0 0 0 0 0 0 0 0 0 # 100km/h

# 3 100 0 0 0 0 0 0 0 0 0 0 # alles frei
