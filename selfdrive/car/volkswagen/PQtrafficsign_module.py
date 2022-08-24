import numpy
import math
import datetime
import json


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
        self.filename = "tsrLog" + "_" + self.startup_ts
        self.log_filename_csv = self.log_path + "tsr/" + self.filename + ".csv"
        self.cnt = 0

    def update(self, can_parser):
        speed_ms = 0.0  # looks like zero = no limit detected, [m/s]

        detected_signs_lst = self.update_bap(can_parser, log=True)

        return speed_ms

    def update_bap(self, can_parser, log=False):

        detected_signs_lst = []

        raw_can_data = self.raw_can_receiver.update()

#        bap_pkg = self.bap.receive_can(self.BAP_VZA_CAN_ID, raw_can_data)

#        if bap_pkg != None:
#            self.update_log(bap_pkg)
#            self.parse_bap_vza(bap_pkg)
        self.update_log_raw( raw_can_data )

        return detected_signs_lst

    def get_ts_now(self, with_date=False):
        if with_date:
            return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        else:
            return datetime.datetime.now().strftime("%H:%M:%S.%f")

    def update_log(self, bap_pkg):
        log_file = open(self.log_filename_csv, 'a')
        log_file.write( str(bap_pkg) + "\n" )
        log_file.close()

    def update_log_raw(self, can_parser_raw):
        #if bool(can_parser_raw):  # check if dict can_parser_raw is empty
        log_file = open(self.log_filename_csv, 'a')
        log_file.write( str(can_parser_raw) + "\n" )
        log_file.close()


    def parse_bap_vza(self, bap_pkg):
        can_id, op_code, lsg_id, fct_id, bap_data = bap_pkg
        if can_id == self.BAP_VZA_CAN_ID:
            if lsg_id == self.BAP_VZA_LSG_ID:
                pass



# CANid 1691
# lsg_id 16
# fct_id 16

# bap data (dec):
# 1 120 0 0 0 0 0 0 0 0 0 0 # 120km/h
# 1 100 0 0 0 0 0 0 0 0 0 0 # 100km/h

# 3 100 0 0 0 0 0 0 0 0 0 0 # alles frei
