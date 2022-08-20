import numpy
import math
import datetime
import json


from selfdrive.car.volkswagen.PQbap_module import PQbap

class PQtsr():
    def __init__(self):
        self.bap = PQbap()
        self.BAP_VZA_CAN_ID = 1691  # dlc=8, 0x69B; Edgy-info: 0x69B FSG camera to cluster; Datasource: PQ-Systembeschreibung on CANs:Extended,Infotainment,Kombi
        
        self.startup_ts = self.get_ts_now(with_date=True)
        self.abs_path = "/data/openpilot/carlosddd/"
        self.log_path = self.abs_path + "logs/"
        self.filename = "tsrLog" + "_" + self.startup_ts
        self.log_filename_csv = self.log_path + "tsr/" + self.filename + ".csv"
        self.log_file = open(self.log_filename_csv, 'a')
        self.cnt = 0

    def update(self, can_parser):
        speed_ms = 0.0  # looks like zero = no limit detected, [m/s]

        detected_signs_lst = self.update_bap(can_parser, log=True)

        return speed_ms

    def update_bap(self, can_parser, log=False):
        detected_signs_lst = []

#        bap_data_raw = []
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data0'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data1'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data2'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data3'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data4'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data5'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data6'])
#        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data7'])
#
#        bap_pkg = self.bap.receive_can(message.arbitration_id, bap_data_raw[:true_dlc])
#        if bap_pkg != None:
#            self.update_log(bap_pkg)
#            self.parse_bap_vza(bap_pkg)
        self.update_log_raw( can_parser.vl["BAP_VZA"] )

        return detected_signs_lst
        
    def get_ts_now(self, with_date=False):
        if with_date:
            return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        else:
            return datetime.datetime.now().strftime("%H:%M:%S.%f")
            
    def update_log(self, bap_pkg):
        self.log_file.write( str(bap_pkg) + "\n" )

    def update_log_raw(self, can_parser_raw):
        self.log_file.write( str(can_parser_raw) + "\n" )


    def parse_bap_vza(self, bap_pkg):
        can_id, op_code, lsg_id, fct_id, bap_data = bap_pkg
        pass



# CANid 1691
# lsg_id 16
# fct_id 16

# bap data (dec):
# 1 120 0 0 0 0 0 0 0 0 0 0 # 120km/h
# 1 100 0 0 0 0 0 0 0 0 0 0 # 100km/h

# 3 100 0 0 0 0 0 0 0 0 0 0 # alles frei