import numpy
import math

from selfdrive.car.volkswagen.PQbap_module import PQbap

class PQtsr():
    def __init__(self):
        self.bap = PQbap()
        self.BAP_VZA_CAN_ID = 1691  # dlc=8, 0x69B; Edgy-info: 0x69B FSG camera to cluster; Datasource: PQ-Systembeschreibung on CANs:Extended,Infotainment,Kombi

    def update(self, can_parser):
        speed_ms = 0.0  # looks like zero = no limit detected

        detected_signs_lst = self.update_bap(can_parser)

        return speed_ms

    def update_bap(self, can_parser):
        detected_signs_lst = []

        bap_data_raw = []
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data0'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data1'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data2'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data3'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data4'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data5'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data6'])
        bap_data_raw.append(can_parser.vl["BAP_VZA"]['BAP_VZA_data7'])

        bap_pkg = self.bap.receive_can(message.arbitration_id, bap_data_raw[:true_dlc])
        if bap_pkg != None:
            can_id, op_code, lsg_id, fct_id, bap_data = bap_pkg

        return detected_signs_lst



# CANid 1691
# lsg_id 16
# fct_id 16

# bap data (dec):
# 1 120 0 0 0 0 0 0 0 0 0 0 # 120km/h
# 1 100 0 0 0 0 0 0 0 0 0 0 # 100km/h

# 3 100 0 0 0 0 0 0 0 0 0 0 # alles frei