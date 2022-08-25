#!/usr/bin/env python3

import cereal.messaging as messaging

'''
script to get the raw data stream for BAP parsing
since BAP has dynamic dlc the "conventional" OP parser is not suitable (no dlc information available)
derived from https://github.com/commaai/openpilot/blob/b3cfe962cf346e70cce1389fe3c9bf23344a5512/selfdrive/debug/can_table.py#L36
'''

class Raw_CAN_Receiver():

    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr
        self.can = messaging.sub_sock('can', conflate=False, timeout=None)
        print(">>>raw_can_receiver.py::__init__() for bus", self.bus, "addr", self.addr)

    def update(self):
        out_data = []
        for msg in messaging.drain_sock(self.can, wait_for_one=True):
            for m in msg.can:
                if m.address == self.addr and m.src == self.bus:
                    out_data.append(list(m.dat))
        if len(out_data) > 0:
            print(">>>raw_can_receiver.py::update() out_data", out_data, "len", len(out_data))
        return out_data
