from carlosddd_logmodule import Carlosddd_Logmodule
import random
import time

CdddL = Carlosddd_Logmodule("Test", test_environment=True)

for i in range(0,130):
    if i != 12:
        CdddL.update("ki", 500.2+i)
        CdddL.update("kp", 200.4+i)
        CdddL.update("V", 590.88+i)
        CdddL.update("gas_pressed", False, convert=True)
    else:
        CdddL.update("ki", float('nan'))
        CdddL.update("kp", 200.4+i)
        CdddL.update("V", 590.88+i)
        CdddL.update("gas_pressed", True, convert=True)
    CdddL.slice_done()