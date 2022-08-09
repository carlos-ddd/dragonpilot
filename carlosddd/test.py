from carlosddd_logmodule import Carlosddd_Logmodule


CdddL = Carlosddd_Logmodule("Test")

for i in range(0,130):
    if i != 12:
        CdddL.update("ki", 500.2+i)
        CdddL.update("kp", 200.4+i)
        CdddL.update("V", 590.88+i)
    else:
        CdddL.update("ki", float('nan'))
        CdddL.update("kp", 200.4+i)
        CdddL.update("V", 590.88+i)
    CdddL.slice_done()