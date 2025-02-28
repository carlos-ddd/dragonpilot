from carlosddd_logmodule import Carlosddd_Logmodule
from carlosddd_acceltest import Carlosddd_Acceltest
import random
import time

#CdddL = Carlosddd_Logmodule("Test")
#
#for i in range(0,130):
#    if i != 12:
#        CdddL.update("ki", 500.2+i)
#        CdddL.update("kp", 200.4+i)
#        CdddL.update("V", 590.88+i)
#    else:
#        CdddL.update("ki", float('nan'))
#        CdddL.update("kp", 200.4+i)
#        CdddL.update("V", 590.88+i)
#    CdddL.slice_done()


test_dict = {}
test_dict['accel_test_allowable'] = True
test_dict['accel_test_gas_schedulable'] = True
test_dict['accel_test_brake_schedulable'] = True
test_dict['accel_test_target_speed_gas'] = 0.0
test_dict['accel_test_target_speed_brake'] = 200.0
test_dict['accel_test_gas_val'] = 100
test_dict['accel_test_brake_val'] = 200


CdddA = Carlosddd_Acceltest(test_dict=test_dict)



# op disengaged, some idling
for i in range(0,5):
    print( CdddA.update(op_enabled=False, vEgo=i*3.7/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "OP disengaged")


print("############")
print("# GAS TEST #")
print("############")

# trigger gas test
test_dict['accel_test_target_speed_gas'] = 70.8
print("OPparams accel_test_target_speed_gas set to", test_dict['accel_test_target_speed_gas'])

# op disengaged
for i in range(0,5):
    print( CdddA.update(op_enabled=False, vEgo=i*3.7/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "OP disengaged")


# engage
print( CdddA.update(op_enabled=True, vEgo=33.7/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "engage OP")
# press clutch
for i in range(0,2):
    print( CdddA.update(op_enabled=True, vEgo=33.7/3.6, aEgo=random.random(), clutchPressed=True, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch pressed")
# clutch released, hopefully accelerating
for i in range(0,10):
    print( CdddA.update(op_enabled=True, vEgo=(33.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "clutch released, accelerating")
for i in range(0,3):
    print( CdddA.update(op_enabled=True, vEgo=(52.7+i*2)/3.6, aEgo=random.random(), clutchPressed=True, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch pressed, shifting")
for i in range(0,10):
    print( CdddA.update(op_enabled=True, vEgo=(60.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "clutch released, accelerating")

for i in range(0,2):
    print( CdddA.update(op_enabled=False, vEgo=(60.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "clutch released, accelerating")

print("##############")
print("# BRAKE TEST #")
print("##############")

# trigger brake test
test_dict['accel_test_target_speed_brake'] = 50.0
print("OPparams accel_test_target_speed_brake set to", test_dict['accel_test_target_speed_brake'])
# disengage, idle
for i in range(0,2):
    print( CdddA.update(op_enabled=False, vEgo=(60.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "op disengaged, idling")
# engage now
for i in range(0,3):
    print( CdddA.update(op_enabled=True, vEgo=(60.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "engageed, clutch released")
# press clutch to start
for i in range(0,3):
    print( CdddA.update(op_enabled=True, vEgo=(70.7+i*2)/3.6, aEgo=random.random(), clutchPressed=True, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch pressed")
for i in range(0,10):
    print( CdddA.update(op_enabled=True, vEgo=(82.7-i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch released")
for i in range(0,5):
    print( CdddA.update(op_enabled=True, vEgo=(52.7+i*2)/3.6, aEgo=random.random(), clutchPressed=True, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "clutch pressed for downshifting")
for i in range(0,10):
    print( CdddA.update(op_enabled=True, vEgo=(62.7-i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch released again")
# idle
for i in range(0,5):
    print( CdddA.update(op_enabled=False, vEgo=i*3.7/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "OP disengaged")

time.sleep(2.0)
print("sleeping for different log file names")

print("##############")
print("# BRAKE TEST #")
print("##############")

# trigger brake test
test_dict['accel_test_target_speed_brake'] = 49.0
print("OPparams accel_test_target_speed_brake set to", test_dict['accel_test_target_speed_brake'])
# disengage, idle
for i in range(0,2):
    print( CdddA.update(op_enabled=False, vEgo=(60.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "op disengaged, idling")
# engage now
for i in range(0,3):
    print( CdddA.update(op_enabled=True, vEgo=(60.7+i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "engageed, clutch released")
# press clutch to start
for i in range(0,3):
    print( CdddA.update(op_enabled=True, vEgo=(70.7+i*2)/3.6, aEgo=random.random(), clutchPressed=True, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch pressed")
for i in range(0,10):
    print( CdddA.update(op_enabled=True, vEgo=(82.7-i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch released")
for i in range(0,5):
    print( CdddA.update(op_enabled=True, vEgo=(52.7+i*2)/3.6, aEgo=random.random(), clutchPressed=True, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "clutch pressed for downshifting")
for i in range(0,10):
    print( CdddA.update(op_enabled=True, vEgo=(62.7-i*2)/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(800,1200), test_dict=test_dict), "clutch released again")
# idle
for i in range(0,5):
    print( CdddA.update(op_enabled=False, vEgo=i*3.7/3.6, aEgo=random.random(), clutchPressed=False, gasPressed=False, gear=0, rpm=random.randint(1200,3000), test_dict=test_dict), "OP disengaged")
