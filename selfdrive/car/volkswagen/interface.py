from cereal import car
from selfdrive.car.volkswagen.values import CAR, BUTTON_STATES, CANBUS, NetworkLocation, TransmissionType, GearShifter, PQ_CARS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.dp_common import common_interface_atl, common_interface_get_params_lqr

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.displayMetricUnitsPrev = None
    self.buttonStatesPrev = BUTTON_STATES.copy()

    # CAN bus / parser definition see interfaces.py (NOT interface.py -> generic, not car specific!)
    # there is define: cp, cp_cam, cp_body, cp_loopback
    # CS.update()
    if CP.networkLocation == NetworkLocation.fwdCamera:
      # DOES NOT APPLY FOR carlos_ddd !
      print(">>> (0) interface.py: networklocation.fwdCamera")
      self.ext_bus = CANBUS.pt  # CANBUS.pt=can0=at CAN-GW (see values.py) -> it's only the bus number (int) see values.py that is passed into carcontroller CC.update()
      self.cp_ext = self.cp

    else:   # carlos-ddd: that's where we are integrated (J533 CAN gateway)
      print(">>> (0) interface.py: networklocation-ELSE")
      # ext_bus is legacy for the ACC button hacking and is used in carcontrollser CC.update() but currently not used at all (keeping it as legacy)
      #      ^      it's only the bus number (int) see values.py that is passed into carcontroller CC.update()
      #      |
      self.ext_bus = CANBUS.cam # CANBUS.cam=can2=isolated (MFK, SWA, Tesla-radar, pedal) (see values.py)
      self.cp_ext = self.cp_cam
      #      |
      #      v
      #    cp_ext is the third bus to be parsed in carstate.py CS.update() and a 100% copy of cp_cam at the moment
   #   self.cp_ext = self.cp_body # carlos_ddd: powertrain tap

    # the parsers are updated in the update()-function (see below in this file) with the CAN strings
    # and then passed to     CS.update(..., self.cp, self.cp_cam, self.cp_ext, ...)
    #                                          |      |             /
    #                                          |      |            /
    #                                          |      |           /
    #                                          v      v         v/
    # where they are calling update_pq(..., pt_cp, cam_cp, acc_cp, ...)

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "volkswagen"

    if True:  # pylint: disable=using-constant-test

      print(">>> (1) interface.py: candidat is", candidate)
      if candidate in PQ_CARS:
        print(">>> (2) interface.py: candidat in PQ_CARS")
        # Configurations shared between all PQ35/PQ46/NMS vehicles
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volkswagenPq)]

        # Determine installed network location and trans type from fingerprint
        #ret.networkLocation = NetworkLocation.fwdCamera if 0x368 in fingerprint[0] else NetworkLocation.gateway # carlos-ddd: 0x368 = mACC_System
        ret.networkLocation = NetworkLocation.gateway # carlos-ddd forced
        print(">>> interface.py: forcing NetworkLocation:", str(ret.networkLocation))
        if 0x440 in fingerprint[0]:  # Getriebe_1
          ret.transmissionType = TransmissionType.automatic
          print(">>> (3) interface.py: automatic transmission detected")
        else:  # No trans at all
          ret.transmissionType = TransmissionType.manual
          print(">>> (3) interface.py: manual transmission detected")
        ret.enableBsm = 0x3BA in fingerprint[0]  # SWA_1
        print(">>> (4) interface.py: enableBsm based on fingerprint:", ret.enableBsm)

      else:
        print(">>> (2) interface.py: candidat is MQB (not in PQ_CARS!!!!)")
        # Set global MQB parameters
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volkswagen)]
        if 0xAD in fingerprint[0]:  # Getriebe_11
          ret.transmissionType = TransmissionType.automatic
        elif 0x187 in fingerprint[0]:  # EV_Gearshift
          ret.transmissionType = TransmissionType.direct
        else:
          ret.transmissionType = TransmissionType.manual

        if any(msg in fingerprint[1] for msg in (0x40, 0x86, 0xB2, 0xFD)):  # Airbag_01, LWI_01, ESP_19, ESP_21
          ret.networkLocation = NetworkLocation.gateway
        else:
          ret.networkLocation = NetworkLocation.fwdCamera
        ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01
      print(">>> (5) interface.py: fingerprint[0]", fingerprint[0])

    # ==================
    # Tuning PARAMETERS
    # ==================
    # all parameters listed here are OP built in values!
    # none of them are defined by user, all are listed in cereal/car.capnp
    # and probably they must be listed there!

    # ------------------
    # LATERAL PARAMETERS
    # ------------------

    # Global lateral tuning defaults, can be overridden per-vehicle
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 1.0
    ret.steerLimitTimer = 0.4
    ret.steerRatio = 15.6  # Let the params learner figure this out
    tire_stiffness_factor = 1.0  # Let the params learner figure this out

#    ret.lateralTuning.pid.kpBP = [0.]
#    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.00006
#    ret.lateralTuning.pid.kpV = [0.6] # old [0.3]
#    ret.lateralTuning.pid.kiV = [0.2] # old [0.1]

# PQ lateral tuning HCA_Status 7 carlos-ddd (old)
#            km/h                 50   126
#    ret.lateralTuning.pid.kpBP = [0., 14., 35.]
#    ret.lateralTuning.pid.kiBP = [0., 14., 35.]
#    ret.lateralTuning.pid.kpV = [0.12, 0.165, 0.185]
#    ret.lateralTuning.pid.kiV = [0.09, 0.10, 0.11]

    # PQ lateral tuning HCA_Status 7 Edgy (new)
    ret.lateralTuning.pid.kpBP = [0., 14., 20.]
    ret.lateralTuning.pid.kiBP = [0., 14., 20.]
    ret.lateralTuning.pid.kpV = [0.116, 0.13, 0.14]
    ret.lateralTuning.pid.kiV = [0.09, 0.10, 0.11]

    # -----------------------
    # LONGITUDINAL PARAMETERS
    # -----------------------

    ret.radarOffCan = False	# carlos_ddd we have tesla radar ? It's not clear if it's used but cereal/car.capnp lists it with comment: "True when radar objects aren't visible on CAN"!
    # Check for Comma Pedal
    ret.enableGasInterceptor = True                     # cereal/car.capnp

    # OP LONG parameters (https://github.com/commaai/openpilot/wiki/Tuning#Tuning-the-longitudinal-PI-controller)
    # see Edgy's impelmentation now .... not longer neede in long planer (in cereal/car.capnp marked as DEPRECCATED)!
#    ret.gasMaxBP = [0., 1.]  # m/s
#    ret.gasMaxV = [0.3, 1.0]  # max gas allowed
#    ret.brakeMaxBP = [0.]  # m/s
#    ret.brakeMaxV = [1.]  # max brake allowed (positive number)
#    ret.startAccel = 0.0

    ret.openpilotLongitudinalControl = True             # cereal/car.capnp is openpilot doing the longitudinal control?
    ret.longitudinalActuatorDelayUpperBound = 0.5       # cereal/car.capnp Gas/Brake actuator delay in seconds, upper bound
    #ret.longitudinalActuatorDelayLowerBound            # cereal/car.capnp Gas/Brake actuator delay in seconds, lower bound
    ret.stoppingControl = True                          # cereal/car.capnp Does the car allows full control even at lows speeds when stopping
    ret.vEgoStopping = 1.0                              # cereal/car.capnp Speed at which the car goes into stopping state
    #ret.vEgoStarting                                   # cereal/car.capnp Speed at which the car goes into starting state
    ret.stopAccel = -0.5                                # cereal/car.capnp Required acceleraton to keep vehicle stationary
    ret.stoppingDecelRate = 0.3                         # cereal/car.capnp m/s^2/s while trying to stop
    ret.directAccelControl = False                      # cereal/car.capnp Does the car have direct accel control or just gas/brake

    #carlos-ddd old
#    ret.longitudinalTuning.deadzoneBP = [0.]  #m/s
#    ret.longitudinalTuning.deadzoneV = [.1]  # if control-loops (internal) error value is within +/- this value -> the error is set to 0.0
#    ret.longitudinalTuning.kpBP = [2.8, 8.3, 13.8, 22.2, 33.3]  # m/s
#    ret.longitudinalTuning.kpV = [2.,   2.,  3.,   4.2,  6.]
#    ret.longitudinalTuning.kiBP = [2.8, 8.3, 13.8, 22.2, 33.3]  # m/s
#    ret.longitudinalTuning.kiV = [2.,   1.,  1.2,  3.2,  3.]

    # Edgy new
    ret.longitudinalTuning.deadzoneBP = [0.]    #m/s
    ret.longitudinalTuning.deadzoneV = [0.]     # if control-loops (internal) error value is within +/- this value -> the error is set to 0.0
    ret.longitudinalTuning.kpBP = [0.]          #m/s
    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiBP = [0.]          #m/s
    ret.longitudinalTuning.kiV = [0.03]


    # Per-chassis tuning values, override tuning defaults here if desired
    if candidate == CAR.ARTEON_MK1:
      ret.mass = 1733 + STD_CARGO_KG
      ret.wheelbase = 2.84

    elif candidate == CAR.ATLAS_MK1:
      ret.mass = 2011 + STD_CARGO_KG
      ret.wheelbase = 2.98

    elif candidate == CAR.GOLF_MK7:
      ret.mass = 1397 + STD_CARGO_KG
      ret.wheelbase = 2.62

    elif candidate == CAR.JETTA_MK7:
      ret.mass = 1328 + STD_CARGO_KG
      ret.wheelbase = 2.71

    elif candidate == CAR.PASSAT_MK8:
      ret.mass = 1551 + STD_CARGO_KG
      ret.wheelbase = 2.79

    elif candidate == CAR.POLO_MK6:
      ret.mass = 1230 + STD_CARGO_KG
      ret.wheelbase = 2.55

    elif candidate == CAR.TAOS_MK1:
      ret.mass = 1498 + STD_CARGO_KG
      ret.wheelbase = 2.69

    elif candidate == CAR.TCROSS_MK1:
      ret.mass = 1150 + STD_CARGO_KG
      ret.wheelbase = 2.60

    elif candidate == CAR.TIGUAN_MK2:
      ret.mass = 1715 + STD_CARGO_KG
      ret.wheelbase = 2.74

    elif candidate == CAR.TOURAN_MK2:
      ret.mass = 1516 + STD_CARGO_KG
      ret.wheelbase = 2.79

    elif candidate == CAR.TRANSPORTER_T61:
      ret.mass = 1926 + STD_CARGO_KG
      ret.wheelbase = 3.00  # SWB, LWB is 3.40, TBD how to detect difference
      ret.minSteerSpeed = 14.0

    elif candidate == CAR.TROC_MK1:
      ret.mass = 1413 + STD_CARGO_KG
      ret.wheelbase = 2.63

    elif candidate == CAR.AUDI_A3_MK3:
      ret.mass = 1335 + STD_CARGO_KG
      ret.wheelbase = 2.61

    elif candidate == CAR.AUDI_Q2_MK1:
      ret.mass = 1205 + STD_CARGO_KG
      ret.wheelbase = 2.61

    elif candidate == CAR.AUDI_Q3_MK2:
      ret.mass = 1623 + STD_CARGO_KG
      ret.wheelbase = 2.68

    elif candidate == CAR.SEAT_ATECA_MK1:
      ret.mass = 1900 + STD_CARGO_KG
      ret.wheelbase = 2.64

    elif candidate == CAR.SEAT_LEON_MK3:
      ret.mass = 1227 + STD_CARGO_KG
      ret.wheelbase = 2.64

    elif candidate == CAR.SKODA_KAMIQ_MK1:
      ret.mass = 1265 + STD_CARGO_KG
      ret.wheelbase = 2.66

    elif candidate == CAR.SKODA_KAROQ_MK1:
      ret.mass = 1278 + STD_CARGO_KG
      ret.wheelbase = 2.66

    elif candidate == CAR.SKODA_KODIAQ_MK1:
      ret.mass = 1569 + STD_CARGO_KG
      ret.wheelbase = 2.79

    elif candidate == CAR.SKODA_OCTAVIA_MK3:
      ret.mass = 1388 + STD_CARGO_KG
      ret.wheelbase = 2.68

    elif candidate == CAR.SKODA_SCALA_MK1:
      ret.mass = 1192 + STD_CARGO_KG
      ret.wheelbase = 2.65

    elif candidate == CAR.SKODA_SUPERB_MK3:
      ret.mass = 1505 + STD_CARGO_KG
      ret.wheelbase = 2.84

    elif candidate == CAR.GENERICPQ:
      print(">>> (6) interface.py: candidat is in CAR.GENERICPQ (usable)")
      ret.mass = 1375 + STD_CARGO_KG  # Average, varies on trim/package
      ret.wheelbase = 2.58
      ret.steerRatio = 15.6
      tire_stiffness_factor = 1.0

    else:
      raise ValueError(f"unsupported car {candidate}")
      print(">>> (6) interface.py: unsupported car (else branch)")

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.centerToFront = ret.wheelbase * 0.45
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    # dp
    ret = common_interface_get_params_lqr(ret)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings, dragonconf):
    buttonEvents = []

    # Process the most recent CAN message traffic, and check for validity
    # The camera CAN has no signals we use at this time, but we process it
    # anyway so we can test connectivity with can_valid
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
    #print(str(can_strings))
#    if self.cp_body:
#      self.cp_body.update_string(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_ext, self.CP.transmissionType)
#    ret = self.CS.update(self.cp, self.cp_cam, self.cp_body, self.CP.transmissionType)

    # dp
    self.dragonconf = dragonconf
    ret.cruiseState.enabled = common_interface_atl(ret, dragonconf.dpAtl)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid# and (self.cp_body is None or self.cp_body.can_valid)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    events = self.create_common_events(ret, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic])

    # Vehicle health and operation safety checks
    if self.CS.parkingBrakeSet:
      events.add(EventName.parkBrake)
    if self.CS.tsk_status in (6, 7):
      events.add(EventName.accFaulted)

    # Low speed steer alert hysteresis logic
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    ret.buttonEvents = buttonEvents

    # update previous car states
    self.displayMetricUnitsPrev = self.CS.displayMetricUnits
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c, c.enabled, self.CS, self.frame, self.ext_bus, c.actuators,
                         hud_control.visualAlert,
                         hud_control.audibleAlert,
                         hud_control.leftLaneVisible,
                         hud_control.rightLaneVisible,
                         hud_control.leftLaneDepart,
                         hud_control.rightLaneDepart,
                         self.dragonconf)
    self.frame += 1
    return ret
