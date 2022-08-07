import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams
from selfdrive.car.volkswagen.PQacc_module import PQacc

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.ACC = PQacc()

    ### START OF MAIN CONFIG OPTIONS ###
    ### Do NOT modify here, modify in /data/bb_openpilot.cfg and reboot
    self.useTeslaRadar = CP.enableGasInterceptor
    print(">>> (0) carstate.py::useTeslaRadar:", self.useTeslaRadar)
    self.radarVIN = "5YJSB7E17HF207544" # carlos_ddd
    self.radarOffset = 0.
    self.radarPosition = 1
    self.radarEpasType = 3
    ### END OF MAIN CONFIG OPTIONS ###


    ## == ##
    ## PQ ##
    ## == ##
    if CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.volkswagenPq:
      can_define = CANDefine(DBC_FILES.pq46)
      # Configure for PQ35/PQ46/NMS network messaging
      # following functions are called / defined by selfdrive/car/interfaces.py (not volkswagen but generic!):
      # CS.get_can_parser() -> self.cp
      # CS.get_cam_can_parser() -> self.cp_cam
      self.get_can_parser = self.get_pq_can_parser
      self.get_cam_can_parser = self.get_pq_cam_can_parser
      # [CS.get_body_can_parser() -> self.cp_body]
      # [CS.get_loopback_can_parser() -> I guess only needed for testing]
      self.update = self.update_pq
      print(">>>  (1) carstate.py: safety model PQ")

      if CP.transmissionType == TransmissionType.automatic:
        self.shifter_values = can_define.dv["Getriebe_1"]['Waehlhebelposition__Getriebe_1_']
        print(">>> (2) carstate.py: automatic gear detected")
      elif CP.transmissionType == TransmissionType.manual:
        print(">>> (2) carstate.py: manual gear detected")

      if CP.enableGasInterceptor:
        print(">>> (3) carstate.py: enableGasInterceptor is true")
        self.openpilot_enabled = False
      else:
        print(">>> (3) carstate.py: enableGasInterceptor is false!")


    ## === ##
    ## MQB ##
    ## === ##
    else:
      print(">>> (1) carstate.py: safety model MQB")
      can_define = CANDefine(DBC_FILES.mqb)

      self.get_can_parser = self.get_mqb_can_parser
      self.get_cam_can_parser = self.get_mqb_cam_can_parser
      self.update = self.update_mqb

      if CP.transmissionType == TransmissionType.automatic:
        self.shifter_values = can_define.dv["Getriebe_11"]["GE_Fahrstufe"]
      elif CP.transmissionType == TransmissionType.direct:
        self.shifter_values = can_define.dv["EV_Gearshift"]["GearPosition"]
      self.hca_status_values = can_define.dv["LH_EPS_03"]["EPS_HCA_Status"]
    self.buttonStates = BUTTON_STATES.copy()
    self.tsk_status = 0
    self.ldw_stock_values = {}

  def update_mqb(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # dp addition - Fix stop and go acc self-resume +1
    ret.standstill = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"]) and ret.vEgoRaw < 0.01

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    hca_status = self.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerError = hca_status in ("DISABLED", "FAULT")
    ret.steerWarning = hca_status in ("INITIALIZING", "REJECTED")

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    self.esp_hold_confirmation = pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"]
    ret.brakeLights = bool(pt_cp.vl["ESP_05"]['ESP_Status_Bremsdruck'])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    elif trans_type == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["EV_Gearshift"]["GearPosition"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_14"]["MO_Kuppl_schalter"]
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_01"]["KBI_MFA_v_Einheit_02"]

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = False #bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = False #bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
    ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    # Update ACC radar status.
    self.tsk_status = pt_cp.vl["TSK_06"]["TSK_Status"]
    if self.tsk_status == 2:
      # ACC okay and enabled, but not currently engaged
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    elif self.tsk_status in (3, 4, 5):
      # ACC okay and enabled, currently regulating speed (3) or driver accel override (4) or overrun coast-down (5)
      ret.cruiseState.available = True
      ret.cruiseState.enabled = True
    else:
      # ACC okay but disabled (1), or a radar visibility or other fault/disruption (6 or 7)
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False
    # dp
    ret.cruiseActualEnabled = ret.cruiseState.enabled

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise:
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw"] * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Hoch"])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Runter"])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Abbrechen"])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Setzen"])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Wiederaufnahme"])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Verstellung_Zeitluecke"])
    ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
    ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["GRA_ACC_01"]["GRA_Hauptschalter"]
    self.graTypHauptschalter = pt_cp.vl["GRA_ACC_01"]["GRA_Typ_Hauptschalter"]
    self.graButtonTypeInfo = pt_cp.vl["GRA_ACC_01"]["GRA_ButtonTypeInfo"]
    self.graTipStufe2 = pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Stufe_2"]
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["GRA_ACC_01"]["COUNTER"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0

    return ret

  # called by selfdrive/car/volkswagen/interface.py: CS.update(self.cp, self.cp_cam, self.cp_ext, ...)
  def update_pq(self, pt_cp, cam_cp, acc_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["Bremse_3"]['Radgeschw__VL_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["Bremse_3"]['Radgeschw__VR_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["Bremse_3"]['Radgeschw__HL_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["Bremse_3"]['Radgeschw__HR_4_1'] * CV.KPH_TO_MS

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]['LH3_BLW'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_BLWSign'])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit'] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit_S'])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]['LH3_LM'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_LMSign'])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]['Giergeschwindigkeit'] * (1, -1)[int(pt_cp.vl["Bremse_5"]['Vorzeichen_der_Giergeschwindigk'])] * CV.DEG_TO_RAD

    # Update gas, brakes, and gearshift.
    if not self.CP.enableGasInterceptor:
      ret.gas = pt_cp.vl["Motor_3"]['Fahrpedal_Rohsignal'] / 100.0
      ret.gasPressed = ret.gas > 0
    else:
      self.gasInterceptorSensor1 = cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']
      self.gasInterceptorSensor2 = cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']
      ret.gas = (self.gasInterceptorSensor1 + self.gasInterceptorSensor2) / 2
      #ret.gas = (cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 468
      #print(">>> carstate.py gasInterceptorSensor1:", self.gasInterceptorSensor1, "gasInterceptorSensor2", self.gasInterceptorSensor2)

    ret.brake = pt_cp.vl["Bremse_5"]['Bremsdruck'] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]['Bremstestschalter'])

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_1"]['Bremsinfo'])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]['ESP_Passiv_getastet'])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_1"]['Waehlhebelposition__Getriebe_1_'], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]['Kupplungsschalter']
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Rueckfahr'])
      # TODO: verify this synthesis has the desired behavior.
      # In particular, should we neutral disengage during normal gear changes?
      # Also, what happens if we're rolling backwards with the clutch pressed
      # in a gear other than reverse?
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      elif ret.standstill and self.parkingBrakeSet:
        ret.gearShifter = GearShifter.park
      elif ret.clutchPressed:
        ret.gearShifter = GearShifter.neutral
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Fa_Tuerkont'])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_1"]["MFA_v_Einheit_02"]

    self.engineRPM = pt_cp.vl["Motor_1"]['Motordrehzahl']

    # Consume blind-spot monitoring info/warning LED states, if available. The
    # info signal (LED on) is enabled whenever a vehicle is detected in the
    # driver's blind spot. The warning signal (LED flashing) is enabled if the
    # driver shows possibly hazardous intent toward a BSM detected vehicle, by
    # setting the turn signal in that direction, or (for cars with factory Lane
    # Assist) approaches the lane boundary in that direction. Size of the BSM
    # detection box is dynamic based on speed and road curvature.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assist Systems,
    # pages 32-35.
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cam_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(cam_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(cam_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(cam_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_1"]# if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
    # carlos-ddd: might be same-ish on PQ!
    #ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    #ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])
    
    # collect HCA_1 steering-data from stock HCA camera (MFK)
    self.stock_HCA_Status = int(cam_cp.vl["HCA_1"]['HCA_Status'])
    self.stock_HCA_LM_Offset = int(cam_cp.vl["HCA_1"]['LM_Offset'])
    self.stock_HCA_LM_OffSign = bool(cam_cp.vl["HCA_1"]['LM_OffSign'])
    self.stock_HCA_SteeringVal =  int(-1 * stock_HCA_LM_Offset) if stock_HCA_LM_OffSign else stock_HCA_LM_Offset    # if sign bit is set -> neg. value

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = acc_cp.vl["ACC_GRA_Anziege"]['ACA_V_Wunsch'] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Up_kurz'])# or bool(pt_cp.vl["GRA_Neu"]['GRA_Up_lang'])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Down_kurz'])# or bool(pt_cp.vl["GRA_Neu"]['GRA_Down_lang'])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Abbrechen'])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Neu_Setzen'])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Recall'])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Zeitluecke'])
    self.buttonStates["longUp"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Up_lang'])
    self.buttonStates["longDown"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Down_lang'])
    # Update ACC radar status.
    # FIXME: This is unfinished and not fully correct, need to improve further
    ret.cruiseState.available = bool(pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt'])
    ret.cruiseState.enabled = True if pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2] else False
    # ACC emulation (overwriting cruiseState-values from CAN)
    self.ACC_engaged, self.v_ACC = self.ACC.update_acc_iter_4CS(ret.vEgo*CV.MS_TO_KPH, self.buttonStates, ret.cruiseState.available, self.openpilot_enabled, ret.cruiseState.enabled)
    # Engage open pilot if ACC emulation says so
    if self.CP.enableGasInterceptor and self.ACC_engaged:
      self.openpilot_enabled = True
    # if ACC emulation is not active we want OP still engaged but setspeed <= current speed
    elif self.CP.enableGasInterceptor and not self.ACC_engaged:
      self.v_ACC = (ret.vEgo - 1) * CV.MS_TO_KPH
    else:
      self.openpilot_enabled = False

    # Check if Gas or Brake pressed and override ACC emulation
    if self.CP.enableGasInterceptor and (ret.gasPressed or ret.brakePressed):
      self.openpilot_enabled = False

    # Override openpilot enabled if gas interceptor installed
    if self.CP.enableGasInterceptor and self.openpilot_enabled:
      ret.cruiseState.enabled = True
    else:
      ret.cruiseState.enabled = False

    ret.cruiseState.speed = self.v_ACC * CV.KPH_TO_MS
    if ret.clutchPressed:                                           # during clutch open do not try to accelerate
      ret.cruiseState.speed = min(ret.vEgo, ret.cruiseState.speed)  # -> neutral speed setpoint but do not increase
                                                                    #    (to not prevent braking with clutch open)      
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0
    
    # blinke / turn indicator
    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_li'])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_re'])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    # TODO: Check to see what info we need to passthru and spoof on PQ
    self.graHauptschalter = pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt']
    self.graTypHauptschalter = False
    self.graButtonTypeInfo = False
    self.graTipStufe2 = False
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    # FIXME: will need msg counter and checksum algo to spoof GRA_neu
    self.graMsgBusCounter = 0
    self.esp_hold_confirmation = 0

    # Check to make sure the electric power steering rack is configured to
    # accept and respond to HCA_01 messages and has not encountered a fault.
    self.steeringFault = pt_cp.vl["Lenkhilfe_2"]['LH2_Sta_HCA'] not in [1, 3, 5, 7] # 1 missing in newer configs ???? carlos_ddd

    # Read ABS pump for checking in ACC braking is working.
    if self.CP.enableGasInterceptor:
      self.ABSWorking = pt_cp.vl["Bremse_8"]["BR8_Sta_ADR_BR"]
      self.currentSpeed = ret.vEgo

    return ret

  @staticmethod
  def get_mqb_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("LWI_Lenkradwinkel", "LWI_01"),           # Absolute steering angle
      ("LWI_VZ_Lenkradwinkel", "LWI_01"),        # Steering angle sign
      ("LWI_Lenkradw_Geschw", "LWI_01"),         # Absolute steering rate
      ("LWI_VZ_Lenkradw_Geschw", "LWI_01"),      # Steering rate sign
      ("ESP_VL_Radgeschw_02", "ESP_19"),         # ABS wheel speed, front left
      ("ESP_VR_Radgeschw_02", "ESP_19"),         # ABS wheel speed, front right
      ("ESP_HL_Radgeschw_02", "ESP_19"),         # ABS wheel speed, rear left
      ("ESP_HR_Radgeschw_02", "ESP_19"),         # ABS wheel speed, rear right
      ("ESP_Gierrate", "ESP_02"),                # Absolute yaw rate
      ("ESP_VZ_Gierrate", "ESP_02"),             # Yaw rate sign
      ("ZV_FT_offen", "Gateway_72"),             # Door open, driver
      ("ZV_BT_offen", "Gateway_72"),             # Door open, passenger
      ("ZV_HFS_offen", "Gateway_72"),            # Door open, rear left
      ("ZV_HBFS_offen", "Gateway_72"),           # Door open, rear right
      ("ZV_HD_offen", "Gateway_72"),             # Trunk or hatch open
      ("Comfort_Signal_Left", "Blinkmodi_02"),   # Left turn signal including comfort blink interval
      ("Comfort_Signal_Right", "Blinkmodi_02"),  # Right turn signal including comfort blink interval
      ("AB_Gurtschloss_FA", "Airbag_02"),        # Seatbelt status, driver
      ("AB_Gurtschloss_BF", "Airbag_02"),        # Seatbelt status, passenger
      ("ESP_Fahrer_bremst", "ESP_05"),           # Brake pedal pressed
      ("ESP_Status_Bremsdruck", "ESP_05"),       # Brakes applied
      ("ESP_Bremsdruck", "ESP_05"),              # Brake pressure applied
      ("MO_Fahrpedalrohwert_01", "Motor_20"),    # Accelerator pedal value
      ("EPS_Lenkmoment", "LH_EPS_03"),           # Absolute driver torque input
      ("EPS_VZ_Lenkmoment", "LH_EPS_03"),        # Driver torque input sign
      ("EPS_HCA_Status", "LH_EPS_03"),           # EPS HCA control status
      ("ESP_Tastung_passiv", "ESP_21"),          # Stability control disabled
      ("ESP_Haltebestaetigung", "ESP_21"),       # ESP hold confirmation
      ("KBI_MFA_v_Einheit_02", "Einheiten_01"),  # MPH vs KMH speed display
      ("KBI_Handbremse", "Kombi_01"),            # Manual handbrake applied
      ("TSK_Status", "TSK_06"),                  # ACC engagement status from drivetrain coordinator
      ("GRA_Hauptschalter", "GRA_ACC_01"),       # ACC button, on/off
      ("GRA_Abbrechen", "GRA_ACC_01"),           # ACC button, cancel
      ("GRA_Tip_Setzen", "GRA_ACC_01"),          # ACC button, set
      ("GRA_Tip_Hoch", "GRA_ACC_01"),            # ACC button, increase or accel
      ("GRA_Tip_Runter", "GRA_ACC_01"),          # ACC button, decrease or decel
      ("GRA_Tip_Wiederaufnahme", "GRA_ACC_01"),  # ACC button, resume
      ("GRA_Verstellung_Zeitluecke", "GRA_ACC_01"),  # ACC button, time gap adj
      ("GRA_Typ_Hauptschalter", "GRA_ACC_01"),   # ACC main button type
      ("GRA_Tip_Stufe_2", "GRA_ACC_01"),         # unknown related to stalk type
      ("GRA_ButtonTypeInfo", "GRA_ACC_01"),      # unknown related to stalk type
      ("COUNTER", "GRA_ACC_01"),                 # GRA_ACC_01 CAN message counter
    ]

    checks = [
      # sig_address, frequency
      ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
      ("LH_EPS_03", 100),   # From J500 Steering Assist with integrated sensors
      ("ESP_19", 100),      # From J104 ABS/ESP controller
      ("ESP_05", 50),       # From J104 ABS/ESP controller
      ("ESP_21", 50),       # From J104 ABS/ESP controller
      ("Motor_20", 50),     # From J623 Engine control module
      ("TSK_06", 50),       # From J623 Engine control module
      ("ESP_02", 50),       # From J104 ABS/ESP controller
      ("GRA_ACC_01", 33),   # From J533 CAN gateway (via LIN from steering wheel controls)
      ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 2),      # From J285 Instrument cluster
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("Einheiten_01", 1),  # From J??? not known if gateway, cluster, or BCM
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals.append(("GE_Fahrstufe", "Getriebe_11"))  # Auto trans gear selector position
      checks.append(("Getriebe_11", 20))  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.direct:
      signals.append(("GearPosition", "EV_Gearshift"))  # EV gear selector position
      checks.append(("EV_Gearshift", 10))  # From J??? unknown EV control module
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("MO_Kuppl_schalter", "Motor_14"),  # Clutch switch
                  ("BCM1_Rueckfahrlicht_Schalter", "Gateway_72")]  # Reverse light from BCM
      checks.append(("Motor_14", 10))  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Radars are here on CANBUS.pt
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    checks = []
    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.pt, False)

  @staticmethod
  def get_pq_can_parser(CP):
    # carlos-ddd: this would be the CAN parser for signals on extended CAN on the GATEWAY-side of panda (can0)
    signals = [
      # sig_name, sig_address, default
      ("LH3_BLW", "Lenkhilfe_3"),                # Absolute steering angle
      ("LH3_BLWSign", "Lenkhilfe_3"),            # Steering angle sign
      ("LH3_LM", "Lenkhilfe_3"),                 # Absolute driver torque input
      ("LH3_LMSign", "Lenkhilfe_3"),             # Driver torque input sign
      ("LH2_Sta_HCA", "Lenkhilfe_2"),            # Steering rack HCA status
      ("Lenkradwinkel_Geschwindigkeit", "Lenkwinkel_1"),  # Absolute steering rate
      ("Lenkradwinkel_Geschwindigkeit_S", "Lenkwinkel_1"),  # Steering rate sign
      ("Radgeschw__VL_4_1", "Bremse_3"),         # ABS wheel speed, front left
      ("Radgeschw__VR_4_1", "Bremse_3"),         # ABS wheel speed, front right
      ("Radgeschw__HL_4_1", "Bremse_3"),         # ABS wheel speed, rear left
      ("Radgeschw__HR_4_1", "Bremse_3"),         # ABS wheel speed, rear right
      ("Giergeschwindigkeit", "Bremse_5"),       # Absolute yaw rate
      ("Vorzeichen_der_Giergeschwindigk", "Bremse_5"),  # Yaw rate sign
      ("GK1_Fa_Tuerkont", "Gate_Komf_1"),     # Door open, driver
      # TODO: locate passenger and rear door states
      ("GK1_Blinker_li", "Gate_Komf_1"),         # Left turn signal on
      ("GK1_Blinker_re", "Gate_Komf_1"),         # Right turn signal on
      ("Gurtschalter_Fahrer", "Airbag_1"),       # Seatbelt status, driver
      ("Gurtschalter_Beifahrer", "Airbag_1"),    # Seatbelt status, passenger
      ("Bremstestschalter", "Motor_2"),          # Brake pedal pressed (brake light test switch)
      ("Bremslichtschalter", "Motor_2"),         # Brakes applied (brake light switch)
      ("Bremsdruck", "Bremse_5"),                # Brake pressure applied
      ("Vorzeichen_Bremsdruck", "Bremse_5"),     # Brake pressure applied sign (???)
      ("Fahrpedal_Rohsignal", "Motor_3"),        # Accelerator pedal value
      ("ESP_Passiv_getastet", "Bremse_1"),       # Stability control disabled
      ("MFA_v_Einheit_02", "Einheiten_1"),       # MPH vs KMH speed display
      ("Bremsinfo", "Kombi_1"),                  # Manual handbrake applied
      ("GRA_Status", "Motor_2"),                 # ACC engagement status
      ("Soll_Geschwindigkeit_bei_GRA_Be", "Motor_2"),  # ACC speed setpoint from ECU??? check this
      ("GRA_Hauptschalt", "GRA_Neu"),              # ACC button, on/off
      ("GRA_Abbrechen", "GRA_Neu"),              # ACC button, cancel
      ("GRA_Neu_Setzen", "GRA_Neu"),             # ACC button, set
      ("GRA_Up_lang", "GRA_Neu"),                # ACC button, increase or accel, long press
      ("GRA_Down_lang", "GRA_Neu"),              # ACC button, decrease or decel, long press
      ("GRA_Up_kurz", "GRA_Neu"),                # ACC button, increase or accel, short press
      ("GRA_Down_kurz", "GRA_Neu"),              # ACC button, decrease or decel, short press
      ("GRA_Recall", "GRA_Neu"),                 # ACC button, resume
      ("GRA_Zeitluecke", "GRA_Neu"),             # ACC button, time gap adj
      ("BR8_Sta_ADR_BR", "Bremse_8"),            # ABS Pump actively braking for ACC
      ("ESP_Eingriff", "Bremse_1"),              # ABS stability intervention
      ("ASR_Anforderung", "Bremse_1"),           # ABS slip detected
      ("Motordrehzahl", "Motor_1"),              # engine RPM
    ]

    checks = [
      # sig_address, frequency
      ("Bremse_3", 100),          # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),           # From J623 Engine control module
      ("Airbag_1", 50),           # From J234 Airbag control module
      ("Bremse_5", 50),           # From J104 ABS/ESP controller
      ("Bremse_8", 50),           # From J??? ABS/ACC controller
      ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      ("Kombi_1", 50),            # From J285 Instrument cluster
      ("Motor_2", 50),            # From J623 Engine control module
      ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),        # From J533 CAN gateway
      ("Einheiten_1", 1),         # From J??? cluster or gateway
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals += [("Waehlhebelposition__Getriebe_1_", "Getriebe_1")]  # Auto trans gear selector position
      checks += [("Getriebe_1", 100)]  # From J743 Auto transmission control module
      print(">>> (4) carstate.py: CAN checks for automatic transmission added!")
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("Kupplungsschalter", "Motor_1"),  # Clutch switch
                  ("GK1_Rueckfahr", "Gate_Komf_1")]  # Reverse light from BCM
      checks += [("Motor_1", 100)]  # From J623 Engine control module
#      checks += [("Getriebe_2", 100)]  # From J623 Engine control module (due to manual shift, there is no TCU so ECU emits it)
      print(">>> (4) carstate.py: CAN checks for manual transmission added!")

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # The ACC radar is here on CANBUS.pt
      signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege")]  # ACC set speed
      checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module
      print(">>> (5) carstate.py: CAN checks for NetworkLocation.fwdCamera added!")

      if CP.enableBsm:
        signals += PqExtraSignals.bsm_radar_signals
        checks += PqExtraSignals.bsm_radar_checks
        print(">>> (6) carstate.py: CAN checks for blindspot added!")

    #checks = []
    print(">>> (7) carstate.py: CAN checks and signals for get_pq_can_parser() (not get_pq_cam_can_parser()):")
    print(">>> carstate.py::pq::signals", signals)
    print(">>> carstate.py::pq::checks", checks)
    return CANParser(DBC_FILES.pq46, signals, checks, CANBUS.pt, False)    # -> assign to CANBUS.pt (can0 see values.py)

  @staticmethod
  def get_mqb_cam_can_parser(CP):
    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        # sig_name, sig_address
        ("LDW_SW_Warnung_links", "LDW_02"),      # Blind spot in warning mode on left side due to lane departure
        ("LDW_SW_Warnung_rechts", "LDW_02"),     # Blind spot in warning mode on right side due to lane departure
        ("LDW_Seite_DLCTLC", "LDW_02"),          # Direction of most likely lane departure (left or right)
        ("LDW_DLC", "LDW_02"),                   # Lane departure, distance to line crossing
        ("LDW_TLC", "LDW_02"),                   # Lane departure, time to line crossing
      ]
      checks += [
        # sig_address, frequency
        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    checks = []
    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.cam, False)

  @staticmethod
  def get_pq_cam_can_parser(CP):
    # carlos-ddd: this would be the CAN parser for signals on extended CAN on the comma pedal, MFK, SWA, ... -side of panda (can2) -> the "isolated" part
    
    # TODO: Need to monitor LKAS camera, if present, for TLC/DLC/warning signals for passthru to SWA
    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.gateway:   # see selfdrive/car/volkswagen/interface.py ~line40
      print(">>> (8) carstate.py::get_pq_cam_can_parser(): network location gateway confirmed!")
      # The ACC radar is here on CANBUS.cam
      signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege")]  # ACC set speed
      checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module
      if CP.enableBsm:
        print(">>> (9) carstate.py::get_pq_cam_can_parser(): blindspot enabled (gateway) checks added")
        signals += PqExtraSignals.bsm_radar_signals
        checks += PqExtraSignals.bsm_radar_checks
    else:
      print(">>> (8) carstate.py::get_pq_cam_can_parser(): network location is NOT gateway !!!")

    if CP.enableGasInterceptor:
      print(">>> (10) carstate.py::get_pq_cam_can_parser(): enableGasInterceptor, checks added ")
      signals += [("INTERCEPTOR_GAS", "GAS_SENSOR"), ("INTERCEPTOR_GAS2", "GAS_SENSOR")]
      checks += [("GAS_SENSOR", 50)]

    signals += [("HCA_Status", "HCA_1"), 
                ("LM_Offset", "HCA_1"),
                ("LM_OffSign", "HCA_1"),
                ]

# wee need a flag like CP.enableBsm (needs to be in car.capnp) for stockHCA present
#    checks += [("HCA_1", 50),
#               ("LDW_1", 50),
#               ]

    #checks = []
    print(">>> (11) carstate.py::get_pq_cam_can_parser(): CAN checks and signals (get_pq_cam_can_parser()):")
    print(">>> carstate.py::pq::signals", signals)
    print(">>> carstate.py::pq::checks", checks)
    return CANParser(DBC_FILES.pq46, signals, checks, CANBUS.cam, False)    # -> assign to CANBUS.cam (can2 see values.py)

class PqExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACA_StaACC", "ACC_GRA_Anziege"),           # ACC drivetrain coordinator status
    ("ACA_V_Wunsch", "ACC_GRA_Anziege"),         # ACC set speed
  ]
  fwd_radar_checks = [
    ("ACC_GRA_Anziege", 25),                        # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_1"),           # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_1"),             # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_1"),           # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_1"),             # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_1", 20),                                  # From J1086 Lane Change Assist
  ]

class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACC_Wunschgeschw", "ACC_02"),              # ACC set speed
    ("AWV2_Freigabe", "ACC_10"),                 # FCW brake jerk release
    ("ANB_Teilbremsung_Freigabe", "ACC_10"),     # AEB partial braking release
    ("ANB_Zielbremsung_Freigabe", "ACC_10"),     # AEB target braking release
  ]
  fwd_radar_checks = [
    ("ACC_10", 50),                                 # From J428 ACC radar control module
    ("ACC_02", 17),                                 # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_01"),          # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_01"),            # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_01"),          # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_01"),            # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_01", 20),                                 # From J1086 Lane Change Assist
  ]
