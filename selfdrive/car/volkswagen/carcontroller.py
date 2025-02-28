from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import PQ_CARS, DBC_FILES, CANBUS, MQB_LDW_MESSAGES, BUTTON_STATES, PQ_LDW_MESSAGES, CarControllerParams as P
from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl
from carlosddd.carlosddd_logmodule import Carlosddd_Logmodule
from carlosddd.carlosddd_acceltest import Carlosddd_Acceltest
from carlosddd.carlosddd_accellearner import Carlosddd_Accellearner
from selfdrive.controls.lib.pid import PIController
from common.realtime import DT_CTRL

VisualAlert = car.CarControl.HUDControl.VisualAlert
#AudibleAlert = car.CarControl.HUDControl.AudibleAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.

    self.apply_steer_last = 0

    self.mobPreEnable = False
    self.mobEnabled = False
    self.mobBrakingProhibited = False

    self.radarVin_idx = 0

    self.coastingAccel = -0.42735

    self.packer_pt = CANPacker(DBC_FILES.mqb)

    if CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.volkswagen:
      print(">>> (1) carcontroller.py: safety model is MQB (volkswagen)")
      self.packer_pt = CANPacker(DBC_FILES.mqb)
      self.create_steering_control = volkswagencan.create_mqb_steering_control
      self.create_acc_buttons_control = volkswagencan.create_mqb_acc_buttons_control
      self.create_hud_control = volkswagencan.create_mqb_hud_control
    elif CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.volkswagenPq:
      print(">>> (1) carcontroller.py: safety model is PQ (volkswagen)")
      self.packer_pt = CANPacker(DBC_FILES.pq46)
      self.create_steering_control = volkswagencan.create_pq_steering_control
#      self.create_acc_buttons_control = volkswagencan.create_pq_acc_buttons_control
      self.create_hud_control = volkswagencan.create_pq_hud_control
      self.create_braking_control = volkswagencan.create_pq_braking_control
      self.create_gas_control = volkswagencan.create_pq_pedal_control
      self.create_awv_control = volkswagencan.create_pq_awv_control
#      self.create_aca_control = volkswagencan.create_pq_aca_control
      self.create_opsta_control = volkswagencan.create_pq_opsta_control

    self.hcaSameTorqueCount = 0
    self.hcaEnabledFrameCount = 0
    self.graButtonStatesToSend = None
    self.graMsgSentCount = 0
    self.graMsgStartFramePrev = 0
    self.graMsgBusCounterPrev = 0

    self.steer_rate_limited = False

    #self.CdddL = Carlosddd_Logmodule("LoC")
    self.CdddA = Carlosddd_Acceltest()
    #self.CdddAL = Carlosddd_Accellearner()
    
    self.pid = PIController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                            (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                            rate=1 / DT_CTRL)
                            #self.pid.reset()
                            #self.pid.neg_limit = accel_limits[0]
                            #self.pid.pos_limit = accel_limits[1]
                            #output_accel = self.pid.update(self.v_pid, CS.vEgo, speed=CS.vEgo, deadzone=deadzone, feedforward=a_target, freeze_integrator=freeze_integrator)


  def update(self, c, enabled, CS, frame, ext_bus, actuators, visual_alert, audible_alert, left_lane_visible, right_lane_visible, left_lane_depart, right_lane_depart, dragonconf):
    """ Controls thread """

    #CdddL_hook = True # log every function call, set to False to only have when gas / brake CAN message is sent

    cddda_apply_gas, cddda_apply_brake, cddda_active = self.CdddA.update(enabled, CS.out.vEgo, CS.out.aEgo, CS.out.clutchPressed, CS.out.gasPressed, CS.detected_gear, CS.out.engineRPM)

    # Send CAN commands.
    can_sends = []
    
    # define for CdddAL
    apply_gas = 0
    apply_brake = 0

    #--------------------------------------------------------------------------
    #                                                                         #
    # Prepare HCA_01 Heading Control Assist messages with steering torque.    #
    #                                                                         #
    #--------------------------------------------------------------------------

    # The factory camera sends at 50Hz while steering and 1Hz when not. When
    # OP is active, Panda filters HCA_01 from the factory camera and OP emits
    # HCA_01 at 50Hz. Rate switching creates some confusion in Cabana and
    # doesn't seem to add value at this time. The rack will accept HCA_01 at
    # 100Hz if we want to control at finer resolution in the future.
    if frame % P.HCA_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # One frame of HCA disabled is enough to reset the timer, without zeroing the
      # torque value. Do that anytime we happen to have 0 torque, or failing that,
      # when exceeding ~1/3 the 360 second timer.

      #if enabled and not (CS.out.standstill or CS.steeringFault): # K2: old v0.8.2
      if c.active and CS.out.vEgo > CS.CP.minSteerSpeed and not (CS.out.standstill or CS.out.steerError or CS.out.steerWarning):
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
        if apply_steer == 0:
          hcaEnabled = False
          self.hcaEnabledFrameCount = 0
        else:
          self.hcaEnabledFrameCount += 1
          if self.hcaEnabledFrameCount >= 118 * (100 / P.HCA_STEP):  # 118s
            hcaEnabled = False
            self.hcaEnabledFrameCount = 0
          else:
            hcaEnabled = True
            if self.apply_steer_last == apply_steer:
              self.hcaSameTorqueCount += 1
              if self.hcaSameTorqueCount > 1.9 * (100 / P.HCA_STEP):  # 1.9s
                apply_steer -= (1, -1)[apply_steer < 0]
                self.hcaSameTorqueCount = 0
            else:
              self.hcaSameTorqueCount = 0
      else:
        hcaEnabled = False
        apply_steer = 0

      # when OP is not active, but stock wants to steer -- we let it steer!
      stock_hca_status =  CS.stock_HCA_Status
      stock_apply_steer = CS.stock_HCA_SteeringVal

      # dp (carlos_ddd: looks like DP is checking some GUI option (like stop steering on blinker_on by zeroing apply_steer))
      blinker_on = CS.out.leftBlinker or CS.out.rightBlinker
      if not enabled:
        self.blinker_end_frame = 0
      if self.last_blinker_on and not blinker_on:
        self.blinker_end_frame = frame + dragonconf.dpSignalOffDelay
      apply_steer = common_controller_ctrl(enabled,
                                           dragonconf,
                                           blinker_on or frame < self.blinker_end_frame,
                                           apply_steer, CS.out.vEgo)
      self.last_blinker_on = blinker_on

      self.apply_steer_last = apply_steer
      idx = (frame / P.HCA_STEP) % 16
      can_sends.append(self.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer,
                                                                 idx, hcaEnabled, stock_hca_status, stock_apply_steer))


    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare PQ_MOB for sending the braking message                          #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.MOB_STEP == 0) and CS.CP.enableGasInterceptor:
      mobEnabled = self.mobEnabled
      mobPreEnable = self.mobPreEnable
      mobBrakeScaling = 150
      mobBrakeMax = min(8190, 8190)
      # TODO make sure we use the full 8190 when calculating braking.
      
      #if actuators.accel < 0:   # 0 ... -3.5m/s^2   (braking = neg. acceleration)
      if actuators.accel < self.coastingAccel:
        apply_brake = abs(actuators.accel)
        apply_brake *= mobBrakeScaling
      else:
        apply_brake = 0

      # acceleration test module output
      if cddda_active:  # overwrite
        apply_brake = cddda_apply_brake

      stopping_wish = False

      # safety to stop any braking when gas pedal is pressed
      # only needed for no-disengage-on-gas situations
      if CS.out.gasPressed:
        # if OP wants to brake during gas pedal pressed
        self.mobBrakingProhibited = True
      elif self.mobBrakingProhibited:
        # gas not pressed (any more) but has been
        # now to reenable braking we need to have reached a safe (soft) braking region
        # or the release of the gas pedal could make for an immediate hard braking
        # which has (obviously) built up in the background (remember: OP wanted to brake)
        if (apply_brake < 200) and (actuators.accel > -0.5):    # MOB-raw, ms/2
          self.mobBrakingProhibited = False     # allow braking again

      if enabled: # and not self.mobBrakingProhibited:
        if (apply_brake < 0):
          apply_brake = 0
        if apply_brake > 0:
          if not mobEnabled:
            mobEnabled = True
            apply_brake = 0
          elif not mobPreEnable:
            mobPreEnable = True
            apply_brake = 0
          elif apply_brake >= mobBrakeMax:
            apply_brake = mobBrakeMax
            CS.brake_warning = True
          if CS.currentSpeed < .8: #3kph
            stopping_wish = True
        else:
          mobPreEnable = False
          mobEnabled = False
      else:
        apply_brake = 0
        mobPreEnable = False
        mobEnabled = False

      apply_brake = int(apply_brake)

      idx = (frame / P.MOB_STEP) % 16
      self.mobPreEnable = mobPreEnable
      self.mobEnabled = mobEnabled
      can_sends.append(self.create_braking_control(self.packer_pt, CANBUS.br, apply_brake, idx, mobEnabled, mobPreEnable, stopping_wish))
      #CdddL_hook = True
    else:
      apply_brake = float('nan')    # for CdddL

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare PQ_AWV for Front Assist LED and Front Assist Text               #
    #   Front Assist warning sign + sound + brake jolt                        #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.AWV_STEP == 0) and CS.CP.enableGasInterceptor:
      send_fcw = visual_alert == car.CarControl.HUDControl.VisualAlert.fcw
      green_led = 1 if enabled and (CS.ABSWorking == 0) else 0
      orange_led = 1 if self.mobPreEnable and self.mobEnabled else 0
      if enabled:
        braking_working = 0 if (CS.ABSWorking == 0) else 5
      else:
        braking_working = 0
      brakejolt_type = 0 # select depending on speed (which are to be determined) for the driver to feel the same jolt intensity
      brake_jolt = False # 2do: pass through from interface.py

      idx = (frame / P.MOB_STEP) % 16

      can_sends.append(
        self.create_awv_control(self.packer_pt, CANBUS.pt, idx, orange_led, green_led, braking_working, send_fcw, send_fcw, brakejolt_type, brake_jolt))


#      # --------------------------------------------------------------------------
#      #                                                                         #
#      # Prepare PQ_ACA for mACC_GRA_Anzeige in cluster, lead distance etc       #
#      #                                                                         #
#     # --------------------------------------------------------------------------
#      if (frame % P.ACA_STEP == 0) and CS.CP.enableGasInterceptor:
#
#        # what is 4 = ACC im Hintergrund ?
#        # what is 6 = ACC reversibel aus ?
#        # what is 7 = ACC irreversibel aus ?
#
#        acc_status = 0 # default
#        if CS.out.cruiseState.available:
#          if enabled:
#              acc_status = 3 # ACC active
#          else:
#              acc_status = 2 # ACC passiv
#        else:
#          acc_status = 0 # GRA Hauptschalter off
#
#        lead_distance = 7 if leadVisible else 0
#        lead_distance_raw = int(leadDistRel / 3) # max ca. 150m
#        print(lead_distance_raw)
#        if lead_distance_raw < 1:
#          lead_dist = 1
#        elif lead_distance_raw > 15:
#          lead_dist = 15
#        lead_distance = lead_distance_raw if leadVisible else 0
#
#        acc_setspeed = setSpeed #CS.out.cruiseState.speed * CV.MS_TO_KPH
#        braking_active = 1 if self.mobPreEnable and self.mobEnabled else 0
#        display_prio = 3 # no priority
#        beep = False
#        chime = False
#        distance_setpoint = 3 # observed at carlos_ddd SIMOS PCR2.1
#        alert_driver = False
#        acc_status_index = 0 # messages to driver ?
#
#        acc_setspeed = 88
#        acc_status = 2
#
#        idx = (frame / P.ACA_STEP) % 16
#
#        can_sends.append(
#          self.create_aca_control(self.packer_pt, CANBUS.pt, idx, acc_status, lead_distance, acc_setspeed, braking_active, display_prio, beep, chime, distance_setpoint, alert_driver, acc_status_index))

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare OPsta for sending towards STM32                                 #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------

    if (frame % P.OPSTA_STEP == 0):
        op_engaged = enabled
        #if CS.out.cruiseState.available:
        lead_distance = 0

        #AudibleAlert (defined in cereal/car.capnp)
        #none
        #engage
        #disengage
        #refuse
        #warningSoft
        #warningImmediate
        #prompt
        #promptRepeat
        #promptDistracted
    
        #VisualAlert (defined in cereal/car.capnp)
        #none
        #fcw
        #steerRequired
        #brakePressed
        #wrongGear
        #seatbeltUnbuckled
        #speedTooHigh
        #ldw
   
        ledbar_info = (audible_alert == car.CarControl.HUDControl.AudibleAlert.prompt) or (audible_alert == car.CarControl.HUDControl.AudibleAlert.promptRepeat) 
        ledbar_warn = (audible_alert == car.CarControl.HUDControl.AudibleAlert.warningSoft) or (audible_alert == car.CarControl.HUDControl.AudibleAlert.warningImmediate)
        ledbar_max = False
        
        ledbar_val = 0
        if ledbar_max:
            ledbar_val = 3
        elif ledbar_warn:
            ledbar_val = 2
        elif ledbar_info or (visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired):
            ledbar_val = 1

        sound_val = 0 # not yet implemented
        
        op_fcw = (visual_alert == car.CarControl.HUDControl.VisualAlert.fcw)
        
        request_turnsignal_val = 0 # not yet implemented

        op_setspeed = int(CS.out.cruiseState.speed * 3.6) #setSpeed #CS.out.cruiseState.speed * CV.MS_TO_KPH or look at the CC.update()-call in interface.py (apply() around line 343)
        
        accel_val_scaled = actuators.accel*100.0    # we want milli-m/s^2
        accel_val = abs(int(accel_val_scaled))
        accel_mode = 0      # 0=no request, 1=request gas, 2=request brake
        accel_inhibits = 0  # 0=no inhibitions, 1=no gas, 2=no brake
        accel_format = 0    # 0=undefined, 1=int milli-m/s^2, 2=raw(12bit gas, mMOB brake)
        accel_signbit = True if accel_val_scaled<0.0 else False

        idx = (frame / P.OPSTA_STEP) % 16 # counter

        can_sends.append(
          self.create_opsta_control(self.packer_pt, CANBUS.br, idx, op_engaged, lead_distance, ledbar_val, sound_val, op_fcw, request_turnsignal_val, op_setspeed, accel_val, accel_mode, accel_inhibits, accel_format, accel_signbit))

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare GAS_COMMAND for sending towards Pedal                           #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.GAS_STEP == 0) and CS.CP.enableGasInterceptor:
      apply_gas = 0     # volkswagencan.py::create_pq_pedal_control() will clip these values to (227<apply_gas<1240)

      self.coastingAccel = interp(CS.out.vEgo, P.COASTING_LOOKUP_BP, P.COASTING_LOOKUP_V)   # new Edgy

      if enabled and not ( CS.out.clutchPressed or CS.gas_inhibit ):

        #apply_gas = clip(actuators.gas, 0., 1.)     # old v0.8.2

        # BEGIN new Edgy
        if actuators.accel > self.coastingAccel:
          speed = CS.out.vEgo
          cd = 0.31
          frontalArea = 2.3
          drag = 0.5 * cd * frontalArea * (speed ** 2)

          mass = 1250
          g = 9.81
          rollingFrictionCoefficient = 0.02
          friction = mass * g * rollingFrictionCoefficient

          desiredAcceleration = actuators.accel
          acceleration = mass * desiredAcceleration

          driveTrainLosses = 0  # 600 for the engine, 200 for trans, low speed estimate
          powerNeeded = (drag + friction + acceleration) * speed + driveTrainLosses
          POWER_LOOKUP_BP = [0, 25000 * 1.6 / 2.6,
                               75000]  # 160NM@1500rpm=25kW but with boost, no boost means *1.6/2.6
          PEDAL_LOOKUP_BP = [227, 1250 * 0.4,
                               1250 * 100 / 140]  # Not max gas, max gas gives 140hp, we want at most 100 hp, also 40% throttle might prevent an upshift

          GAS_MULTIPLIER_BP = [0, 0.1, 0.2, 0.4, 8.3]
          GAS_MULTIPLIER_V = [1.15, 1.15, 1.2, 1.25, 1.]

          powerNeeded_mult = interp(CS.out.vEgo, [20 / 3.6, 40 / 3.6], [2, 1])
          powerNeeded = int(round(powerNeeded * powerNeeded_mult))
          apply_gas = int(round(interp(powerNeeded, POWER_LOOKUP_BP, PEDAL_LOOKUP_BP)))
          apply_gas = int(round(apply_gas * int(round(interp(speed, GAS_MULTIPLIER_BP, GAS_MULTIPLIER_V)))))
        else:
          apply_gas = 0
        # END new Edgy


        # acceleration test module output
        if cddda_active:  # overwrite
          apply_gas = cddda_apply_gas


        apply_gas = int(apply_gas)

      can_sends.append(self.create_gas_control(self.packer_pt, CANBUS.cam, apply_gas, frame // 2))
      #CdddL_hook = True
    else:
      apply_brake = float('nan')    # for CdddL

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare VIN_MESSAGE for sending towards Panda                           #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    # if using radar, we need to send the VIN
    if CS.useTeslaRadar and (frame % 100 == 0):
      can_sends.append(
        volkswagencan.create_radar_VIN_msg(self.radarVin_idx, CS.radarVIN, 2, 0x4A0, CS.useTeslaRadar,
                                            CS.radarPosition,
                                            CS.radarEpasType))
      self.radarVin_idx += 1
      self.radarVin_idx = self.radarVin_idx % 3

    #--------------------------------------------------------------------------
    #                                                                         #
    # Prepare LDW_02 HUD messages with lane borders, confidence levels, and   #
    # the LKAS status LED.                                                    #
    #                                                                         #
    #--------------------------------------------------------------------------

    # The factory camera emits this message at 10Hz. When OP is active, Panda
    # filters LDW_02 from the factory camera and OP emits LDW_02 at 10Hz.


    # **** HUD Controls ***************************************************** #
    if frame % P.LDW_STEP == 0:
    
    
      if visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = PQ_LDW_MESSAGES["laneAssistTakeOver"]
      else:
        hud_alert = PQ_LDW_MESSAGES["none"]

      can_sends.append(self.create_hud_control(self.packer_pt, CANBUS.pt, enabled,
                                                            CS.out.steeringPressed, hud_alert, left_lane_visible,
                                                            right_lane_visible, CS.ldw_stock_values,
                                                            left_lane_depart, right_lane_depart))


    #--------------------------------------------------------------------------
    #                                                                         #
    # Prepare GRA_ACC_01 ACC control messages with button press events.       #
    #                                                                         #
    #--------------------------------------------------------------------------

    # The car sends this message at 33hz. OP sends it on-demand only for
    # virtual button presses.
    #
    # First create any virtual button press event needed by openpilot, to sync
    # stock ACC with OP disengagement, or to auto-resume from stop.
    
#    if CS.CP.pcmCruise:
#      if frame > self.graMsgStartFramePrev + P.GRA_VBP_STEP:
#        if not enabled and CS.out.cruiseState.enabled:
#          # Cancel ACC if it's engaged with OP disengaged.
#          self.graButtonStatesToSend = BUTTON_STATES.copy()
#          self.graButtonStatesToSend["cancel"] = True
#        elif enabled and CS.esp_hold_confirmation:
#          # Blip the Resume button if we're engaged at standstill.
#          # FIXME: This is a naive implementation, improve with visiond or radar input.
#          self.graButtonStatesToSend = BUTTON_STATES.copy()
#          self.graButtonStatesToSend["resumeCruise"] = True

#
#    # OP/Panda can see this message but can't filter it when integrated at the
#    # R242 LKAS camera. It could do so if integrated at the J533 gateway, but
#    # we need a generalized solution that works for either. The message is
#    # counter-protected, so we need to time our transmissions very precisely
#    # to achieve fast and fault-free switching between message flows accepted
#    # at the J428 ACC radar.
#    #
#    # Example message flow on the bus, frequency of 33Hz (GRA_ACC_STEP):
#    #
#    # CAR: 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  0  1  2  3  4  5  6
#    # EON:        3  4  5  6  7  8  9  A  B  C  D  E  F  0  1  2  GG^
#    #
#    # If OP needs to send a button press, it waits to see a GRA_ACC_01 message
#    # counter change, and then immediately follows up with the next increment.
#    # The OP message will be sent within about 1ms of the car's message, which
#    # is about 2ms before the car's next message is expected. OP sends for an
#    # arbitrary duration of 16 messages / ~0.5 sec, in lockstep with each new
#    # message from the car.
#    #
#    # Because OP's counter is synced to the car, J428 immediately accepts the
#    # OP messages as valid. Further messages from the car get discarded as
#    # duplicates without a fault. When OP stops sending, the extra time gap
#    # (GG) to the next valid car message is less than 1 * GRA_ACC_STEP. J428
#    # tolerates the gap just fine and control returns to the car immediately.

#      if CS.graMsgBusCounter != self.graMsgBusCounterPrev:
#        self.graMsgBusCounterPrev = CS.graMsgBusCounter
#        if self.graButtonStatesToSend is not None:
#          if self.graMsgSentCount == 0:
#            self.graMsgStartFramePrev = frame
#          idx = (CS.graMsgBusCounter + 1) % 16
#          can_sends.append(self.create_acc_buttons_control(self.packer_pt, ext_bus, self.graButtonStatesToSend, CS, idx))
#          self.graMsgSentCount += 1
#          if self.graMsgSentCount >= P.GRA_VBP_COUNT:
#            self.graButtonStatesToSend = None
#            self.graMsgSentCount = 0

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX

    # if CdddL_hook:
    #     self.CdddL.update('vEgo', CS.out.vEgo)
    #     self.CdddL.update('aEgo', CS.out.aEgo)
    #     self.CdddL.update('final_accel', actuators.accel)
    #     self.CdddL.update('apply_brake', apply_brake)
    #     self.CdddL.update('apply_gas', apply_gas)
    #     self.CdddL.update('detected_gear', CS.detected_gear)
    #     self.CdddL.update('engineRPM', CS.out.engineRPM)
    #     self.CdddL.update('clutchPressed', CS.out.clutchPressed, convert=True)
    #     self.CdddL.update('ecuGas', CS.gas_ecu)
    #     self.CdddL.update('ecuLeergas', CS.leergas, convert=True)
    #     self.CdddL.update('brakePressed', CS.out.brakePressed, convert=True)
    #     self.CdddL.update('cddda_active', cddda_active, convert=True)
    #     self.CdddL.update('enabled', enabled, convert=True)
    #     self.CdddL.update('pedal1', CS.gasInterceptorSensor1) # CS.gas is simply (gasInterceptorSensor1+gasInterceptorSensor2)/2 -> calculated yourself in numpy!
    #     self.CdddL.update('pedal2', CS.gasInterceptorSensor2)
    #     self.CdddL.slice_done()

    self.CdddAL.update(enabled, CS.out.vEgo, CS.out.aEgo, CS.out.clutchPressed, CS.detected_gear, CS.out.engineRPM, apply_gas, apply_brake, CS.out.gas, CS.leergas, CS.out.brakePressed)

    return new_actuators, can_sends
