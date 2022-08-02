from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import PQ_CARS, DBC_FILES, CANBUS, MQB_LDW_MESSAGES, BUTTON_STATES, PQ_LDW_MESSAGES, CarControllerParams as P
from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.

    self.apply_steer_last = 0
    self.mobPreEnable = False
    self.mobEnabled = False
    self.radarVin_idx = 0

    self.packer_pt = CANPacker(DBC_FILES.mqb)

    if CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.volkswagen:
      self.packer_pt = CANPacker(DBC_FILES.mqb)
      self.create_steering_control = volkswagencan.create_mqb_steering_control
      self.create_acc_buttons_control = volkswagencan.create_mqb_acc_buttons_control
      self.create_hud_control = volkswagencan.create_mqb_hud_control
    elif CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.volkswagenPq:
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


  def update(self, c, enabled, CS, frame, ext_bus, actuators, visual_alert, left_lane_visible, right_lane_visible, left_lane_depart, right_lane_depart, dragonconf):
    """ Controls thread """

    # Send CAN commands.
    can_sends = []

    print("\n", "carcontroller.py::update()", str(dir(P)),"\n")

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

      # dp
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
                                                                 idx, hcaEnabled))


    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare PQ_MOB for sending the braking message                          #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.MOB_STEP == 0) and CS.CP.enableGasInterceptor:
      mobEnabled = self.mobEnabled
      mobPreEnable = self.mobPreEnable
      mobBrakeScaling = 650
      mobBrakeMax = min(8190, 8000)
      # TODO make sure we use the full 8190 when calculating braking.
      apply_brake = actuators.brake * mobBrakeScaling
      stopping_wish = False

      if enabled:
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

        # none
        # chimeEngage
        # chimeDisengage
        # chimeError
        # chimePrompt 
        # chimeWarning1
        # chimeWarning2
        # chimeWarning2Repeat
        # chimeWarningRepeat
        ledbar_info = (audible_alert == car.CarControl.HUDControl.AudibleAlert.chimePrompt) or (audible_alert == car.CarControl.HUDControl.AudibleAlert.chimeWarning2Repeat) 
        ledbar_warn = (audible_alert == car.CarControl.HUDControl.AudibleAlert.chimeWarningRepeat) or (audible_alert == car.CarControl.HUDControl.AudibleAlert.chimeWarning1)
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

        op_setspeed = 0 #setSpeed #CS.out.cruiseState.speed * CV.MS_TO_KPH

        idx = (frame / P.OPSTA_STEP) % 16 # counter

        can_sends.append(
          self.create_opsta_control(self.packer_pt, CANBUS.br, idx, op_engaged, lead_distance, ledbar_val, sound_val, op_fcw, request_turnsignal_val, op_setspeed))

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare GAS_COMMAND for sending towards Pedal                           #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.GAS_STEP == 0) and CS.CP.enableGasInterceptor:
      apply_gas = 0
      if enabled and not CS.out.clutchPressed:
        apply_gas = clip(actuators.gas, 0., 1.)

      can_sends.append(self.create_gas_control(self.packer_pt, CANBUS.cam, apply_gas, frame // 2))

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

    return new_actuators, can_sends
