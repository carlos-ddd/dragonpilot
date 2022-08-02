# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

from selfdrive.car import crc8_pedal

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "SET_ME_0X3": 0x3,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": not lkas_enabled,
    "HCA_Active": lkas_enabled,
    "SET_ME_0XFE": 0xFE,
    "SET_ME_0X07": 0x07,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_mqb_hud_control(packer, bus, enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                           ldw_stock_values, left_lane_depart, right_lane_depart):
  # Lane color reference:
  # 0 (LKAS disabled) - off
  # 1 (LKAS enabled, no lane detected) - dark gray
  # 2 (LKAS enabled, lane detected) - light gray on VW, green or white on Audi depending on year or virtual cockpit.  On a color MFD on a 2015 A3 TDI it is white, virtual cockpit on a 2018 A3 e-Tron its green.
  # 3 (LKAS enabled, lane departure detected) - white on VW, red on Audi
  values = ldw_stock_values.copy()
  values.update({
    "LDW_Status_LED_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if left_lane_depart else 1 + left_lane_visible,
    "LDW_Lernmodus_rechts": 3 if right_lane_depart else 1 + right_lane_visible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)

def create_mqb_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }
  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)

# ----------------------------------------------------------------------- #
#                                                                         #
# CAN message packing for PQ35/PQ46/NMS vehicles                          #
#                                                                         #
# ----------------------------------------------------------------------- #

def create_pq_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_Zaehler": idx,
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": 7 if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  dat = packer.make_can_msg("HCA_1", bus, values)[2]
  values["HCA_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("HCA_1", bus, values)
  
  
def create_pq_braking_control(packer, bus, apply_brake, idx, brake_enabled, brake_pre_enable, stopping_wish):
  values = {
    "PQ_MOB_COUNTER": idx,
    "MOB_Bremsmom": abs(apply_brake),
#    "MOB_Bremsstgr": abs(apply_brake),
    "MOB_Standby": 1 if (brake_enabled) else 0,
    "MOB_Freigabe": 1 if (brake_enabled and brake_pre_enable) else 0,
    "MOB_Anhaltewunsch": 1 if stopping_wish else 0,
  }

  dat = packer.make_can_msg("MOB_1", bus, values)[2]
  values["PQ_MOB_CHECKSUM"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5]
  return packer.make_can_msg("MOB_1", bus, values)
  
  
def create_pq_awv_control(packer, bus, idx, led_orange, led_green, abs_working, brake_symbol, warn_sound, brakejolt_type, brake_jolt):
  values = {
    "AWV_2_Fehler" : 1 if led_orange else 0,
    "AWV_2_Status" : 1 if led_green else 0,
    "AWV_Zaehler": idx,
    "AWV_Text": abs_working,
    "AWV_Infoton": 1 if (abs_working == 5) else 0,
    "AWV_2_Warnsymbol": 1 if brake_symbol else 0,
    "AWV_2_Warnton": 1 if warn_sound else 0,
    "AWV_2_Ruckprofil": brakejolt_type if 0<=brakejolt_type<=7 else 0, # 2^3bit=8 but VW says only 6 different profiles available of which only 3 are used
    "AWV_2_Freigabe": 1 if brake_jolt else 0,
  }

  dat = packer.make_can_msg("mAWV", bus, values)[2]
  values["AWV_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("mAWV", bus, values)

#def create_pq_aca_control(packer, bus, idx, acc_status, lead_distance, acc_setspeed, braking_active, display_prio, beep, chime, distance_setpoint, alert_driver, acc_status_index):
#  values = {
#    "ACA_StaACC" : acc_status if (0<=acc_status<=7) else 7, #0=GRA_Hauptschalter_off, 1=reserved, 2=acc_passive, 3=acc_active, 4=ACC_in_background, 5=frei, 6=ACC_reversible_off, 7=ACC_irreversible_off
#    "ACA_ID_StaACC": acc_status_index if (0<=acc_status_index<=31) else 0, # 0=keine Anzeige
#    "ACA_Fahrerhinw" : 1 if alert_driver else 0,
#    "ACA_AnzDisplay" : 0, # Anzeige erweitern ???
#    "ACA_Zeitluecke" : distance_setpoint if (0<=distance_setpoint<=15) else 0, #0..15, 0=not defined
#    "ACA_V_Wunsch" : int(acc_setspeed) if (0<=int(acc_setspeed)<=15) else 255, # 255 = no setspeed
#    "ACA_kmh_mph" : 0, #kmh
#    "ACA_Akustik1": 1 if chime else 0, # display_prop->2
#    "ACA_Akustik2" : 1 if beep else 0, # display_prio->1
#    "ACA_PrioDisp" : display_prio if (0<=display_prio<=3) else 3, # priority: 0=highest, 1=mid, 2=low, 3=no request for display
#    "ACA_gemZeitl" : lead_distance if (0<=lead_distance<=15) else 0, # 0..15 0=no target, 1=very near, 15=very far away
#    "ACA_ACC_Verz" : 1 if braking_active else 0,
#    "ACA_StaGRA" : 2, # not using GRA, seen at carlos_ddd SIMOS PCR2.1
#    "ACA_ID_StaGRA": 0, # not using GRA
#    "ACA_Codierung": 0, # 0=ACC, 1=GRA
#    "ACA_Tachokranz": 0, # always observed with 0
#    "ACA_Aend_Zeitluecke": 0,# maybe only changes when radar commands distance set change ?
#    "ACA_Zaehler": idx,
#  }
#
#  dat = packer.make_can_msg("ACC_GRA_Anziege", bus, values)[2]
#  values["ACA_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
#  return packer.make_can_msg("ACC_GRA_Anziege", bus, values)

def create_pq_opsta_control(packer, bus, idx, op_engaged, lead_distance, ledbar_val, sound_val, op_fcw, request_turnsignal_val, op_setspeed):
  values = {
    "OPS1_set_speed": op_setspeed,
    "OPS1_turnsignal_request": request_turnsignal_val,
    "OPS1_FCW": 1 if op_fcw else 0,
    "OPS1_acoustic": sound_val,
    "OPS1_led_bar": ledbar_val,
    "OPS1_lead_distance": lead_distance,
    "OPS1_engaged": 1 if op_engaged else 0,
    "OPS1_counter": idx,
  }

  dat = packer.make_can_msg("mOP_Status1", bus, values)[2]
  values["OPS1_checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("mOP_Status1", bus, values)


def create_pq_pedal_control(packer, bus, apply_gas, idx):
  # Common gas pedal msg generator
  enable = apply_gas > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    apply_gas = apply_gas * 1125.
    if (apply_gas < 227):
      apply_gas = 227
    values["GAS_COMMAND"] = apply_gas
    values["GAS_COMMAND2"] = apply_gas

  dat = packer.make_can_msg("GAS_COMMAND", bus, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", bus, values)


def create_pq_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                          ldw_stock_values, left_lane_depart, right_lane_depart):
  if hca_enabled:
    left_lane_hud = 3 if left_lane_visible else 1
    right_lane_hud = 3 if right_lane_visible else 1
  else:
    left_lane_hud = 2 if left_lane_visible else 1
    right_lane_hud = 2 if right_lane_visible else 1

  values = {
    "Right_Lane_Status": right_lane_hud,
    "Left_Lane_Status": left_lane_hud,
    "SET_ME_X1": 1,
    "Kombi_Lamp_Orange": 1 if hca_enabled and steering_pressed else 0,
    "Kombi_Lamp_Green": 1 if hca_enabled and not steering_pressed else 0,
    "LDW_Textbits": hud_alert,
  }
  return packer.make_can_msg("LDW_1", bus, values)


def create_pq_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Neu_Zaehler": idx,
    "GRA_Sender": CS.graSenderCoding,
    "GRA_Abbrechen": 1 if (buttonStatesToSend["cancel"] or CS.buttonStates["cancel"]) else 0,
    "GRA_Hauptschalt": CS.graHauptschalter,
  }

  dat = packer.make_can_msg("GRA_Neu", bus, values)[2]
  values["GRA_Checksum"] = dat[1] ^ dat[2] ^ dat[3]
  return packer.make_can_msg("GRA_Neu", bus, values)


def create_radar_VIN_msg(id,radarVIN,radarCAN,radarTriggerMessage,useRadar,radarPosition,radarEpasType):
  msg_id = 0x560
  msg_len = 8
  msg = create_string_buffer(msg_len)
  if id == 0:
    struct.pack_into('BBBBBBBB', msg, 0, id,radarCAN,useRadar + (radarPosition << 1) + (radarEpasType << 3), ((radarTriggerMessage >> 8) & 0xFF),(radarTriggerMessage & 0xFF),ord(radarVIN[0]),ord(radarVIN[1]),ord(radarVIN[2]))
  if id == 1:
    struct.pack_into('BBBBBBBB', msg, 0, id,ord(radarVIN[3]),ord(radarVIN[4]),ord(radarVIN[5]),ord(radarVIN[6]),ord(radarVIN[7]),ord(radarVIN[8]),ord(radarVIN[9]))
  if id == 2:
    struct.pack_into('BBBBBBBB', msg, 0, id,ord(radarVIN[10]),ord(radarVIN[11]),ord(radarVIN[12]),ord(radarVIN[13]),ord(radarVIN[14]),ord(radarVIN[15]),ord(radarVIN[16]))
  return [msg_id, 2, msg.raw, 0]
