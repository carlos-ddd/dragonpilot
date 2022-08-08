//#define VW_USE_TESLA_RADAR

#ifdef VW_USE_TESLA_RADAR
// Include Tesla radar safety code
#include "safety_teslaradar.h"
#endif

// Safety-relevant steering constants for Volkswagen
const int VOLKSWAGEN_PQ_MAX_STEER = 300;                // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
const int VOLKSWAGEN_PQ_MAX_RT_DELTA = 188;              // 10 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 125 ; 125 * 1.5 for safety pad = 187.5
const uint32_t VOLKSWAGEN_PQ_RT_INTERVAL = 250000;      // 250ms between real time checks
const int VOLKSWAGEN_PQ_MAX_RATE_UP = 10;                // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
const int VOLKSWAGEN_PQ_MAX_RATE_DOWN = 10;             // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
const int VOLKSWAGEN_PQ_DRIVER_TORQUE_ALLOWANCE = 80;
const int VOLKSWAGEN_PQ_DRIVER_TORQUE_FACTOR = 3;

const int VOLKSWAGEN_GAS_INTERCEPTOR_THRSLD = 475;  // ratio between offset and gain from dbc file
#define VOLKSWAGEN_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2) // avg between 2 tracks


// Safety-relevant CAN messages for the Volkswagen PQ35/PQ46/NMS platforms
#define MSG_LENKHILFE_3 0x0D0   // RX from EPS, for steering angle and driver steering torque
#define MSG_HCA_1       0x0D2   // TX by OP, Heading Control Assist steering torque
#define MSG_MOB_1       0x284   // TX by OP, Braking Control
#define MSG_GAS_COMMAND 0x200   // TX by OP, Gas Control
#define MSG_GAS_SENSOR  0x201   // RX from Pedal
#define MSG_AWV_1       0x366   // TX by OP, ACC LED
#define MSG_MOTOR_2     0x288   // RX from ECU, for CC state and brake switch state
#define MSG_MOTOR_3     0x380   // RX from ECU, for driver throttle input
#define MSG_GRA_NEU     0x38A   // TX by OP, ACC control buttons for cancel/resume
#define MSG_BREMSE_1    0x1A0   // RX from ABS, for ego speed
#define MSG_LDW_1       0x5BE   // TX by OP, Lane line recognition and text alerts
#define MSG_ACA         0x56A   // TX by OP, mACC_GRA_Anzeige for displaying ACC things in cluster
#define MSG_OPSTA       0x410   // TX by OP to STM32


// CAN Messages for Tesla Radar
#define MSG_TESLA_VIN   0x560   // TX by OP, Tesla VIN Message
#define MSG_TESLA_x2B9  0x2B9
#define MSG_TESLA_x159  0x159
#define MSG_TESLA_x219  0x219
#define MSG_TESLA_x149  0x149
#define MSG_TESLA_x129  0x129
#define MSG_TESLA_x1A9  0x1A9
#define MSG_TESLA_x199  0x199
#define MSG_TESLA_x169  0x169
#define MSG_TESLA_x119  0x119
#define MSG_TESLA_x109  0x109

// Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
// carlos_ddd: bus number, dlc ???
const CanMsg VOLKSWAGEN_PQ_TX_MSGS[] = { {MSG_HCA_1, 0, 5}, {MSG_GRA_NEU, 0, 4}, {MSG_GRA_NEU, 1, 4}, {MSG_GRA_NEU, 2, 4}, {MSG_LDW_1, 0, 8}, {MSG_MOB_1, 1, 6}, {MSG_GAS_COMMAND, 2, 6}, {MSG_AWV_1, 0, 5}, {MSG_OPSTA, 1, 8} }; //{MSG_ACA, 0, 8}
#define VOLKSWAGEN_PQ_TX_MSGS_LEN (sizeof(VOLKSWAGEN_PQ_TX_MSGS) / sizeof(VOLKSWAGEN_PQ_TX_MSGS[0]))

AddrCheckStruct volkswagen_pq_addr_checks[] = {
  {.msg = {{MSG_LENKHILFE_3, 0, 6, .check_checksum = true,  .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_2, 0, 8, .check_checksum = false, .max_counter = 0U,  .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_3, 0, 8, .check_checksum = false, .max_counter = 0U,  .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{MSG_BREMSE_1, 0, 8, .check_checksum = false, .max_counter = 0U,  .expected_timestep = 10000U}, { 0 }, { 0 }}},
};
#define VOLKSWAGEN_PQ_ADDR_CHECKS_LEN (sizeof(volkswagen_pq_addr_checks) / sizeof(volkswagen_pq_addr_checks[0]))
addr_checks volkswagen_pq_rx_checks = {volkswagen_pq_addr_checks, VOLKSWAGEN_PQ_ADDR_CHECKS_LEN};


static uint8_t volkswagen_pq_get_checksum(CANPacket_t *to_push) {
  return (uint8_t)GET_BYTE(to_push, 0);
}

static uint8_t volkswagen_pq_get_counter(CANPacket_t *to_push) {
  // Few PQ messages have counters, and their offsets are inconsistent. This
  // function works only for Lenkhilfe_3 at this time.
  return (uint8_t)(GET_BYTE(to_push, 1) & 0xF0U) >> 4;
}

static uint8_t volkswagen_pq_compute_checksum(CANPacket_t *to_push) {
  int len = GET_LEN(to_push);
  uint8_t checksum = 0U;

  for (int i = 1; i < len; i++) {
    checksum ^= (uint8_t)GET_BYTE(to_push, i);
  }

  return checksum;
}

static const addr_checks* volkswagen_pq_init(int16_t param) {
  UNUSED(param);

  controls_allowed = false;
  relay_malfunction_reset();
  return &volkswagen_pq_rx_checks;
}

static int volkswagen_pq_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &volkswagen_pq_rx_checks,
                                volkswagen_pq_get_checksum, volkswagen_pq_compute_checksum, volkswagen_pq_get_counter);

#ifdef VW_USE_TESLA_RADAR
  teslaradar_rx_hook(to_push);
#endif

  if (valid) {
    int addr = GET_ADDR(to_push);

    // Update in-motion state from speed value.
    // Signal: Bremse_1.Geschwindigkeit_neu__Bremse_1_
    if ((addr == MSG_BREMSE_1) && (GET_BUS(to_push) == 0)) {
      int speed = ((GET_BYTE(to_push, 2) & 0xFEU) >> 1) | (GET_BYTE(to_push, 3) << 7);
      // DBC speed scale 0.01: 0.3m/s = 108.
      vehicle_moving = speed > 108;
    }

    // Update driver input torque samples
    // Signal: Lenkhilfe_3.LH3_LM (absolute torque)
    // Signal: Lenkhilfe_3.LH3_LMSign (direction)
    if ((addr == MSG_LENKHILFE_3) && (GET_BUS(to_push) == 0)) {
      int torque_driver_new = GET_BYTE(to_push, 2) | ((GET_BYTE(to_push, 3) & 0x3U) << 8);
      int sign = (GET_BYTE(to_push, 3) & 0x4U) >> 2;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    // Gasinterceptor detection and control allowance 
    // Generally allow controls if gas interceptor is detected unless gas pressed
    // Exit controls on rising edge of interceptor gas press
    if ( (addr == MSG_GAS_SENSOR) && (GET_BUS(to_push) == 2) ) {
      gas_interceptor_detected = 1;     // see safety_declarations.h
      controls_allowed = 1;
      int gas_interceptor = VOLKSWAGEN_GET_INTERCEPTOR(to_push);
      if ( (gas_interceptor > VOLKSWAGEN_GAS_INTERCEPTOR_THRSLD) &&
           (gas_interceptor_prev <= VOLKSWAGEN_GAS_INTERCEPTOR_THRSLD) ) {
        controls_allowed = 0;
      }
      gas_interceptor_prev = gas_interceptor;
      // was ist mit gas_pressed?
    }

    // Update ACC status from ECU for controls-allowed state
    // If gas interceptor not detected, controls only allowed when GRA/ACC active
    // Signal: Motor_2.GRA_Status
    if ( (addr == MSG_MOTOR_2) && ! (gas_interceptor_detected) && (GET_BUS(to_push) == 0) ) {
      int acc_status = (GET_BYTE(to_push, 2) & 0xC0U) >> 6;
      int cruise_engaged = ((acc_status == 1) || (acc_status == 2)) ? 1 : 0;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // Signal: Motor_3.Fahrpedal_Rohsignal
    if ((addr == MSG_MOTOR_3) && !gas_interceptor_detected && (GET_BUS(to_push) == 0)) {
      gas_pressed = (GET_BYTE(to_push, 2));
    }

    // Signal: Motor_2.Bremslichtschalter
    if ((addr == MSG_MOTOR_2) && (GET_BUS(to_push) == 0)) {
      brake_pressed = (GET_BYTE(to_push, 2) & 0x1U);
    }

    generic_rx_checks((addr == MSG_HCA_1) && (GET_BUS(to_push) == 0));
  }
  return valid;
}

static bool volkswagen_steering_check(int desired_torque) {
  bool violation = false;
  uint32_t ts = microsecond_timer_get();

  if (controls_allowed) {
    // *** global torque limit check ***
    violation |= max_limit_check(desired_torque, VOLKSWAGEN_PQ_MAX_STEER, -VOLKSWAGEN_PQ_MAX_STEER);

    // *** torque rate limit check ***
    violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
      VOLKSWAGEN_PQ_MAX_STEER, VOLKSWAGEN_PQ_MAX_RATE_UP, VOLKSWAGEN_PQ_MAX_RATE_DOWN,
      VOLKSWAGEN_PQ_DRIVER_TORQUE_ALLOWANCE, VOLKSWAGEN_PQ_DRIVER_TORQUE_FACTOR);
    desired_torque_last = desired_torque;

    // *** torque real time rate limit check ***
    violation |= rt_rate_limit_check(desired_torque, rt_torque_last, VOLKSWAGEN_PQ_MAX_RT_DELTA);

    // every RT_INTERVAL set the new limits
    uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
    if (ts_elapsed > VOLKSWAGEN_PQ_RT_INTERVAL) {
      rt_torque_last = desired_torque;
      ts_last = ts;
    }
  }

  // no torque if controls is not allowed
  if (!controls_allowed && (desired_torque != 0)) {
    violation = true;
  }

  // reset to 0 if either controls is not allowed or there's a violation
  if (violation || !controls_allowed) {
    desired_torque_last = 0;
    rt_torque_last = 0;
    ts_last = ts;
  }

  return violation;
}

static int volkswagen_pq_tx_hook(CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  int tx = 1;

  if (!msg_allowed(to_send, VOLKSWAGEN_PQ_TX_MSGS, VOLKSWAGEN_PQ_TX_MSGS_LEN)) {
    tx = 0;
  }

  // Safety check for HCA_1 Heading Control Assist torque
  // Signal: HCA_1.LM_Offset (absolute torque)
  // Signal: HCA_1.LM_Offsign (direction)
  if (addr == MSG_HCA_1) {
    int desired_torque = GET_BYTE(to_send, 2) | ((GET_BYTE(to_send, 3) & 0x7FU) << 8);
    desired_torque = desired_torque / 32;  // DBC scale from PQ network to centi-Nm
    int sign = (GET_BYTE(to_send, 3) & 0x80U) >> 7;
    if (sign == 1) {
      desired_torque *= -1;
    }

    if (volkswagen_steering_check(desired_torque)) {
      tx = 0;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == MSG_GRA_NEU) && !controls_allowed) {
    // disallow resume and set: bits 16 and 17
    if ((GET_BYTE(to_send, 2) & 0x3U) != 0U) {
      tx = 0;
    }
  }

 // Tesla Radar TX hook
 // check if this is a teslaradar vin message
 // capture message for radarVIN and settings
 // Tesla Radar advertising its VIN, position and EPAS-type
 //  those need to be sent to it later on

// cave: debug only
int tesla_radar_should_send = 0; //set to 1 from EON via fake message when we want to use it
int radarPosition = 0;
int radarEpasType = 0;
uint32_t tesla_radar_trigger_message_id = 0x4A0; //id of the message
uint8_t tesla_radar_can = 2; // 0, 1 or 2 set from EON via fake message
char radar_VIN[] = "                 "; //leave empty if your radar VIN matches the car VIN
int tesla_radar_vin_complete = 0; //set to 7 when complete vin is received

 if (addr == MSG_TESLA_VIN) {

   UNUSED(radar_VIN);
   UNUSED(tesla_radar_can);
   UNUSED(tesla_radar_trigger_message_id);
   UNUSED(radarEpasType);
   UNUSED(radarPosition);
   UNUSED(tesla_radar_should_send);
   UNUSED(tesla_radar_vin_complete);

   int id = ( GET_BYTE(to_send, 0) );
   int radarVin_b1 = ( GET_BYTE(to_send, 1) );
   int radarVin_b2 = ( GET_BYTE(to_send, 2) );
   int radarVin_b3 = ( GET_BYTE(to_send, 3) );
   int radarVin_b4 = ( GET_BYTE(to_send, 4) );
   int radarVin_b5 = ( GET_BYTE(to_send, 5) );
   int radarVin_b6 = ( GET_BYTE(to_send, 6) );
   int radarVin_b7 = ( GET_BYTE(to_send, 7) );
   if (id == 0) {
     tesla_radar_should_send = (radarVin_b2 & 0x01);
     radarPosition =  ((radarVin_b2 >> 1) & 0x03);
     radarEpasType = ((radarVin_b2 >> 3) & 0x07);
     tesla_radar_trigger_message_id = (radarVin_b3 << 8) + radarVin_b4;
     tesla_radar_can = radarVin_b1;
     radar_VIN[0] = radarVin_b5;
     radar_VIN[1] = radarVin_b6;
     radar_VIN[2] = radarVin_b7;
     tesla_radar_vin_complete = tesla_radar_vin_complete | 1;
   }
   if (id == 1) {
     radar_VIN[3] = radarVin_b1;
     radar_VIN[4] = radarVin_b2;
     radar_VIN[5] = radarVin_b3;
     radar_VIN[6] = radarVin_b4;
     radar_VIN[7] = radarVin_b5;
     radar_VIN[8] = radarVin_b6;
     radar_VIN[9] = radarVin_b7;
     tesla_radar_vin_complete = tesla_radar_vin_complete | 2;
   }
   if (id == 2) {
     radar_VIN[10] = radarVin_b1;
     radar_VIN[11] = radarVin_b2;
     radar_VIN[12] = radarVin_b3;
     radar_VIN[13] = radarVin_b4;
     radar_VIN[14] = radarVin_b5;
     radar_VIN[15] = radarVin_b6;
     radar_VIN[16] = radarVin_b7;
     tesla_radar_vin_complete = tesla_radar_vin_complete | 4;
    }
    else {
      return 0;
    }
  }

  // GAS PEDAL: safety check
  if (addr == MSG_GAS_COMMAND) {
    if (!controls_allowed) {
      if (GET_BYTE(to_send, 0) || GET_BYTE(to_send, 1)) {
        tx = 0;
      }
    }
  }

  // 1 allows the message through
  return tx;
}

static int volkswagen_pq_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int addr = GET_ADDR(to_fwd);
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      // Forward all traffic from the Extended CAN onward
      bus_fwd = 2;
      break;
    case 2:
      if ((addr == MSG_HCA_1) || (addr == MSG_LDW_1)) {
        // OP takes control of the Heading Control Assist and Lane Departure Warning messages from the camera
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

// Volkswagen PQ35/PQ46/NMS platforms
const safety_hooks volkswagen_pq_hooks = {
  .init = volkswagen_pq_init,
  .rx = volkswagen_pq_rx_hook,
  .tx = volkswagen_pq_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = volkswagen_pq_fwd_hook,
};
