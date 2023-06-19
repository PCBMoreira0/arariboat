#pragma once
// MESSAGE CONTROL_SYSTEM PACKING

#define MAVLINK_MSG_ID_CONTROL_SYSTEM 170


typedef struct __mavlink_control_system_t {
 float dac_output; /*< [mV] Digital-Analog converter output to control the motor.*/
 float potentiometer_signal; /*< [mV] Potentiometer output to control the motor.*/
 uint8_t pump_mask; /*<  Bitfield that encodes whether the pumps are active or not.*/
} mavlink_control_system_t;

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN 9
#define MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN 9
#define MAVLINK_MSG_ID_170_LEN 9
#define MAVLINK_MSG_ID_170_MIN_LEN 9

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC 202
#define MAVLINK_MSG_ID_170_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONTROL_SYSTEM { \
    170, \
    "CONTROL_SYSTEM", \
    3, \
    {  { "dac_output", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_system_t, dac_output) }, \
         { "potentiometer_signal", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_system_t, potentiometer_signal) }, \
         { "pump_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_control_system_t, pump_mask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONTROL_SYSTEM { \
    "CONTROL_SYSTEM", \
    3, \
    {  { "dac_output", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_system_t, dac_output) }, \
         { "potentiometer_signal", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_system_t, potentiometer_signal) }, \
         { "pump_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_control_system_t, pump_mask) }, \
         } \
}
#endif

/**
 * @brief Pack a control_system message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dac_output [mV] Digital-Analog converter output to control the motor.
 * @param potentiometer_signal [mV] Potentiometer output to control the motor.
 * @param pump_mask  Bitfield that encodes whether the pumps are active or not.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_system_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float dac_output, float potentiometer_signal, uint8_t pump_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN];
    _mav_put_float(buf, 0, dac_output);
    _mav_put_float(buf, 4, potentiometer_signal);
    _mav_put_uint8_t(buf, 8, pump_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN);
#else
    mavlink_control_system_t packet;
    packet.dac_output = dac_output;
    packet.potentiometer_signal = potentiometer_signal;
    packet.pump_mask = pump_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_SYSTEM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
}

/**
 * @brief Pack a control_system message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dac_output [mV] Digital-Analog converter output to control the motor.
 * @param potentiometer_signal [mV] Potentiometer output to control the motor.
 * @param pump_mask  Bitfield that encodes whether the pumps are active or not.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_system_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float dac_output,float potentiometer_signal,uint8_t pump_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN];
    _mav_put_float(buf, 0, dac_output);
    _mav_put_float(buf, 4, potentiometer_signal);
    _mav_put_uint8_t(buf, 8, pump_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN);
#else
    mavlink_control_system_t packet;
    packet.dac_output = dac_output;
    packet.potentiometer_signal = potentiometer_signal;
    packet.pump_mask = pump_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_SYSTEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
}

/**
 * @brief Encode a control_system struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_system C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_system_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_system_t* control_system)
{
    return mavlink_msg_control_system_pack(system_id, component_id, msg, control_system->dac_output, control_system->potentiometer_signal, control_system->pump_mask);
}

/**
 * @brief Encode a control_system struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_system C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_system_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_control_system_t* control_system)
{
    return mavlink_msg_control_system_pack_chan(system_id, component_id, chan, msg, control_system->dac_output, control_system->potentiometer_signal, control_system->pump_mask);
}

/**
 * @brief Send a control_system message
 * @param chan MAVLink channel to send the message
 *
 * @param dac_output [mV] Digital-Analog converter output to control the motor.
 * @param potentiometer_signal [mV] Potentiometer output to control the motor.
 * @param pump_mask  Bitfield that encodes whether the pumps are active or not.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_system_send(mavlink_channel_t chan, float dac_output, float potentiometer_signal, uint8_t pump_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN];
    _mav_put_float(buf, 0, dac_output);
    _mav_put_float(buf, 4, potentiometer_signal);
    _mav_put_uint8_t(buf, 8, pump_mask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SYSTEM, buf, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
#else
    mavlink_control_system_t packet;
    packet.dac_output = dac_output;
    packet.potentiometer_signal = potentiometer_signal;
    packet.pump_mask = pump_mask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SYSTEM, (const char *)&packet, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
#endif
}

/**
 * @brief Send a control_system message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_control_system_send_struct(mavlink_channel_t chan, const mavlink_control_system_t* control_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_control_system_send(chan, control_system->dac_output, control_system->potentiometer_signal, control_system->pump_mask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SYSTEM, (const char *)control_system, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_control_system_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float dac_output, float potentiometer_signal, uint8_t pump_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, dac_output);
    _mav_put_float(buf, 4, potentiometer_signal);
    _mav_put_uint8_t(buf, 8, pump_mask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SYSTEM, buf, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
#else
    mavlink_control_system_t *packet = (mavlink_control_system_t *)msgbuf;
    packet->dac_output = dac_output;
    packet->potentiometer_signal = potentiometer_signal;
    packet->pump_mask = pump_mask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SYSTEM, (const char *)packet, MAVLINK_MSG_ID_CONTROL_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN, MAVLINK_MSG_ID_CONTROL_SYSTEM_CRC);
#endif
}
#endif

#endif

// MESSAGE CONTROL_SYSTEM UNPACKING


/**
 * @brief Get field dac_output from control_system message
 *
 * @return [mV] Digital-Analog converter output to control the motor.
 */
static inline float mavlink_msg_control_system_get_dac_output(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field potentiometer_signal from control_system message
 *
 * @return [mV] Potentiometer output to control the motor.
 */
static inline float mavlink_msg_control_system_get_potentiometer_signal(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pump_mask from control_system message
 *
 * @return  Bitfield that encodes whether the pumps are active or not.
 */
static inline uint8_t mavlink_msg_control_system_get_pump_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a control_system message into a struct
 *
 * @param msg The message to decode
 * @param control_system C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_system_decode(const mavlink_message_t* msg, mavlink_control_system_t* control_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    control_system->dac_output = mavlink_msg_control_system_get_dac_output(msg);
    control_system->potentiometer_signal = mavlink_msg_control_system_get_potentiometer_signal(msg);
    control_system->pump_mask = mavlink_msg_control_system_get_pump_mask(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN? msg->len : MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN;
        memset(control_system, 0, MAVLINK_MSG_ID_CONTROL_SYSTEM_LEN);
    memcpy(control_system, _MAV_PAYLOAD(msg), len);
#endif
}
