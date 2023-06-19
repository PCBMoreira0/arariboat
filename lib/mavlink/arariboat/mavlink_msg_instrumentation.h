#pragma once
// MESSAGE INSTRUMENTATION PACKING

#define MAVLINK_MSG_ID_INSTRUMENTATION 171


typedef struct __mavlink_instrumentation_t {
 float current_zero; /*< [mA] Current Sensor 0*/
 float current_one; /*< [mA] Current sensor 1.*/
 float current_two; /*< [mA] Current sensor 2.*/
 float voltage_battery; /*< [mA] Voltage sensor.*/
} mavlink_instrumentation_t;

#define MAVLINK_MSG_ID_INSTRUMENTATION_LEN 16
#define MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN 16
#define MAVLINK_MSG_ID_171_LEN 16
#define MAVLINK_MSG_ID_171_MIN_LEN 16

#define MAVLINK_MSG_ID_INSTRUMENTATION_CRC 143
#define MAVLINK_MSG_ID_171_CRC 143



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INSTRUMENTATION { \
    171, \
    "INSTRUMENTATION", \
    4, \
    {  { "current_zero", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_instrumentation_t, current_zero) }, \
         { "current_one", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_instrumentation_t, current_one) }, \
         { "current_two", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_instrumentation_t, current_two) }, \
         { "voltage_battery", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_instrumentation_t, voltage_battery) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INSTRUMENTATION { \
    "INSTRUMENTATION", \
    4, \
    {  { "current_zero", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_instrumentation_t, current_zero) }, \
         { "current_one", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_instrumentation_t, current_one) }, \
         { "current_two", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_instrumentation_t, current_two) }, \
         { "voltage_battery", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_instrumentation_t, voltage_battery) }, \
         } \
}
#endif

/**
 * @brief Pack a instrumentation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param current_zero [mA] Current Sensor 0
 * @param current_one [mA] Current sensor 1.
 * @param current_two [mA] Current sensor 2.
 * @param voltage_battery [mA] Voltage sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_instrumentation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float current_zero, float current_one, float current_two, float voltage_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INSTRUMENTATION_LEN];
    _mav_put_float(buf, 0, current_zero);
    _mav_put_float(buf, 4, current_one);
    _mav_put_float(buf, 8, current_two);
    _mav_put_float(buf, 12, voltage_battery);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INSTRUMENTATION_LEN);
#else
    mavlink_instrumentation_t packet;
    packet.current_zero = current_zero;
    packet.current_one = current_one;
    packet.current_two = current_two;
    packet.voltage_battery = voltage_battery;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INSTRUMENTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INSTRUMENTATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
}

/**
 * @brief Pack a instrumentation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_zero [mA] Current Sensor 0
 * @param current_one [mA] Current sensor 1.
 * @param current_two [mA] Current sensor 2.
 * @param voltage_battery [mA] Voltage sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_instrumentation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float current_zero,float current_one,float current_two,float voltage_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INSTRUMENTATION_LEN];
    _mav_put_float(buf, 0, current_zero);
    _mav_put_float(buf, 4, current_one);
    _mav_put_float(buf, 8, current_two);
    _mav_put_float(buf, 12, voltage_battery);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INSTRUMENTATION_LEN);
#else
    mavlink_instrumentation_t packet;
    packet.current_zero = current_zero;
    packet.current_one = current_one;
    packet.current_two = current_two;
    packet.voltage_battery = voltage_battery;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INSTRUMENTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INSTRUMENTATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
}

/**
 * @brief Encode a instrumentation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param instrumentation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_instrumentation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_instrumentation_t* instrumentation)
{
    return mavlink_msg_instrumentation_pack(system_id, component_id, msg, instrumentation->current_zero, instrumentation->current_one, instrumentation->current_two, instrumentation->voltage_battery);
}

/**
 * @brief Encode a instrumentation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param instrumentation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_instrumentation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_instrumentation_t* instrumentation)
{
    return mavlink_msg_instrumentation_pack_chan(system_id, component_id, chan, msg, instrumentation->current_zero, instrumentation->current_one, instrumentation->current_two, instrumentation->voltage_battery);
}

/**
 * @brief Send a instrumentation message
 * @param chan MAVLink channel to send the message
 *
 * @param current_zero [mA] Current Sensor 0
 * @param current_one [mA] Current sensor 1.
 * @param current_two [mA] Current sensor 2.
 * @param voltage_battery [mA] Voltage sensor.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_instrumentation_send(mavlink_channel_t chan, float current_zero, float current_one, float current_two, float voltage_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INSTRUMENTATION_LEN];
    _mav_put_float(buf, 0, current_zero);
    _mav_put_float(buf, 4, current_one);
    _mav_put_float(buf, 8, current_two);
    _mav_put_float(buf, 12, voltage_battery);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INSTRUMENTATION, buf, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
#else
    mavlink_instrumentation_t packet;
    packet.current_zero = current_zero;
    packet.current_one = current_one;
    packet.current_two = current_two;
    packet.voltage_battery = voltage_battery;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INSTRUMENTATION, (const char *)&packet, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
#endif
}

/**
 * @brief Send a instrumentation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_instrumentation_send_struct(mavlink_channel_t chan, const mavlink_instrumentation_t* instrumentation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_instrumentation_send(chan, instrumentation->current_zero, instrumentation->current_one, instrumentation->current_two, instrumentation->voltage_battery);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INSTRUMENTATION, (const char *)instrumentation, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_INSTRUMENTATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_instrumentation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float current_zero, float current_one, float current_two, float voltage_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, current_zero);
    _mav_put_float(buf, 4, current_one);
    _mav_put_float(buf, 8, current_two);
    _mav_put_float(buf, 12, voltage_battery);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INSTRUMENTATION, buf, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
#else
    mavlink_instrumentation_t *packet = (mavlink_instrumentation_t *)msgbuf;
    packet->current_zero = current_zero;
    packet->current_one = current_one;
    packet->current_two = current_two;
    packet->voltage_battery = voltage_battery;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INSTRUMENTATION, (const char *)packet, MAVLINK_MSG_ID_INSTRUMENTATION_MIN_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_LEN, MAVLINK_MSG_ID_INSTRUMENTATION_CRC);
#endif
}
#endif

#endif

// MESSAGE INSTRUMENTATION UNPACKING


/**
 * @brief Get field current_zero from instrumentation message
 *
 * @return [mA] Current Sensor 0
 */
static inline float mavlink_msg_instrumentation_get_current_zero(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current_one from instrumentation message
 *
 * @return [mA] Current sensor 1.
 */
static inline float mavlink_msg_instrumentation_get_current_one(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current_two from instrumentation message
 *
 * @return [mA] Current sensor 2.
 */
static inline float mavlink_msg_instrumentation_get_current_two(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field voltage_battery from instrumentation message
 *
 * @return [mA] Voltage sensor.
 */
static inline float mavlink_msg_instrumentation_get_voltage_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a instrumentation message into a struct
 *
 * @param msg The message to decode
 * @param instrumentation C-struct to decode the message contents into
 */
static inline void mavlink_msg_instrumentation_decode(const mavlink_message_t* msg, mavlink_instrumentation_t* instrumentation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    instrumentation->current_zero = mavlink_msg_instrumentation_get_current_zero(msg);
    instrumentation->current_one = mavlink_msg_instrumentation_get_current_one(msg);
    instrumentation->current_two = mavlink_msg_instrumentation_get_current_two(msg);
    instrumentation->voltage_battery = mavlink_msg_instrumentation_get_voltage_battery(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INSTRUMENTATION_LEN? msg->len : MAVLINK_MSG_ID_INSTRUMENTATION_LEN;
        memset(instrumentation, 0, MAVLINK_MSG_ID_INSTRUMENTATION_LEN);
    memcpy(instrumentation, _MAV_PAYLOAD(msg), len);
#endif
}
