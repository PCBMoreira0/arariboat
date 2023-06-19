#pragma once
// MESSAGE TEMPERATURES PACKING

#define MAVLINK_MSG_ID_TEMPERATURES 172


typedef struct __mavlink_temperatures_t {
 float temperature_motor; /*< [degC] Motor temperature.*/
 float temperature_mppt; /*< [degC] MPPT temperature.*/
} mavlink_temperatures_t;

#define MAVLINK_MSG_ID_TEMPERATURES_LEN 8
#define MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN 8
#define MAVLINK_MSG_ID_172_LEN 8
#define MAVLINK_MSG_ID_172_MIN_LEN 8

#define MAVLINK_MSG_ID_TEMPERATURES_CRC 159
#define MAVLINK_MSG_ID_172_CRC 159



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TEMPERATURES { \
    172, \
    "TEMPERATURES", \
    2, \
    {  { "temperature_motor", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_temperatures_t, temperature_motor) }, \
         { "temperature_mppt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_temperatures_t, temperature_mppt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TEMPERATURES { \
    "TEMPERATURES", \
    2, \
    {  { "temperature_motor", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_temperatures_t, temperature_motor) }, \
         { "temperature_mppt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_temperatures_t, temperature_mppt) }, \
         } \
}
#endif

/**
 * @brief Pack a temperatures message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature_motor [degC] Motor temperature.
 * @param temperature_mppt [degC] MPPT temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_temperatures_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float temperature_motor, float temperature_mppt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TEMPERATURES_LEN];
    _mav_put_float(buf, 0, temperature_motor);
    _mav_put_float(buf, 4, temperature_mppt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEMPERATURES_LEN);
#else
    mavlink_temperatures_t packet;
    packet.temperature_motor = temperature_motor;
    packet.temperature_mppt = temperature_mppt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEMPERATURES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TEMPERATURES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
}

/**
 * @brief Pack a temperatures message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature_motor [degC] Motor temperature.
 * @param temperature_mppt [degC] MPPT temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_temperatures_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float temperature_motor,float temperature_mppt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TEMPERATURES_LEN];
    _mav_put_float(buf, 0, temperature_motor);
    _mav_put_float(buf, 4, temperature_mppt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEMPERATURES_LEN);
#else
    mavlink_temperatures_t packet;
    packet.temperature_motor = temperature_motor;
    packet.temperature_mppt = temperature_mppt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEMPERATURES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TEMPERATURES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
}

/**
 * @brief Encode a temperatures struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param temperatures C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_temperatures_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_temperatures_t* temperatures)
{
    return mavlink_msg_temperatures_pack(system_id, component_id, msg, temperatures->temperature_motor, temperatures->temperature_mppt);
}

/**
 * @brief Encode a temperatures struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperatures C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_temperatures_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_temperatures_t* temperatures)
{
    return mavlink_msg_temperatures_pack_chan(system_id, component_id, chan, msg, temperatures->temperature_motor, temperatures->temperature_mppt);
}

/**
 * @brief Send a temperatures message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature_motor [degC] Motor temperature.
 * @param temperature_mppt [degC] MPPT temperature.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_temperatures_send(mavlink_channel_t chan, float temperature_motor, float temperature_mppt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TEMPERATURES_LEN];
    _mav_put_float(buf, 0, temperature_motor);
    _mav_put_float(buf, 4, temperature_mppt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEMPERATURES, buf, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
#else
    mavlink_temperatures_t packet;
    packet.temperature_motor = temperature_motor;
    packet.temperature_mppt = temperature_mppt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEMPERATURES, (const char *)&packet, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
#endif
}

/**
 * @brief Send a temperatures message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_temperatures_send_struct(mavlink_channel_t chan, const mavlink_temperatures_t* temperatures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_temperatures_send(chan, temperatures->temperature_motor, temperatures->temperature_mppt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEMPERATURES, (const char *)temperatures, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
#endif
}

#if MAVLINK_MSG_ID_TEMPERATURES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_temperatures_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float temperature_motor, float temperature_mppt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temperature_motor);
    _mav_put_float(buf, 4, temperature_mppt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEMPERATURES, buf, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
#else
    mavlink_temperatures_t *packet = (mavlink_temperatures_t *)msgbuf;
    packet->temperature_motor = temperature_motor;
    packet->temperature_mppt = temperature_mppt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEMPERATURES, (const char *)packet, MAVLINK_MSG_ID_TEMPERATURES_MIN_LEN, MAVLINK_MSG_ID_TEMPERATURES_LEN, MAVLINK_MSG_ID_TEMPERATURES_CRC);
#endif
}
#endif

#endif

// MESSAGE TEMPERATURES UNPACKING


/**
 * @brief Get field temperature_motor from temperatures message
 *
 * @return [degC] Motor temperature.
 */
static inline float mavlink_msg_temperatures_get_temperature_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperature_mppt from temperatures message
 *
 * @return [degC] MPPT temperature.
 */
static inline float mavlink_msg_temperatures_get_temperature_mppt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a temperatures message into a struct
 *
 * @param msg The message to decode
 * @param temperatures C-struct to decode the message contents into
 */
static inline void mavlink_msg_temperatures_decode(const mavlink_message_t* msg, mavlink_temperatures_t* temperatures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    temperatures->temperature_motor = mavlink_msg_temperatures_get_temperature_motor(msg);
    temperatures->temperature_mppt = mavlink_msg_temperatures_get_temperature_mppt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TEMPERATURES_LEN? msg->len : MAVLINK_MSG_ID_TEMPERATURES_LEN;
        memset(temperatures, 0, MAVLINK_MSG_ID_TEMPERATURES_LEN);
    memcpy(temperatures, _MAV_PAYLOAD(msg), len);
#endif
}
