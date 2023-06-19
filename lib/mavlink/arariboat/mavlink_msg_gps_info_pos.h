#pragma once
// MESSAGE GPS_INFO_POS PACKING

#define MAVLINK_MSG_ID_GPS_INFO_POS 174


typedef struct __mavlink_gps_info_pos_t {
 float latitude; /*<  Latitude info. Sixth decimal digit represents 11cm resolution*/
 float longitude; /*<  Longitude info. Sixth decimal digit represents 11cm resolution*/
} mavlink_gps_info_pos_t;

#define MAVLINK_MSG_ID_GPS_INFO_POS_LEN 8
#define MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN 8
#define MAVLINK_MSG_ID_174_LEN 8
#define MAVLINK_MSG_ID_174_MIN_LEN 8

#define MAVLINK_MSG_ID_GPS_INFO_POS_CRC 85
#define MAVLINK_MSG_ID_174_CRC 85



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_INFO_POS { \
    174, \
    "GPS_INFO_POS", \
    2, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps_info_pos_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gps_info_pos_t, longitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_INFO_POS { \
    "GPS_INFO_POS", \
    2, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps_info_pos_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gps_info_pos_t, longitude) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_info_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  Latitude info. Sixth decimal digit represents 11cm resolution
 * @param longitude  Longitude info. Sixth decimal digit represents 11cm resolution
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_info_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INFO_POS_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INFO_POS_LEN);
#else
    mavlink_gps_info_pos_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INFO_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INFO_POS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
}

/**
 * @brief Pack a gps_info_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  Latitude info. Sixth decimal digit represents 11cm resolution
 * @param longitude  Longitude info. Sixth decimal digit represents 11cm resolution
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_info_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float latitude,float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INFO_POS_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INFO_POS_LEN);
#else
    mavlink_gps_info_pos_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INFO_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INFO_POS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
}

/**
 * @brief Encode a gps_info_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_info_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_info_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_info_pos_t* gps_info_pos)
{
    return mavlink_msg_gps_info_pos_pack(system_id, component_id, msg, gps_info_pos->latitude, gps_info_pos->longitude);
}

/**
 * @brief Encode a gps_info_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_info_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_info_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_info_pos_t* gps_info_pos)
{
    return mavlink_msg_gps_info_pos_pack_chan(system_id, component_id, chan, msg, gps_info_pos->latitude, gps_info_pos->longitude);
}

/**
 * @brief Send a gps_info_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  Latitude info. Sixth decimal digit represents 11cm resolution
 * @param longitude  Longitude info. Sixth decimal digit represents 11cm resolution
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_info_pos_send(mavlink_channel_t chan, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INFO_POS_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO_POS, buf, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
#else
    mavlink_gps_info_pos_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO_POS, (const char *)&packet, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
#endif
}

/**
 * @brief Send a gps_info_pos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_info_pos_send_struct(mavlink_channel_t chan, const mavlink_gps_info_pos_t* gps_info_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_info_pos_send(chan, gps_info_pos->latitude, gps_info_pos->longitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO_POS, (const char *)gps_info_pos, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_INFO_POS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_info_pos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO_POS, buf, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
#else
    mavlink_gps_info_pos_t *packet = (mavlink_gps_info_pos_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO_POS, (const char *)packet, MAVLINK_MSG_ID_GPS_INFO_POS_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_LEN, MAVLINK_MSG_ID_GPS_INFO_POS_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_INFO_POS UNPACKING


/**
 * @brief Get field latitude from gps_info_pos message
 *
 * @return  Latitude info. Sixth decimal digit represents 11cm resolution
 */
static inline float mavlink_msg_gps_info_pos_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from gps_info_pos message
 *
 * @return  Longitude info. Sixth decimal digit represents 11cm resolution
 */
static inline float mavlink_msg_gps_info_pos_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a gps_info_pos message into a struct
 *
 * @param msg The message to decode
 * @param gps_info_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_info_pos_decode(const mavlink_message_t* msg, mavlink_gps_info_pos_t* gps_info_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_info_pos->latitude = mavlink_msg_gps_info_pos_get_latitude(msg);
    gps_info_pos->longitude = mavlink_msg_gps_info_pos_get_longitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_INFO_POS_LEN? msg->len : MAVLINK_MSG_ID_GPS_INFO_POS_LEN;
        memset(gps_info_pos, 0, MAVLINK_MSG_ID_GPS_INFO_POS_LEN);
    memcpy(gps_info_pos, _MAV_PAYLOAD(msg), len);
#endif
}
