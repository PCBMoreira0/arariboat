#pragma once
// MESSAGE GPS_INFO PACKING

#define MAVLINK_MSG_ID_GPS_INFO 173


typedef struct __mavlink_gps_info_t {
 float latitude; /*<  Latitude info. Sixth decimal digit represents 11cm resolution*/
 float longitude; /*<  Longitude info. Sixth decimal digit represents 11cm resolution*/
 float speed; /*< [km/h] Speed*/
 float course; /*< [deg] Course*/
 uint8_t satellites_visible; /*<  Number of visible satellites*/
} mavlink_gps_info_t;

#define MAVLINK_MSG_ID_GPS_INFO_LEN 17
#define MAVLINK_MSG_ID_GPS_INFO_MIN_LEN 17
#define MAVLINK_MSG_ID_173_LEN 17
#define MAVLINK_MSG_ID_173_MIN_LEN 17

#define MAVLINK_MSG_ID_GPS_INFO_CRC 182
#define MAVLINK_MSG_ID_173_CRC 182



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_INFO { \
    173, \
    "GPS_INFO", \
    5, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps_info_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gps_info_t, longitude) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gps_info_t, speed) }, \
         { "course", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gps_info_t, course) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_gps_info_t, satellites_visible) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_INFO { \
    "GPS_INFO", \
    5, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps_info_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gps_info_t, longitude) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gps_info_t, speed) }, \
         { "course", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gps_info_t, course) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_gps_info_t, satellites_visible) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  Latitude info. Sixth decimal digit represents 11cm resolution
 * @param longitude  Longitude info. Sixth decimal digit represents 11cm resolution
 * @param speed [km/h] Speed
 * @param course [deg] Course
 * @param satellites_visible  Number of visible satellites
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float latitude, float longitude, float speed, float course, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INFO_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, course);
    _mav_put_uint8_t(buf, 16, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INFO_LEN);
#else
    mavlink_gps_info_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.speed = speed;
    packet.course = course;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
}

/**
 * @brief Pack a gps_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  Latitude info. Sixth decimal digit represents 11cm resolution
 * @param longitude  Longitude info. Sixth decimal digit represents 11cm resolution
 * @param speed [km/h] Speed
 * @param course [deg] Course
 * @param satellites_visible  Number of visible satellites
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float latitude,float longitude,float speed,float course,uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INFO_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, course);
    _mav_put_uint8_t(buf, 16, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INFO_LEN);
#else
    mavlink_gps_info_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.speed = speed;
    packet.course = course;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
}

/**
 * @brief Encode a gps_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_info_t* gps_info)
{
    return mavlink_msg_gps_info_pack(system_id, component_id, msg, gps_info->latitude, gps_info->longitude, gps_info->speed, gps_info->course, gps_info->satellites_visible);
}

/**
 * @brief Encode a gps_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_info_t* gps_info)
{
    return mavlink_msg_gps_info_pack_chan(system_id, component_id, chan, msg, gps_info->latitude, gps_info->longitude, gps_info->speed, gps_info->course, gps_info->satellites_visible);
}

/**
 * @brief Send a gps_info message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  Latitude info. Sixth decimal digit represents 11cm resolution
 * @param longitude  Longitude info. Sixth decimal digit represents 11cm resolution
 * @param speed [km/h] Speed
 * @param course [deg] Course
 * @param satellites_visible  Number of visible satellites
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_info_send(mavlink_channel_t chan, float latitude, float longitude, float speed, float course, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INFO_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, course);
    _mav_put_uint8_t(buf, 16, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO, buf, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
#else
    mavlink_gps_info_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.speed = speed;
    packet.course = course;
    packet.satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO, (const char *)&packet, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
#endif
}

/**
 * @brief Send a gps_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_info_send_struct(mavlink_channel_t chan, const mavlink_gps_info_t* gps_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_info_send(chan, gps_info->latitude, gps_info->longitude, gps_info->speed, gps_info->course, gps_info->satellites_visible);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO, (const char *)gps_info, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float latitude, float longitude, float speed, float course, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, course);
    _mav_put_uint8_t(buf, 16, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO, buf, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
#else
    mavlink_gps_info_t *packet = (mavlink_gps_info_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->speed = speed;
    packet->course = course;
    packet->satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INFO, (const char *)packet, MAVLINK_MSG_ID_GPS_INFO_MIN_LEN, MAVLINK_MSG_ID_GPS_INFO_LEN, MAVLINK_MSG_ID_GPS_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_INFO UNPACKING


/**
 * @brief Get field latitude from gps_info message
 *
 * @return  Latitude info. Sixth decimal digit represents 11cm resolution
 */
static inline float mavlink_msg_gps_info_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from gps_info message
 *
 * @return  Longitude info. Sixth decimal digit represents 11cm resolution
 */
static inline float mavlink_msg_gps_info_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field speed from gps_info message
 *
 * @return [km/h] Speed
 */
static inline float mavlink_msg_gps_info_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field course from gps_info message
 *
 * @return [deg] Course
 */
static inline float mavlink_msg_gps_info_get_course(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field satellites_visible from gps_info message
 *
 * @return  Number of visible satellites
 */
static inline uint8_t mavlink_msg_gps_info_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a gps_info message into a struct
 *
 * @param msg The message to decode
 * @param gps_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_info_decode(const mavlink_message_t* msg, mavlink_gps_info_t* gps_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_info->latitude = mavlink_msg_gps_info_get_latitude(msg);
    gps_info->longitude = mavlink_msg_gps_info_get_longitude(msg);
    gps_info->speed = mavlink_msg_gps_info_get_speed(msg);
    gps_info->course = mavlink_msg_gps_info_get_course(msg);
    gps_info->satellites_visible = mavlink_msg_gps_info_get_satellites_visible(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_INFO_LEN? msg->len : MAVLINK_MSG_ID_GPS_INFO_LEN;
        memset(gps_info, 0, MAVLINK_MSG_ID_GPS_INFO_LEN);
    memcpy(gps_info, _MAV_PAYLOAD(msg), len);
#endif
}
