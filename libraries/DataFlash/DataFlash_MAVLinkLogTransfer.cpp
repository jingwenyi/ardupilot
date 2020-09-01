/*
  MAVLink logfile transfer functions
 */

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h> // for LOG_ENTRY

extern const AP_HAL::HAL& hal;


// We avoid doing log messages when timing is critical:
bool DataFlash_Class::should_handle_pos_data_message()
{
/*
    if (!WritesEnabled()) {
        // this is currently used as a proxy for "in_mavlink_delay"
        return false;
    }
 */
    if (vehicle_is_armed()) {
        return false;
    }
    return true;
}


/**
   handle all types of log download requests from the GCS
 */
void DataFlash_Class::handle_pos_data_message(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (!should_handle_pos_data_message()) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_LIST:
        handle_pos_data_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_DATA:
        handle_pos_data_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_POS_DATA_ERASE:
        handle_pos_data_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_END:
        handle_pos_data_request_end(link, msg);
        break;
    }
}

/**
   handle all types of pos data download requests from the GCS
 */
void DataFlash_Class::handle_pos_data_request_list(GCS_MAVLINK &link, mavlink_message_t *msg)
{
	//printf("_pos_data_sending_chan is %d\n", _pos_data_sending_chan);

    if (_pos_data_sending_chan >= 0) {
        link.send_text(MAV_SEVERITY_INFO, "Pos data download in progress");
        return;
    }

    mavlink_pos_data_request_list_t packet;
    mavlink_msg_pos_data_request_list_decode(msg, &packet);

    _pos_data_listing = false;
    _pos_data_sending = false;

    _pos_data_num = get_num_pos_data();
	//printf("get_num_pos_data %d, [%d] [%d]\n", _pos_data_num, packet.start, packet.end);
    if (_pos_data_num == 0) {
        _pos_data_next_list_entry = 0;
        _pos_data_last_list_entry = 0;        
    } else {
        _pos_data_next_list_entry = packet.start;
        _pos_data_last_list_entry = packet.end;

        if (_pos_data_last_list_entry > _pos_data_num) {
            _pos_data_last_list_entry = _pos_data_num;
        }
        if (_pos_data_next_list_entry < 1) {
            _pos_data_next_list_entry = 1;
        }
    }
	//printf("_pos_data_next_list_entry[%d], _pos_data_last_list_entry[%d]\n", _pos_data_next_list_entry, _pos_data_last_list_entry);
    _pos_data_listing = true;
    _pos_data_sending_chan = link.get_chan();
    handle_pos_data_send_listing(link);
}

/**
   trigger sending of pos data messages if there are some pending
 */
void DataFlash_Class::handle_pos_data_send_listing(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_pos_data_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        get_pos_data_info(_pos_data_next_list_entry, size, time_utc);
    	//printf("get_pos_data_info %d %d\n", size, time_utc);
    }
    mavlink_msg_pos_data_entry_send(link.get_chan(), _pos_data_next_list_entry, _pos_data_num, _pos_data_last_list_entry, time_utc, size);
    if (_pos_data_next_list_entry == _pos_data_last_list_entry) {
        _pos_data_listing = false;
        _pos_data_sending_chan = -1;
    } else {
        _pos_data_next_list_entry++;
    }
}

/**
   handle request for pos data
 */
void DataFlash_Class::handle_pos_data_request_data(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (_pos_data_sending_chan >= 0) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_pos_data_sending_chan != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Pos download in progress");
        }
        return;
    }

    mavlink_pos_data_request_data_t packet;
    mavlink_msg_pos_data_request_data_decode(msg, &packet);

    _in_pos_data_download = true;

    _pos_data_listing = false;
    if (!_pos_data_sending || _pos_data_num_data != packet.id) {
        _pos_data_sending = false;

        uint16_t num_pos_data = get_num_pos_data();
        if (packet.id > num_pos_data || packet.id < 1) {
            return;
        }

        uint32_t time_utc, size;
        get_pos_data_info(packet.id, size, time_utc);
        _pos_data_num_data = packet.id;
        _pos_data_data_size = size;

        uint16_t end;
        get_pos_data_boundaries(packet.id, _pos_data_data_page, end);
    }

    _pos_data_data_offset = packet.ofs;
    if (_pos_data_data_offset >= _pos_data_data_size) {
        _pos_data_data_remaining = 0;
    } else {
        _pos_data_data_remaining = _pos_data_data_size - _pos_data_data_offset;
    }
    if (_pos_data_data_remaining > packet.count) {
        _pos_data_data_remaining = packet.count;
    }
    _pos_data_sending = true;
    _pos_data_sending_chan = link.get_chan();

    handle_pos_data_send(link);
}

/**
   handle request to erase pos data
 */
void DataFlash_Class::handle_pos_data_request_erase(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    //mavlink_pos_data_erase_t packet;
    //mavlink_msg_pos_data_erase_decode(msg, &packet);

    EraseAllPosData();
}

/**
   handle request to stop transfer and resume normal pos
 */
void DataFlash_Class::handle_pos_data_request_end(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    mavlink_pos_data_request_end_t packet;
    mavlink_msg_pos_data_request_end_decode(msg, &packet);
    _in_pos_data_download = false;
    _pos_data_sending = false;
    _pos_data_sending_chan = -1;
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_pos_data_send(GCS_MAVLINK &link)
{
    if (_pos_data_sending_chan != link.get_chan()) {
        return;
    }
    if (_pos_data_listing) {
        handle_pos_data_send_listing(link);
    }
    if (!_pos_data_sending) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (link.is_high_bandwidth() && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (link.have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_pos_data_sending) {
            if (!handle_pos_data_send_data(link)) break;
        }
    }
}

/**
   trigger sending of raw data if there are some pending
 */
bool DataFlash_Class::handle_pos_data_send_data(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _pos_data_data_remaining;
	mavlink_pos_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = get_pos_data(_pos_data_num_data, _pos_data_data_page, _pos_data_data_offset, len, packet.data);
    if (ret < 0) {
        // report as EOF on error
        ret = 0;
    }
    if (ret < 90) {
        memset(&packet.data[ret], 0, 90-ret);
    }

    packet.ofs = _pos_data_data_offset;
    packet.id = _pos_data_num_data;
    packet.count = ret;
    _mav_finalize_message_chan_send(link.get_chan(), MAVLINK_MSG_ID_POS_DATA, (const char *)&packet,
                                    MAVLINK_MSG_ID_POS_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_POS_DATA_LEN,
                                    MAVLINK_MSG_ID_POS_DATA_CRC);

    _pos_data_data_offset += len;
    _pos_data_data_remaining -= len;
    if (ret < 90 || _pos_data_data_remaining == 0) {
        _pos_data_sending = false;
        _pos_data_sending_chan = -1;
    }
    return true;
}

// We avoid doing log messages when timing is critical:
bool DataFlash_Class::should_handle_raw_data_message()
{
/*
    if (!WritesEnabled()) {
        // this is currently used as a proxy for "in_mavlink_delay"
        return false;
    }
 */
    if (vehicle_is_armed()) {
        return false;
    }
    return true;
}

/**
   handle all types of log download requests from the GCS
 */
void DataFlash_Class::handle_raw_data_message(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (!should_handle_raw_data_message()) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_LIST:
        handle_raw_data_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_DATA:
        handle_raw_data_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_RAW_DATA_ERASE:
        handle_raw_data_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_END:
        handle_raw_data_request_end(link, msg);
        break;
    }
}

/**
   handle all types of raw data download requests from the GCS
 */
void DataFlash_Class::handle_raw_data_request_list(GCS_MAVLINK &link, mavlink_message_t *msg)
{
	//printf("_raw_data_sending_chan is %d\n", _raw_data_sending_chan);

    if (_raw_data_sending_chan >= 0) {
        link.send_text(MAV_SEVERITY_INFO, "Raw data download in progress");
        return;
    }

    mavlink_raw_data_request_list_t packet;
    mavlink_msg_raw_data_request_list_decode(msg, &packet);

    _raw_data_listing = false;
    _raw_data_sending = false;

    _raw_data_num = get_num_raw_data();
	//printf("get_num_raw_data %d, [%d] [%d]\n", _raw_data_num, packet.start, packet.end);
    if (_raw_data_num == 0) {
        _raw_data_next_list_entry = 0;
        _raw_data_last_list_entry = 0;        
    } else {
        _raw_data_next_list_entry = packet.start;
        _raw_data_last_list_entry = packet.end;

        if (_raw_data_last_list_entry > _raw_data_num) {
            _raw_data_last_list_entry = _raw_data_num;
        }
        if (_raw_data_next_list_entry < 1) {
            _raw_data_next_list_entry = 1;
        }
    }
	//printf("_raw_data_next_list_entry[%d], _raw_data_last_list_entry[%d]\n", _raw_data_next_list_entry, _raw_data_last_list_entry);
    _raw_data_listing = true;
    _raw_data_sending_chan = link.get_chan();
    handle_raw_data_send_listing(link);
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_raw_data_send_listing(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_raw_data_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        get_raw_data_info(_raw_data_next_list_entry, size, time_utc);
    	//printf("get_raw_data_info %d %d\n", size, time_utc);
    }
    mavlink_msg_raw_data_entry_send(link.get_chan(), _raw_data_next_list_entry, _raw_data_num, _raw_data_last_list_entry, time_utc, size);
    if (_raw_data_next_list_entry == _raw_data_last_list_entry) {
        _raw_data_listing = false;
        _raw_data_sending_chan = -1;
    } else {
        _raw_data_next_list_entry++;
    }
}


/**
   handle request for raw data
 */
void DataFlash_Class::handle_raw_data_request_data(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (_raw_data_sending_chan >= 0) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_raw_data_sending_chan != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Raw download in progress");
        }
        return;
    }

    mavlink_raw_data_request_data_t packet;
    mavlink_msg_raw_data_request_data_decode(msg, &packet);

    _in_raw_data_download = true;

    _raw_data_listing = false;
    if (!_raw_data_sending || _raw_data_num_data != packet.id) {
        _raw_data_sending = false;

        uint16_t num_raw_data = get_num_raw_data();
        if (packet.id > num_raw_data || packet.id < 1) {
            return;
        }

        uint32_t time_utc, size;
        get_raw_data_info(packet.id, size, time_utc);
        _raw_data_num_data = packet.id;
        _raw_data_data_size = size;

        uint16_t end;
        get_raw_data_boundaries(packet.id, _raw_data_data_page, end);
    }

    _raw_data_data_offset = packet.ofs;
    if (_raw_data_data_offset >= _raw_data_data_size) {
        _raw_data_data_remaining = 0;
    } else {
        _raw_data_data_remaining = _raw_data_data_size - _raw_data_data_offset;
    }
    if (_raw_data_data_remaining > packet.count) {
        _raw_data_data_remaining = packet.count;
    }
    _raw_data_sending = true;
    _raw_data_sending_chan = link.get_chan();

    handle_raw_data_send(link);
}

/**
   handle request to erase raw data
 */
void DataFlash_Class::handle_raw_data_request_erase(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    //mavlink_raw_data_erase_t packet;
    //mavlink_msg_raw_data_erase_decode(msg, &packet);

    EraseAllRawData();
}

/**
   handle request to stop transfer and resume normal logging
 */
void DataFlash_Class::handle_raw_data_request_end(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    mavlink_raw_data_request_end_t packet;
    mavlink_msg_raw_data_request_end_decode(msg, &packet);
    _in_raw_data_download = false;
    _raw_data_sending = false;
    _raw_data_sending_chan = -1;
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_raw_data_send(GCS_MAVLINK &link)
{
    if (_raw_data_sending_chan != link.get_chan()) {
        return;
    }
    if (_raw_data_listing) {
        handle_raw_data_send_listing(link);
    }
    if (!_raw_data_sending) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (link.is_high_bandwidth() && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (link.have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_raw_data_sending) {
            if (!handle_raw_data_send_data(link)) break;
        }
    }
}


/**
   trigger sending of raw data if there are some pending
 */
bool DataFlash_Class::handle_raw_data_send_data(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _raw_data_data_remaining;
	mavlink_raw_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = get_raw_data(_raw_data_num_data, _raw_data_data_page, _raw_data_data_offset, len, packet.data);
    if (ret < 0) {
        // report as EOF on error
        ret = 0;
    }
    if (ret < 90) {
        memset(&packet.data[ret], 0, 90-ret);
    }

    packet.ofs = _raw_data_data_offset;
    packet.id = _raw_data_num_data;
    packet.count = ret;
    _mav_finalize_message_chan_send(link.get_chan(), MAVLINK_MSG_ID_RAW_DATA, (const char *)&packet,
                                    MAVLINK_MSG_ID_RAW_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_RAW_DATA_LEN,
                                    MAVLINK_MSG_ID_RAW_DATA_CRC);

    _raw_data_data_offset += len;
    _raw_data_data_remaining -= len;
    if (ret < 90 || _raw_data_data_remaining == 0) {
        _raw_data_sending = false;
        _raw_data_sending_chan = -1;
    }
    return true;
}


// We avoid doing log messages when timing is critical:
bool DataFlash_Class::should_handle_log_message()
{
    if (!WritesEnabled()) {
        // this is currently used as a proxy for "in_mavlink_delay"
        return false;
    }
    if (vehicle_is_armed()) {
        return false;
    }
    return true;
}

/**
   handle all types of log download requests from the GCS
 */
void DataFlash_Class::handle_log_message(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (!should_handle_log_message()) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        handle_log_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        handle_log_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        handle_log_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_request_end(link, msg);
        break;
    }
}

/**
   handle all types of log download requests from the GCS
 */
void DataFlash_Class::handle_log_request_list(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (_log_sending_chan >= 0) {
        link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        return;
    }

    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(msg, &packet);

    _log_listing = false;
    _log_sending = false;

    _log_num_logs = get_num_logs();
    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        _log_next_list_entry = packet.start;
        _log_last_list_entry = packet.end;

        if (_log_last_list_entry > _log_num_logs) {
            _log_last_list_entry = _log_num_logs;
        }
        if (_log_next_list_entry < 1) {
            _log_next_list_entry = 1;
        }
    }
	
    _log_listing = true;
    _log_sending_chan = link.get_chan();
    handle_log_send_listing(link);
}


/**
   handle request for log data
 */
void DataFlash_Class::handle_log_request_data(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (_log_sending_chan >= 0) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_log_sending_chan != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        }
        return;
    }

    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(msg, &packet);

    _in_log_download = true;

    _log_listing = false;
    if (!_log_sending || _log_num_data != packet.id) {
        _log_sending = false;

        uint16_t num_logs = get_num_logs();
        if (packet.id > num_logs || packet.id < 1) {
            return;
        }

        uint32_t time_utc, size;
        get_log_info(packet.id, size, time_utc);
        _log_num_data = packet.id;
        _log_data_size = size;

        uint16_t end;
        get_log_boundaries(packet.id, _log_data_page, end);
    }

    _log_data_offset = packet.ofs;
    if (_log_data_offset >= _log_data_size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = _log_data_size - _log_data_offset;
    }
    if (_log_data_remaining > packet.count) {
        _log_data_remaining = packet.count;
    }
    _log_sending = true;
    _log_sending_chan = link.get_chan();

    handle_log_send(link);
}

/**
   handle request to erase log data
 */
void DataFlash_Class::handle_log_request_erase(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    // mavlink_log_erase_t packet;
    // mavlink_msg_log_erase_decode(msg, &packet);

    EraseAll();
}

/**
   handle request to stop transfer and resume normal logging
 */
void DataFlash_Class::handle_log_request_end(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    mavlink_log_request_end_t packet;
    mavlink_msg_log_request_end_decode(msg, &packet);
    _in_log_download = false;
    _log_sending = false;
    _log_sending_chan = -1;
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_log_send(GCS_MAVLINK &link)
{
    if (_log_sending_chan != link.get_chan()) {
        return;
    }
    if (_log_listing) {
        handle_log_send_listing(link);
    }
    if (!_log_sending) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (link.is_high_bandwidth() && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (link.have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_log_sending) {
            if (!handle_log_send_data(link)) break;
        }
    }
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_log_send_listing(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_log_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        get_log_info(_log_next_list_entry, size, time_utc);
    }
    mavlink_msg_log_entry_send(link.get_chan(), _log_next_list_entry, _log_num_logs, _log_last_list_entry, time_utc, size);
    if (_log_next_list_entry == _log_last_list_entry) {
        _log_listing = false;
        _log_sending_chan = -1;
    } else {
        _log_next_list_entry++;
    }
}

/**
   trigger sending of log data if there are some pending
 */
bool DataFlash_Class::handle_log_send_data(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _log_data_remaining;
	mavlink_log_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, packet.data);
    if (ret < 0) {
        // report as EOF on error
        ret = 0;
    }
    if (ret < 90) {
        memset(&packet.data[ret], 0, 90-ret);
    }

    packet.ofs = _log_data_offset;
    packet.id = _log_num_data;
    packet.count = ret;
    _mav_finalize_message_chan_send(link.get_chan(), MAVLINK_MSG_ID_LOG_DATA, (const char *)&packet,
                                    MAVLINK_MSG_ID_LOG_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_CRC);

    _log_data_offset += len;
    _log_data_remaining -= len;
    if (ret < 90 || _log_data_remaining == 0) {
        _log_sending = false;
        _log_sending_chan = -1;
    }
    return true;
}
