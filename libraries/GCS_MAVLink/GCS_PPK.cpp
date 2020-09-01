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
#include "GCS.h"
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

void GCS_MAVLINK::handle_raw_data_message(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_LIST:
        handle_raw_data_request_list(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_DATA:
        handle_raw_data_request_data(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_RAW_DATA_ERASE:
        handle_raw_data_request_erase(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_END:
        handle_raw_data_request_end(msg, dataflash);
        break;
    }
}

void GCS_MAVLINK::handle_pos_data_message(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_LIST:
        handle_pos_data_request_list(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_DATA:
        handle_pos_data_request_data(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_POS_DATA_ERASE:
        handle_pos_data_request_erase(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_END:
        handle_pos_data_request_end(msg, dataflash);
        break;
    }
}

/**
   handle all types of ppk download requests from the GCS
 */
void GCS_MAVLINK::handle_ppk_message(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_LIST:
	/* fall through */
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_DATA:
	/* fall through */
    case MAVLINK_MSG_ID_RAW_DATA_ERASE:
        /* fall through */
    case MAVLINK_MSG_ID_RAW_DATA_REQUEST_END:
        handle_raw_data_message(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_LIST:
	/* fall through */
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_DATA:
	/* fall through */
    case MAVLINK_MSG_ID_POS_DATA_ERASE:
	/* fall through */
    case MAVLINK_MSG_ID_POS_DATA_REQUEST_END:
	handle_pos_data_message(msg, dataflash);
	break;
    }
}

/**
   handle all types of raw data download requests from the GCS
 */
void GCS_MAVLINK::handle_raw_data_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_raw_data_request_list_t packet;
    mavlink_msg_raw_data_request_list_decode(msg, &packet);

    _raw_data_listing = false;
    _raw_data_sending = false;

    _raw_data_num = dataflash.get_num_raw_data();
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

    _raw_data_listing = true;
    handle_raw_data_send_listing(dataflash);
}


/**
   handle request for raw_data data
 */
void GCS_MAVLINK::handle_raw_data_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_raw_data_request_data_t packet;
    mavlink_msg_raw_data_request_data_decode(msg, &packet);

    DataFlash_Class::instance()->set_in_raw_data_download(true);

    _raw_data_listing = false;
    if (!_raw_data_sending || _raw_data_num_data != packet.id) {
        _raw_data_sending = false;

        uint16_t num_raw_datas = dataflash.get_num_raw_data();
        if (packet.id > num_raw_datas || packet.id < 1) {
            return;
        }

        uint32_t time_utc, size;
        dataflash.get_raw_data_info(packet.id, size, time_utc);
        _raw_data_num_data = packet.id;
        _raw_data_data_size = size;

        uint16_t end;
        dataflash.get_raw_data_boundaries(packet.id, _raw_data_data_page, end);
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

    handle_raw_data_send(dataflash);
}

/**
   handle request to erase raw_data data
 */
void GCS_MAVLINK::handle_raw_data_request_erase(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_raw_data_erase_t packet;
    mavlink_msg_raw_data_erase_decode(msg, &packet);

    dataflash.EraseAllRawData();
}

/**
   handle request to stop transfer and resume normal raw_dataging
 */
void GCS_MAVLINK::handle_raw_data_request_end(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_raw_data_request_end_t packet;
    mavlink_msg_raw_data_request_end_decode(msg, &packet);
    DataFlash_Class::instance()->set_in_raw_data_download(false);
    _raw_data_sending = false;
}

/**
   trigger sending of raw_data messages if there are some pending
 */
void GCS_MAVLINK::handle_raw_data_send(DataFlash_Class &dataflash)
{
    if (_raw_data_listing) {
        handle_raw_data_send_listing(dataflash);
    }
    if (!_raw_data_sending) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of raw_data download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_raw_data_sending) {
            if (!handle_raw_data_send_data(dataflash)) break;
        }
    }
}

/**
   trigger sending of raw_data messages if there are some pending
 */
void GCS_MAVLINK::handle_raw_data_send_listing(DataFlash_Class &dataflash)
{
    if (!HAVE_PAYLOAD_SPACE(chan, LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - last_heartbeat_time > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_raw_data_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        dataflash.get_raw_data_info(_raw_data_next_list_entry, size, time_utc);
    }
    mavlink_msg_raw_data_entry_send(chan, _raw_data_next_list_entry, _raw_data_num, _raw_data_last_list_entry, time_utc, size);
    if (_raw_data_next_list_entry == _raw_data_last_list_entry) {
        _raw_data_listing = false;
    } else {
        _raw_data_next_list_entry++;
    }
}

/**
   trigger sending of raw_data data if there are some pending
 */
bool GCS_MAVLINK::handle_raw_data_send_data(DataFlash_Class &dataflash)
{
    if (!HAVE_PAYLOAD_SPACE(chan, LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - last_heartbeat_time > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _raw_data_data_remaining;
    mavlink_raw_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = dataflash.get_raw_data(_raw_data_num_data, _raw_data_data_page, _raw_data_data_offset, len, packet.data);
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
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_DATA, (const char *)&packet, 
                                    MAVLINK_MSG_ID_RAW_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_RAW_DATA_LEN,
                                    MAVLINK_MSG_ID_RAW_DATA_CRC);

    _raw_data_data_offset += len;
    _raw_data_data_remaining -= len;
	//printf("_raw_data_data_offset[%d], _raw_data_data_remaining[%d]\n", _raw_data_data_offset, _raw_data_data_remaining);
    if (ret < 90 || _raw_data_data_remaining == 0) {
        _raw_data_sending = false;
    }
    return true;
}

/**
   handle all types of pos data download requests from the GCS
 */
void GCS_MAVLINK::handle_pos_data_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_pos_data_request_list_t packet;
    mavlink_msg_pos_data_request_list_decode(msg, &packet);

    _pos_data_listing = false;
    _pos_data_sending = false;

    _pos_data_num = dataflash.get_num_pos_data();
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

    _pos_data_listing = true;
    handle_pos_data_send_listing(dataflash);
}


/**
   handle request for pos_data data
 */
void GCS_MAVLINK::handle_pos_data_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_pos_data_request_data_t packet;
    mavlink_msg_pos_data_request_data_decode(msg, &packet);

    DataFlash_Class::instance()->set_in_pos_data_download(true);

    _pos_data_listing = false;
    if (!_pos_data_sending || _pos_data_num_data != packet.id) {
        _pos_data_sending = false;

        uint16_t num_pos_datas = dataflash.get_num_pos_data();
        if (packet.id > num_pos_datas || packet.id < 1) {
            return;
        }

        uint32_t time_utc, size;
        dataflash.get_pos_data_info(packet.id, size, time_utc);
        _pos_data_num_data = packet.id;
        _pos_data_data_size = size;

        uint16_t end;
        dataflash.get_pos_data_boundaries(packet.id, _pos_data_data_page, end);
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

    handle_pos_data_send(dataflash);
}

/**
   handle request to erase pos_data data
 */
void GCS_MAVLINK::handle_pos_data_request_erase(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_pos_data_erase_t packet;
    mavlink_msg_pos_data_erase_decode(msg, &packet);

    dataflash.EraseAllPosData();
}

/**
   handle request to stop transfer and resume normal pos_dataging
 */
void GCS_MAVLINK::handle_pos_data_request_end(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_pos_data_request_end_t packet;
    mavlink_msg_pos_data_request_end_decode(msg, &packet);
    DataFlash_Class::instance()->set_in_pos_data_download(false);
    _pos_data_sending = false;
}

/**
   trigger sending of pos_data messages if there are some pending
 */
void GCS_MAVLINK::handle_pos_data_send(DataFlash_Class &dataflash)
{
    if (_pos_data_listing) {
        handle_pos_data_send_listing(dataflash);
    }
    if (!_pos_data_sending) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of pos_data download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_pos_data_sending) {
            if (!handle_pos_data_send_data(dataflash)) break;
        }
    }
}

/**
   trigger sending of pos_data messages if there are some pending
 */
void GCS_MAVLINK::handle_pos_data_send_listing(DataFlash_Class &dataflash)
{
    if (!HAVE_PAYLOAD_SPACE(chan, LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - last_heartbeat_time > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_pos_data_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        dataflash.get_pos_data_info(_pos_data_next_list_entry, size, time_utc);
    }
    mavlink_msg_pos_data_entry_send(chan, _pos_data_next_list_entry, _pos_data_num, _pos_data_last_list_entry, time_utc, size);
    if (_pos_data_next_list_entry == _pos_data_last_list_entry) {
        _pos_data_listing = false;
    } else {
        _pos_data_next_list_entry++;
    }
}

/**
   trigger sending of pos_data data if there are some pending
 */
bool GCS_MAVLINK::handle_pos_data_send_data(DataFlash_Class &dataflash)
{
    if (!HAVE_PAYLOAD_SPACE(chan, LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - last_heartbeat_time > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _pos_data_data_remaining;
    mavlink_pos_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = dataflash.get_pos_data(_pos_data_num_data, _pos_data_data_page, _pos_data_data_offset, len, packet.data);
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
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POS_DATA, (const char *)&packet, 
                                    MAVLINK_MSG_ID_POS_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_POS_DATA_LEN,
                                    MAVLINK_MSG_ID_POS_DATA_CRC);

    _pos_data_data_offset += len;
    _pos_data_data_remaining -= len;
    if (ret < 90 || _pos_data_data_remaining == 0) {
        _pos_data_sending = false;
    }
    return true;
}

