#ifndef _CH55xduino_NRFLite_
#define _CH55xduino_NRFLite_

#include "nRF24L01.h"
#include <Arduino.h>

typedef enum { BITRATE2MBPS, BITRATE1MBPS, BITRATE250KBPS } Bitrate;
typedef enum { REQUIRE_ACK, NO_ACK } Send_Type;

bool NRFLite_init(uint8_t _radio_id, uint8_t _ce_pin, uint8_t _csn_pin, Bitrate _bitrate, uint8_t _channel, bool _begin_spi);
void NRFLite_set_address_prefix(uint8_t _byte1, uint8_t _byte2, uint8_t _byte3, uint8_t _byte4);

uint8_t NRFLite_has_data();
void NRFLite_read_data(void *_data);
bool NRFLite_send_data(__data uint8_t _to_radio_id, void *_data, __data uint8_t _length, __data Send_Type _send_type);

#endif