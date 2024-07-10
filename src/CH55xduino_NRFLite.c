#include "CH55xduino_NRFLite.h"
#include <SPI.h>

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

static __data uint8_t ADDRESS_PREFIX[4] = {1, 2, 3, 4}; // 1st 4 bytes of addresses, 5th byte will be RadioId.
const static __data uint8_t CONFIG_REG_SETTINGS_FOR_RX_MODE = _BV(PWR_UP) | _BV(PRIM_RX) | _BV(EN_CRC);
#define NRF_SPICLOCK 4000000 // Speed to use for SPI communication with the transceiver.

// Delay used to discharge the radio's CSN pin when operating in 2-pin mode.
// Determined by measuring time to discharge CSN on a 1MHz ATtiny using 0.1uF capacitor and 1K resistor.
#define CSN_DISCHARGE_MICROS 500

#define OFF_TO_POWERDOWN_MILLIS 100     // Vcc > 1.9V power on reset time.
#define POWERDOWN_TO_RXTX_MODE_MILLIS 5 // 4500uS to Standby + 130uS to RX or TX mode, so 5ms is enough.
#define CE_TRANSMISSION_MICROS 10       // Time to initiate data transmission.

#define MAX_NRF_CHANNEL 125

static __data bool initialized = false;
static __data Bitrate bitrate = BITRATE2MBPS;
static __data uint8_t radio_id = 0;
static __data uint8_t pin_ce = 0;
static __data uint8_t pin_csn = 0;
static __data uint8_t channel = 100;

static __data volatile uint8_t *momi_PORT;
static __data volatile uint8_t *momi_DDR;
static __data volatile uint8_t *momi_PIN;
static __data volatile uint8_t *sck_PORT;
static __data uint8_t momi_MASK, sck_MASK;
static __data volatile uint8_t reset_interrupt_flags;
//static uint8_t _useTwoPinSpiTransfer, _usingSeparateCeAndCsnPins;
static __data uint16_t transmission_retry_wait_micros, max_has_data_interval_micros;
static __data int16_t last_to_radio_id = -1;
static __data uint32_t micros_since_last_data_check;

typedef enum { READ_OPERATION, WRITE_OPERATION } Spi_Transfer_Type;

static void NRFLite_spi_transfer(Spi_Transfer_Type _transfer_type, uint8_t _reg_name, void *_data, uint8_t _length){
    uint8_t* int_data = (uint8_t*)_data;

    noInterrupts(); // Prevent an interrupt from interferring with the communication.
    digitalWrite(pin_csn, LOW); // Signal radio to listen to the SPI bus.

    // Transfer with the Arduino SPI library.
    SPI_beginTransaction(SPISettings(NRF_SPICLOCK, MSBFIRST, SPI_MODE0));
    SPI_transfer(_reg_name);

    for(__data uint8_t i = 0; i < _length; ++i){
        __data uint8_t new_data = SPI_transfer(int_data[i]);
        if(_transfer_type == READ_OPERATION) int_data[i] = new_data;
    }

    digitalWrite(pin_csn, HIGH); // Stop radio from listening to the SPI bus.

    interrupts();
}

static void NRFLite_read_register_length(__data uint8_t _reg_name, __data void *_data, __data uint8_t _length){
    NRFLite_spi_transfer(READ_OPERATION, (R_REGISTER | (REGISTER_MASK & _reg_name)), _data, _length);
}

static uint8_t NRFLite_read_register(__data uint8_t _reg_name){
    __data uint8_t data;
    NRFLite_read_register_length(_reg_name, &data, 1);
    return data;
}

static void NRFLite_write_register_length(__data uint8_t _reg_name, void *_data, __data uint8_t _length){
    NRFLite_spi_transfer(WRITE_OPERATION, (W_REGISTER | (REGISTER_MASK & _reg_name)), _data, _length);
}

static void NRFLite_write_register(__data uint8_t _reg_name, __data uint8_t _data){
    NRFLite_write_register_length(_reg_name, &_data, 1);
}

static uint8_t NRFLite_wait_for_tx_to_complete(){
    reset_interrupt_flags = 0; // Disable interrupt flag reset logic in 'whatHappened'.

    __data uint8_t fifo_reg, status_reg;
    __data uint8_t tx_buffer_is_empty;
    __data uint8_t packet_was_sent, packet_could_not_be_sent;
    __data uint8_t tx_attempt_count = 0;
    __data uint8_t result = 0; // Default to indicating a failure.

    // TX buffer can store 3 packets, sends retry up to 15 times, and the retry wait time is about half
    // the time necessary to send a 32 byte packet and receive a 32 byte ACK response.  3 x 15 x 2 = 90
    #define MAX_TX_ATTEMPT_COUNT 90

    while(tx_attempt_count++ < MAX_TX_ATTEMPT_COUNT){
        fifo_reg = NRFLite_read_register(FIFO_STATUS);
        tx_buffer_is_empty = fifo_reg & _BV(TX_EMPTY);

        if(tx_buffer_is_empty){
            result = 1; // Indicate success.
            break;
        }

        digitalWrite(pin_ce, HIGH);
        delayMicroseconds(CE_TRANSMISSION_MICROS);
        digitalWrite(pin_ce, LOW);

        delayMicroseconds(transmission_retry_wait_micros);
        
        status_reg = NRFLite_read_register(STATUS_NRF);
        packet_was_sent = status_reg & _BV(TX_DS);
        packet_could_not_be_sent = status_reg & _BV(MAX_RT);

        if(packet_was_sent){
            NRFLite_write_register(STATUS_NRF, _BV(TX_DS));   // Clear TX success flag.
        }else if(packet_could_not_be_sent){
            NRFLite_spi_transfer(WRITE_OPERATION, FLUSH_TX, NULL, 0); // Clear TX buffer.
            NRFLite_write_register(STATUS_NRF, _BV(MAX_RT));          // Clear max retry flag.
            break;
        }
    }

    reset_interrupt_flags = 1; // Re-enable interrupt reset logic in 'whatHappened'.

    return result;
}

static void NRFLite_prep_for_tx(__data uint8_t _to_radio_id, __data Send_Type _send_type){
    if(last_to_radio_id != _to_radio_id){
        last_to_radio_id = _to_radio_id;

        // TX pipe address sets the destination radio for the data.
        // RX pipe 0 is special and needs the same address in order to receive ACK packets from the destination radio.
        uint8_t address[5] = { ADDRESS_PREFIX[0], ADDRESS_PREFIX[1], ADDRESS_PREFIX[2], ADDRESS_PREFIX[3], _to_radio_id };
        NRFLite_write_register_length(TX_ADDR, &address, 5);
        NRFLite_write_register_length(RX_ADDR_P0, &address, 5);
    }

    // Ensure radio is ready for TX operation.
    __data uint8_t config_reg = NRFLite_read_register(CONFIG);
    __data uint8_t ready_for_tx = (config_reg == (CONFIG_REG_SETTINGS_FOR_RX_MODE & ~_BV(PRIM_RX)));

    if(!ready_for_tx){
        // Put radio into Standby-I mode in order to transition into TX mode.
        digitalWrite(pin_ce, LOW);
        config_reg = CONFIG_REG_SETTINGS_FOR_RX_MODE & ~_BV(PRIM_RX);
        NRFLite_write_register(CONFIG, config_reg);
        delay(POWERDOWN_TO_RXTX_MODE_MILLIS);
    }

    __data uint8_t fifo_reg = NRFLite_read_register(FIFO_STATUS);

    // If RX buffer is full and we require an ACK, clear it so we can receive the ACK response.
    __data uint8_t rx_buffer_is_full = fifo_reg & _BV(RX_FULL);
    if(_send_type == REQUIRE_ACK && rx_buffer_is_full) NRFLite_spi_transfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
    
    // If TX buffer is full, wait for all queued packets to be sent.
    __data uint8_t tx_buffer_is_full = fifo_reg & _BV(FIFO_FULL);
    if(tx_buffer_is_full) NRFLite_wait_for_tx_to_complete();
}

static bool NRFLite_start_rx(){
    NRFLite_wait_for_tx_to_complete();

    // Put radio into Standby-I mode in order to transition into RX mode.
    digitalWrite(pin_ce, LOW);

    // Configure the radio for receiving.
    NRFLite_write_register(CONFIG, CONFIG_REG_SETTINGS_FOR_RX_MODE);

    // Put radio into RX mode.
    digitalWrite(pin_ce, HIGH);

    // Wait for the transition into RX mode.
    delay(POWERDOWN_TO_RXTX_MODE_MILLIS);

    return (NRFLite_read_register(CONFIG) == CONFIG_REG_SETTINGS_FOR_RX_MODE);
}

static bool NRFLite_init_radio(void){
    last_to_radio_id = -1;
    reset_interrupt_flags = 1;

    delay(OFF_TO_POWERDOWN_MILLIS);

    // Valid channel range is 2400 - 2525 MHz, in 1 MHz increments.
    if (channel > MAX_NRF_CHANNEL) { channel = MAX_NRF_CHANNEL; }
    NRFLite_write_register(RF_CH, channel);

    // Transmission speed, retry times, and output power setup.
    // For 2 Mbps or 1 Mbps operation, a 500 uS retry time is necessary to support the max ACK packet size.
    // For 250 Kbps operation, a 1500 uS retry time is necessary.
    if (bitrate == BITRATE2MBPS)
    {
        NRFLite_write_register(RF_SETUP, 14);   // 2 Mbps, 0 dBm output power
        NRFLite_write_register(SETUP_RETR, 31); // 0001 =  500 uS between retries, 1111 = 15 retries
        transmission_retry_wait_micros = 600;   // 100 more than the retry delay
        max_has_data_interval_micros = 1200;
    }
    else if (bitrate == BITRATE1MBPS)
    {
        NRFLite_write_register(RF_SETUP, 6);   // 1 Mbps, 0 dBm output power
        NRFLite_write_register(SETUP_RETR, 31); // 0001 =  500 uS between retries, 1111 = 15 retries
        transmission_retry_wait_micros = 600;   // 100 more than the retry delay
        max_has_data_interval_micros = 1700;
    }
    else
    {
        NRFLite_write_register(RF_SETUP, 38);   // 250 Kbps, 0 dBm output power
        NRFLite_write_register(SETUP_RETR, 95); // 0101 = 1500 uS between retries, 1111 = 15 retries
        transmission_retry_wait_micros = 1600;  // 100 more than the retry delay
        max_has_data_interval_micros = 5000;
    }

    // Assign this radio's address to RX pipe 1.  When another radio sends us data, this is the address
    // it will use.  We use RX pipe 1 to store our address since the address in RX pipe 0 is reserved
    // for use with auto-acknowledgment packets.
    __data uint8_t address[5] = { ADDRESS_PREFIX[0], ADDRESS_PREFIX[1], ADDRESS_PREFIX[2], ADDRESS_PREFIX[3], radio_id };
    NRFLite_write_register_length(RX_ADDR_P1, &address, 5);

    // Enable dynamically sized packets on the 2 RX pipes we use, 0 and 1.
    // RX pipe address 1 is used to for normal packets from radios that send us data.
    // RX pipe address 0 is used to for auto-acknowledgment packets from radios we transmit to.
    NRFLite_write_register(DYNPD, _BV(DPL_P0) | _BV(DPL_P1));

    // Enable dynamically sized payloads, ACK payloads, and TX support with or without an ACK request.
    NRFLite_write_register(FEATURE, _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK));

    // Ensure RX and TX buffers are empty.  Each buffer can hold 3 packets.
    NRFLite_spi_transfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
    NRFLite_spi_transfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);

    // Clear any interrupts.
    NRFLite_write_register(STATUS_NRF, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    return NRFLite_start_rx();
}

bool NRFLite_init(uint8_t _radio_id, uint8_t _ce_pin, uint8_t _csn_pin, Bitrate _bitrate, uint8_t _channel, bool _begin_spi){
    if(initialized){
        return false;
    }

    radio_id = _radio_id;
    pin_ce = _ce_pin;
    pin_csn = _csn_pin;
    bitrate = _bitrate;
    channel = _channel;

    pinMode(pin_ce, OUTPUT);
    pinMode(pin_csn, OUTPUT);
    digitalWrite(pin_csn, HIGH);

    if(_begin_spi) SPI_begin();

    if(!NRFLite_init_radio()){
        return false;
    }

    initialized = true;
    return true;
}

void NRFLite_set_address_prefix(uint8_t _byte1, uint8_t _byte2, uint8_t _byte3, uint8_t _byte4){
    ADDRESS_PREFIX[0] = _byte1;
    ADDRESS_PREFIX[1] = _byte2;
    ADDRESS_PREFIX[2] = _byte3;
    ADDRESS_PREFIX[3] = _byte4;
}

static uint8_t NRFLite_get_pipe_of_first_rx_packet()
{
    // The pipe number is bits 3, 2, and 1.  So B1110 masks them and we shift right by 1 to get the pipe number.
    // 000-101 = Data Pipe Number
    //     110 = Not Used
    //     111 = RX FIFO Empty
    return (NRFLite_read_register(STATUS_NRF) & 14) >> 1;
}

static uint8_t NRFLite_get_rx_packet_length(){
    // Read the length of the first data packet sitting in the RX buffer.
    __data uint8_t data_length;
    NRFLite_spi_transfer(READ_OPERATION, R_RX_PL_WID, &data_length, 1);

    // Verify the data length is valid (0 - 32 bytes).
    if(data_length > 32){
        NRFLite_spi_transfer(WRITE_OPERATION, FLUSH_RX, NULL, 0); // Clear invalid data in the RX buffer.
        NRFLite_write_register(STATUS_NRF, NRFLite_read_register(STATUS_NRF) | _BV(TX_DS) | _BV(MAX_RT) | _BV(RX_DR));
        return 0;
    }
    
    return data_length;
}

uint8_t NRFLite_has_data(){
    __data uint8_t not_in_rx_mode = NRFLite_read_register(CONFIG) != CONFIG_REG_SETTINGS_FOR_RX_MODE;
    if (not_in_rx_mode) NRFLite_start_rx();

    // If we have a pipe 1 packet sitting at the top of the RX buffer, we have data.
    if (NRFLite_get_pipe_of_first_rx_packet() == 1) return NRFLite_get_rx_packet_length(); // Return the length of the data packet in the RX buffer.
    return 0;
}

void NRFLite_read_data(void *_data){
    // Determine length of data in the RX buffer and read it.
    __data uint8_t data_length;
    NRFLite_spi_transfer(READ_OPERATION, R_RX_PL_WID, &data_length, 1);
    NRFLite_spi_transfer(READ_OPERATION, R_RX_PAYLOAD, _data, data_length);
    
    // Clear data received flag.
    __data uint8_t status_reg = NRFLite_read_register(STATUS_NRF);
    if(status_reg & _BV(RX_DR)) NRFLite_write_register(STATUS_NRF, status_reg | _BV(RX_DR));
}

bool NRFLite_send_data(__data uint8_t _to_radio_id, void *_data, __data uint8_t _length, __data Send_Type _send_type){
    NRFLite_prep_for_tx(_to_radio_id, _send_type);

    // Clear any previously asserted TX success or max retries flags.
    NRFLite_write_register(STATUS_NRF, _BV(TX_DS) | _BV(MAX_RT));
    
    // Add data to the TX buffer, with or without an ACK request.
    if(_send_type == NO_ACK) NRFLite_spi_transfer(WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, _data, _length);
    else NRFLite_spi_transfer(WRITE_OPERATION, W_TX_PAYLOAD, _data, _length);

    return NRFLite_wait_for_tx_to_complete();
}