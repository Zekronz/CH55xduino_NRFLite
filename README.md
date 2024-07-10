# CH55xduino_NRFLite
[Check out the original project here.](https://github.com/dparson55/NRFLite)  
  
NRFLite port for CH55x microcontrollers using CH55xduino.  
This is not intended to be a full port, but rather one that suits my needs. I'll consider porting the entire library if people want.
  
## Examples  
### Receive  
```c
#include "src/CH55xduino_NRFLite.h"

//Define pins.
#define CE 32
#define CSN 14

//Buffer to store received data. Max 32 bytes.
static __xdata uint8_t buffer[32];

void setup(){
    //Sets the address prefix.
    //The receiver and transmitter must use the same prefix. By default it's set to ( 1, 2, 3, 4 ).
    //Call this before NRFLite_init.
    NRFLite_set_address_prefix('T', 'E', 'S', 'T');

    //Initializes receiver with radio id 0.
    //Bitrate = 2Mbps.
    //Channel = 100.
    if(!NRFLite_init(0, CE, CSN, BITRATE2MBPS, 100, true)){
        for(;;); //Loop forever if failed to initialize.
    }

}

void loop(){
    uint8_t len; //Stores data length.

    //Keep looping until we've read all available data.
    while((len = NRFLite_has_data()) > 0){

        //Reads packet and stores the data in the buffer.
        NRFLite_read_data(buffer);

        //Process data...
    }
}
```
  
### Transmit  
```c
#include "src/CH55xduino_NRFLite.h"

//Define pins.
#define CE 32
#define CSN 14

//Stores data to send. Max 32 bytes.
static __xdata uint8_t buffer[5] = { 'H', 'e', 'l', 'l', 'o' };

void setup(){
    //Sets the address prefix.
    //The transmitter and receiver must use the same prefix. By default it's set to ( 1, 2, 3, 4 ).
    //Call this before NRFLite_init.
    NRFLite_set_address_prefix('T', 'E', 'S', 'T');

    //Initializes transmitter with radio id 1.
    //Bitrate = 2Mbps.
    //Channel = 100.
    if(!NRFLite_init(1, CE, CSN, BITRATE2MBPS, 100, true)){
        for(;;); //Loop forever if failed to initialize.
    }

}

void loop(){
    //Attempts to send data to receiver with radio id 0.
    //Waits for acknowledgement from receiver. Will retry up to 15 times.
    //Use NO_ACK if you don't care about the data being received or not.
    if(!NRFLite_send_data(0, buffer, sizeof(buffer), REQUIRE_ACK)){
        //Failed to send data.
    }

    //Wait a second before sending the packet again.
    delay(1000);
}
```
  
## Functions  
```c
bool NRFLite_init(uint8_t _radio_id, uint8_t _ce_pin, uint8_t _csn_pin, Bitrate _bitrate, uint8_t _channel, bool _begin_spi);
void NRFLite_set_address_prefix(uint8_t _byte1, uint8_t _byte2, uint8_t _byte3, uint8_t _byte4);
uint8_t NRFLite_has_data();
void NRFLite_read_data(void *_data);
bool NRFLite_send_data(uint8_t _to_radio_id, void *_data, uint8_t _length, Send_Type _send_type);
```
