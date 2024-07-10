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