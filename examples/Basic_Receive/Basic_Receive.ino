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