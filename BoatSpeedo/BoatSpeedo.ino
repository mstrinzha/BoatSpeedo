#include <TM74HC595Display.h>

#include "../mavlink/include/mavlink.h"

int SCLK = 7;
int RCLK = 6;
int DIO = 5;

bool newData = false;

TM74HC595Display disp(SCLK, RCLK, DIO);
unsigned char LED_0F[29];
int speed = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  disp.dot(1);
  disp.digit4(1111);
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;
  mavlink_gps_raw_int_t gps;
  
  while(Serial.available() > 0 ) 
  {
    uint8_t c = Serial.read();
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    {
        switch(msg.msgid)
        {
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_msg_gps_raw_int_decode(&msg, &gps);
            newData = true;   
        }
        default:
          break;
        }
      }
  }
  if(newData) {
    newData = false;
    speed = gps.vel*3600./100000;
  }
  disp.digit4(speed);
}
