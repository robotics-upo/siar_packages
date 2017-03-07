
#include <math.h>
#include <Wire.h>

// Initialize magnetometer
bool HMC5883_init()
{
  int success = 0;
  uint8_t aux_byte;

  Wire.begin();
  delay(10);

  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write((uint8_t)ConfigRegA);
  aux_byte = (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation);
  Wire.write(aux_byte);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write((uint8_t)ModeRegister);
  Wire.write((uint8_t)ContinuousConversion);        // Set continuous mode (default to 10Hz)
  Wire.endTransmission();                 // End transmission
  delay(50);
  
  return(1);
}

// Read Sensor data in chip axis
void HMC5883_read()
{
  int i = 0;
  byte buff[6];

  // Request data
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  // Read data
  Wire.requestFrom(COMPASS_ADDRESS, 6);    // request 6 bytes from device
  while(Wire.available())
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  // All bytes received?
  if (i==6)
  {  
    // MSB byte first, then LSB
    magX = ((((int)buff[0]) << 8) | buff[1]);    // X axis
    magY = ((((int)buff[4]) << 8) | buff[5]);    // Y axis
    magZ = ((((int)buff[2]) << 8) | buff[3]);    // Z axis
  }
}




