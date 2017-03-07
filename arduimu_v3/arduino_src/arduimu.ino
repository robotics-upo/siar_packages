#include "MPU6000.h"
#include "HMC5883.h"

int data[11];
int count;

void setup()
{ 
  // Setup serial port
  Serial.begin(115200, SERIAL_8N1);

  // MPU6000 initialization
  MPU6000_Init();       
  
  // HMC5883 initialization
  HMC5883_init();
  count = 2;
}

//***************************************************************************************
void loop() //Main Loop
{
  // Send data if new are available
  if(newData)  
  {
    // Crate a buffer to send all the information at once
    *(unsigned long *)data = timeStamp;
    data[2] = accelX;
    data[3] = accelY;
    data[4] = accelZ;
    data[5] = gyroX;
    data[6] = gyroY;
    data[7] = gyroZ;
    
    // Set newData flag to false
    newData = 0;
        
    // Read magnetometer at 66Hz (configured at 75Hz)
    if(count++ == 2)
    {
      // Reset cicle count
      count = 0;
      
      // Read magnetometer
      HMC5883_read();
      
      // Saves data in output buffer
      data[8] = magX;
      data[9] = magY;
      data[10] = magZ;
    }
    
    // Send data
    Serial.write(0x23);
    if(count == 0)
      Serial.write((byte *)data, 22);
    else
      Serial.write((byte *)data, 16);
    Serial.write(0x2A);
  }
}


