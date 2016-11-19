// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"


#define MPU9250_ADDRESS 0x68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

uint8_t Gscale = 0;
uint8_t Ascale = 0;
uint8_t Mscale = 1; // 16-bit magnetometer resolution

float mRes = 10.*4800.0/32767.0; // Proper scale to return milliGauss

MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
int16_t magCount[3], accelCount[3];

float ax_base, ay_base, az_base;
float gx_base, gy_base, gz_base;
float gx_angle, gy_angle, gz_angle;

float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};
bool newMagData = false;
float mRes = 10.*4912./32760.0;
float aRes = 2.0/32768.0;
float heading = 0.0;

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};
float a12, a22, a31, a32, a33;
float pitch, yaw, roll;
float aRoll, aPitch;
float RADIANS_TO_DEGREES = 180/3.14159;
float dt, last_time;
float alpha = 0.96;
float last_x_angle, last_y_angle, last_z_angle;
float magx, magy;

float GyroMeasError = PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

uint32_t Now = 0, lastUpdate = 0;
float deltat = 0.0f;

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(38400);

    Serial.println("MPU9250 9-axis motion sensor...");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);


    if (c == 0x71) {
        Serial.println("MPU9250 is online...");

        initMPU9250();
        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      
    }
    // initialize device
    Serial.println("Initializing I2C devices...");
    //accelGyroMag.initialize();
    //Initialize using KrisWinner method instead of this one
    accelGyroMag.initMPU9250();
    accelGyroMag.initAK8963(magCalibration);

    //Calibrate accelerometer and gyroscope
    /*
    accelGyroCalMPU9250();
    */
    
    //Calibrate magnetometer
    /*
    accelGyroMag.magcalMPU9250(magBias, magScale, magCalibration);
    delay(5000);// add delay to see results before serial spew of data
    */
    
    //mag biases: 21.71, 33.16, -21.51
    //mag scale: 1.11, 0.97, 0.93 
    
    //mag biases: 27.63, 39.01, -28.68
    //mag scale: 1.01, 0.99, 0.99

    //mag biases: 21.71, 29.26, -26.88
    //mag scale: 1.08, 0.99, 0.94

    magBias[0] = 23.68;
    magBias[1] = 33.81;
    magBias[2] = -25.69;
    magScale[0] = 1.06;
    magScale[1] = 0.98;
    magScale[2] = 0.95;

    ax_base = -184.55;
    ay_base = 207.77;
    //az_base = 15749.66; //az must not be 0, so offset value is not applied
    gx_base = -279.67;
    gy_base = 139.34;
    gz_base = -65.23;
    
    // verify connection
    /*
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
    */

    last_time = millis();
    last_x_angle = 0;
    last_y_angle = 0;
    last_z_angle = 0;
}

void loop() {
    // read raw accel/gyro/mag measurements from device
//    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);
    uint8_t b = (accelGyroMag.readByte(0x68, 0x3A) & 0x01);
    float ax2, ay2, az2, gx2, gy2, gz2;
    if (b) {
        accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        accelGyroMag.readMagData(magCount);

        ax2 = (float)ax/16384;
        ay2 = (float)ay/16384;
        az2 = (float)az/16384;

        gx2 = (float)(gx-gx_base)/131;
        gy2 = (float)(gy-gy_base)/131;
        gz2 = (float)(gz-gz_base)/131;

        float res = 4800.0/32767.0;
        float res2 = 10.*4800/32767.0;

        /*Serial.print("mx = "); Serial.print(magCount[0]); 
        Serial.print(" my = "); Serial.print(magCount[1]); 
        Serial.print(" mz = "); Serial.print(magCount[2]); Serial.println(" raw");
        Serial.print("mx = "); Serial.print(magCount[0]*res); 
        Serial.print(" my = "); Serial.print(magCount[1]*res); 
        Serial.print(" mz = "); Serial.print(magCount[2]*res); Serial.println(" uT");
        Serial.print("mx = "); Serial.print(magCount[0]*res2); 
        Serial.print(" my = "); Serial.print(magCount[1]*res2); 
        Serial.print(" mz = "); Serial.print(magCount[2]*res2); Serial.println(" mG");*/
        Serial.print(magCount[0]*res2); Serial.print(",");
        Serial.print(magCount[1]*res2); Serial.print(",");
        Serial.println(magCount[2]*res2);
        
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];

        Now = micros();
        deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        //MadgwickAHRSupdate(gx2, gy2, gz2, ax2, ay2, az2, mx, my, mz);
        MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
        
        /*
        complementaryFilter(ax2, ay2, az2, gx2, gy2, gz2, mx, my, mz);
        */
    
        a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(a32);
        roll  = atan2f(a31, a33);
        yaw   = atan2f(a12, a22);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / PI;

        //Serial.println("q[0]\tq[1]\tq[2]\tq[3]");
        //Serial.print(q[0]); Serial.print("\t"); Serial.print(q[1]);
        //Serial.print("\t"); Serial.print(q[2]); Serial.print("\t"); Serial.println(q[3]);
        //Serial.println("Simple\tMadgwick");
        //Serial.print(simpleHeading()); Serial.print("\t"); Serial.println(yaw);
    }

    /*
    Serial.print("ax = "); Serial.print(ax); 
    Serial.print(" ay = "); Serial.print(ay); 
    Serial.print(" az = "); Serial.print(az); Serial.println(" raw");
    Serial.print("ax2 = "); Serial.print(ax2); 
    Serial.print(" ay2 = "); Serial.print(ay2); 
    Serial.print(" az2 = "); Serial.print(az2); Serial.println(" g");
    Serial.print("gx = "); Serial.print(gx); 
    Serial.print(" gy = "); Serial.print(gy); 
    Serial.print(" gz = "); Serial.print(gz); Serial.println(" raw");
    Serial.print("gx2 = "); Serial.print(gx2); 
    Serial.print(" gy2 = "); Serial.print(gy2); 
    Serial.print(" gz2 = "); Serial.print(gz2); Serial.println(" grad/s");
    Serial.print("mx = "); Serial.print(mx); 
    Serial.print(" my = "); Serial.print(my); 
    Serial.print(" mz = "); Serial.print(mz); Serial.println(" mG");
    */


    //Heading calculation, works but very imprecise.
    /*
    heading = simpleHeading();
    */

    //Delay not necessary, as in readMagData it is checked if the data is available
    delay(50);
}


void initMPU9250() {
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    delay(100); // Wait for all registers to reset 
    
    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200); 
    
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  
    
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                  // determined inset in CONFIG above
    
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
    
    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer 
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
    
    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
    
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
    
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    //   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}


uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;                            // `data` will store the register data   
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
//  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
//    Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }         // Put read results in the Rx buffer
}
