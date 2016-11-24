/*
 * TODO:    
*/
/* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */
/*
 *  Uses the imu to gather orientation and motion flow camera to gather location along with the processing demo
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ADNS3080.h>

bool FirstRunBNO = false; // reset droll, dpitch, dyaw, values
bool FirstRunADNS = false; // reset d*_cam values
bool FirstRunL = false; // reset d*_cm, d*_in values

inline double cm2in(double a) {return (a*0.393701);}

double ct = 0.00;
double pt = 0.00;
double dt = 0.00;

double alt_cm = 0.00;
double alt_in = 0.00;
double roll = 0.00, pitch = 0.00, yaw = 0.00;
double droll = 0.00, dpitch = 0.00, dyaw = 0.00;
double proll = 0.00, ppitch = 0.00, pyaw = 0.00;
double x_cm = 0.00, y_cm = 0.00;    // 
double dx_cm = 0.00, dy_cm = 0.00;  // displacement
double px_cm = 0.00, py_cm = 0.00;  // previous values
double ix_cm = 0.00, iy_cm = 0.00;  // integral

double x_in = 0.00, y_in = 0.00;
double ix_in = 0.00, iy_in = 0.00;

// for motion flow cam
int32_t x_cam = 0.00;
int32_t y_cam = 0.00;
double x_cam_comp = 0.00;
double y_cam_comp = 0.00;
int8_t dx_cam = 0.00;
int8_t dy_cam = 0.00;


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*
 *  FULL RUNNING LOOP TO BE CALLED IN VOID LOOP()
 *  Calibration of variables function called in void setup()
 */
void fullRun();
void partRun();

/*
 *  BNO055 IMU functions
 */
void displaySensorDetails(void);
void printOrientation(sensors_event_t &event);

/*
 * Lidar  Functions
 */
 void lidarSetup();
 void getAlt_cm();
 
/*
 *  ADNS3080 object
 */

myADNS3080 moFlow;// lol


void setup()
{
  
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /******************************************************************************
   * THE IMU SENSOR SETUP
   *****************************************************************************/
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  displaySensorDetails();
  /******************************************************************************
   * THE OPTICAL FLOW SENSOR SETUP
   *****************************************************************************/
  moFlow.setup();
  
  /******************************************************************************
   * THE Lidar lite SENSOR SETUP
   *****************************************************************************/
  lidarSetup();

  /*
   * Run the function once to make sure it starts at zero position
   */
   Serial.println("Calibrating initical conditions");
   partRun();
   Serial.println("Initial conditions set");
  
  
}

void loop()
{
  fullRun();
}
void partRun()
{
  sensors_event_t event;
  bno.getEvent(&event);
  pt = ct;
  ct = millis();
  dt = ct-pt;
  proll = roll;
  ppitch = pitch;
  pyaw = yaw;
  roll = event.orientation.z;
  pitch = event.orientation.y;
  yaw = event.orientation.x;
  
  droll = roll - proll;
  dpitch = pitch - ppitch;
  dyaw = yaw - pyaw;

  getAlt_cm();
  alt_in = cm2in(alt_cm);  
  moFlow.updateLocation();
  
  x_cam = moFlow.getY() * alt_cm * moFlow.conv_factor;
  y_cam = moFlow.getX() * alt_cm * moFlow.conv_factor;
  dx_cam = moFlow.getDY() * alt_cm * moFlow.conv_factor;
  dy_cam = moFlow.getDX() * alt_cm * moFlow.conv_factor;
  
  //x_cam_comp = x_cam + ix_cm;
  //x_cam_comp = x_cam + ix_cm;
  //x_cam += dx_cam;

  x_cam = 0.00;
  y_cam = 0.00;
  x_cam_comp = 0.00;
  y_cam_comp = 0.00;
  dx_cam = 0.00;
  dy_cam = 0.00;
  ix_cm = 0.00;
  iy_cm = 0.00;
 
  droll = 0.00;
  dpitch = 0.00;
  dyaw = 0.00;
  
}
void fullRun()
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  pt = ct;
  ct = millis();
  dt = ct-pt;
  proll = roll;
  ppitch = pitch;
  roll = event.orientation.z;
  pitch = event.orientation.y;
  droll = roll - proll;
  dpitch = pitch - ppitch;

  getAlt_cm();
  alt_in = cm2in(alt_cm);  

  moFlow.updateLocation();
  x_cam = (moFlow.getX()*alt_cm)*moFlow.conv_factor;//*moFlow.conv_factor*alt_cm;
  y_cam = (moFlow.getY()*alt_cm)*moFlow.conv_factor;
  dx_cam = (moFlow.getDY()*alt_cm)*moFlow.conv_factor;//*moFlow.conv_factor*alt_cm;
  dy_cam = (moFlow.getDX()*alt_cm)*moFlow.conv_factor;
  //x_cam_comp += (dx_cam + dx_cm);//
  if(abs(dx_cam) >= 0)
  {
    x_cam_comp = x_cam - ix_cm;
  }
  else
  {
    x_cam_comp = x_cam;
  }

  if(abs(dx_cam) >= 0)
  {
    y_cam_comp = y_cam + iy_cm;
  }
  else
  {
    y_cam_comp = y_cam;
  }
  //x_cam += dx_cam;
  
  x_in = cm2in(x_cm);
  y_in = cm2in(y_cm);
  ix_in = cm2in(ix_cm);
  iy_in = cm2in(iy_cm);
  
  
  /*//Serial.printf("X:"); Serial.printf("%06f",x_in);
  Serial.printf("iX: "); Serial.printf("%06f", ix_cm);
  Serial.printf("\t Alt: "); Serial.printf("%4f", alt_cm);
  Serial.printf("\t\t R: "); Serial.printf("%4f", roll);
  Serial.printf("\t\t X_cam: "); Serial.printf("%4d", x_cam);
  Serial.printf("\t\t X_compensate: "); Serial.printf("%4f", x_cam_comp);
  Serial.println();
  //Serial.println(s); */
  Serial.printf("iX: "); Serial.printf("%04f", ix_cm);
  Serial.printf("\t iY: "); Serial.printf("%4f", iy_cm);
  Serial.printf("\t\t Y_cam: "); Serial.printf("%4d", y_cam);
  Serial.printf("\t Y_compensate: "); Serial.printf("%4f", y_cam_comp);
  Serial.printf("\t\t X_cam: "); Serial.printf("%4d", x_cam);
  Serial.printf("\t X_compensate: "); Serial.printf("%4f", x_cam_comp);
  Serial.println();
  //Serial.println(s); 
}

/*
 * Lidar  Functions
 */
 void lidarSetup()
 {
    pinMode(2, OUTPUT); // Set pin 2 as trigger pin
    pinMode(3, INPUT); // Set pin 3 as monitor pin
    digitalWrite(2, LOW); // Set trigger LOW for continuous read
 }

 void getAlt_cm()
 {
    double pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
    if(pulse_width != 0)
    {
      alt_cm = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
      px_cm = x_cm; // not sure if needed
      py_cm = y_cm; // not sure if needed 
      
      y_cm = alt_cm*sin(radians(roll));
      x_cm = alt_cm*sin(radians(pitch));
    
      dy_cm = alt_cm*sin(radians(droll));//x_xm - px_cm;
      dx_cm = alt_cm*sin(radians(dpitch));//y_cm - py_cm;
    
      ix_cm += dx_cm;
      iy_cm += dy_cm;
    }
    
 }

/*
 *  BNO055 IMU functions
 */
void displaySensorDetails(void)
{ 
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void printOrientation(sensors_event_t &event)
{

  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation: "));
  Serial.print((double)event.orientation.x);// yaw
  Serial.print(F(" "));
  Serial.print((double)event.orientation.y);// pitch
  Serial.print(F(" "));
  Serial.print((double)event.orientation.z);// roll
  Serial.println(F(""));
}

