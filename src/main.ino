#include <EEPROM.h>
#include <MadgwickAHRS.h>
#include <NXPMotionSense.h>
#include <Servo.h>
#include <SparkFun_Qwiic_Relay.h>
#include <Wire.h>

#define RELAY_ADDR 0x18

Madgwick filter;
NXPMotionSense imu;
Qwiic_Relay relay(RELAY_ADDR);
Servo xservo;
Servo yservo;

float cm = 0; // Centre of mass (m), unused
float range = 10.0; // Gimbal range (deg)
float thrust = 0; // Average thrust (N), unused
float g_threshold = 2; // Launch detection threshold (g)

float xc = 90.0; // Gimbal center
float yc = 90.0;

float xmax = xc + range;
float xmin = xc - range;
float ymax = yc + range;
float ymin = yc - range;

float xpos = xc;
float ypos = yc;

float xtorque = 0.0; // Torque about centre of mass (N⋅m), unused
float ytorque = 0.0;

float Kp = 0.0; // PID values
float Ki = 0.0;
float Kd = 0.0;
float xp = 0.0;
float xi = 0.0;
float xd = 0.0;
float yp = 0.0;
float yi = 0.0;
float yd = 0.0;

int time_now = 0; // Current time
int time_prev = 0; // Previous time
int time_wait = 0; // Safety delay
int time_detect = 0; // Launch detection time
int time_launch = 0; // Launch time
int time_confirm = 0; // Launch detection interval
int time_burn = 0; // Boost phase interval

float xp_prev;
float yp_prev;

bool wait = true; // Safety delay
bool go = true; // Go Vector
bool launch = false; // Launch status
bool detect = false; // Launch detection status
bool gimbal = false; // Gimbal status
bool eject = false; // Parachute ejection status

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  xservo.attach(0);
  yservo.attach(1);
  imu.begin();
  filter.begin(100);
  if(!relay.begin()){
    Serial.print("Inspect relay");
  }
}

void loop()
{
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float phi, the, psi;
  float dt;

  if(imu.available()) {
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az); // Update Madgwick filter
    phi = filter.getRoll();
    the = filter.getPitch();
    psi = filter.getYaw();
    time_prev = time_now;
    time_now = millis();
    dt = float(time_now - time_prev) / 1000.0;
    xp_prev = xp;
    yp_prev = yp;

    // Wait
    if(wait) {
      if(time_now > time_wait) {
        wait = false;
      }
    }

    // Launch detection
    if(!launch && !wait && go) {
      if(az > g_threshold) {
        if(detect) {
          if(time_now - time_detect > time_confirm) {
            gimbal = true;
            launch = true;
            time_launch = time_now;
          }
        } else {
         time_detect = time_now;
         detect = true;
        }
      } else {
        detect = false;
      }
    }

    // Boost phase
    if(launch) {
      xp = 0.0 - phi;
      yp = 0.0 - the;
      xi = xi + (xp * dt);
      yi = yi + (yp * dt);
      xd = (xp - xp_prev) / dt;
      yd = (yp - yp_prev) / dt;
      xpos = xpos + (xp * Kp) + (xi * Ki) + (xd * Kd);
      ypos = ypos + (yp * Kp) + (yi * Ki) + (yd * Kd);
      if (xpos > xmax) {
        xpos = xmax;
      } else if (xpos < xmin) {
        xpos = xmin;
      }
      if (ypos > ymax) {
        ypos = ymax;
      } else if (ypos < ymin) {
        ypos = ymin;
      }
      xservo.write(int(xpos));
      yservo.write(int(ypos));
      if(time_now - time_launch > time_burn) {
        gimbal = false;
        go = false;
        launch = false;
        eject = true;
      }
    }

    // Gimbal disabled
    if(!gimbal) {
      xservo.write(int(xc));
      yservo.write(int(yc));
    }

    // Parachute ejection
    if(eject) {
      relay.turnRelayOn();
      delay(100);
      relay.turnRelayOff();
      eject = false;
    }

  }
}
