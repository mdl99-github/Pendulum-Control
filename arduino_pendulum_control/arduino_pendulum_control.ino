#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <arduino.h>
#include <avr/io.h>
#define MAX_SERVO 3750
#define MIN_SERVO 1300
#define PIN_SERVO 9
#define PIN_POTE A0


Adafruit_MPU6050 mpu;

float angulo_x = 0;
float t_inicio = millis();
float t = t_inicio;
float phi = 0;
float T = 0.01;
int flag = 1;
float phi_entrada;
float a = 17.75;
float b = 1.145;
float c = 75.57;
float k1 = 120.1;
float k2 = -0.6455;

float phi_est = 0;
float phi_est_ant = 0;
float phi_p_est = 0;
float phi_p_est_ant = 0;
float theta_est = 0;
float theta_est_ant = 0;
float theta_p_est = 0;
float theta_p_est_ant = 0;

//Observer
float l11=0.04752;
float l12=0.54232;
float l21=1.0697;
float l22=-3.725;
float l31=0.75873;
float l32=0.065638;
float l41=0.44609;
float l42=3.1707;

//K
float c1=0.76344;
float c2=-0.091154;
float c3=0.4778;
float c4=-0.06582;

//F
float f1 = 0;
float f2 = 0.2366;

//Extended K (integral action)
float K_int[] = {0.76426,-0.091103,0.47815,-0.06574};
float H =0.0030758;

float phi_ref = -20;
float theta_ref = 0;

float q_ant = 0;
float q = 0;

void setup_pwm(void) {
  pinMode(PIN_SERVO, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  ICR1 = 39999;
}

void setup_imu(int acc_range, int gyro_range, int bw) {
  	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(acc_range);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(gyro_range);

	// set filter bandwidth to 5-10-21-44-94-184-260 Hz
	mpu.setFilterBandwidth(bw);

	delay(100);
}

void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
  b = (byte *) &dato4;
  Serial.write(b,4);
  b = (byte *) &dato5;
  Serial.write(b,4);
  b = (byte *) &dato6;
  Serial.write(b,4);
  b = (byte *) &dato7;
  Serial.write(b,4);
  b = (byte *) &dato8;
  Serial.write(b,4);
}


void mover_servo(float phi){
  if(phi <= 60 && phi >= -60){
    unsigned int servo = map(phi, -57, 63, MIN_SERVO, MAX_SERVO);
    OCR1A = servo;
    return;
  }
  if(phi > 60){
    unsigned int servo = map(63, -57, 63, MIN_SERVO, MAX_SERVO);
    OCR1A = servo;
    return;
  }
  if(phi < -60){
    unsigned int servo = map(-57, -57, 63, MIN_SERVO, MAX_SERVO);
    OCR1A = servo;
    return;
  }
}

void setup() {
  Serial.begin(115200*2);
  setup_imu(MPU6050_RANGE_8_G, MPU6050_RANGE_500_DEG, MPU6050_BAND_44_HZ);
  setup_pwm();
}

void loop() {
  unsigned long t_prev = millis();

  sensors_event_t ac, g, temp;
  mpu.getEvent(&ac, &g, &temp);

  float angulo_x_g = (angulo_x + g.gyro.x*0.01*180/PI);
  float angulo_x_a = (atan2(ac.acceleration.y, ac.acceleration.z))*180/PI;
  angulo_x = 0.1*angulo_x_a + 0.9*angulo_x_g;

  int v_pote = analogRead(PIN_POTE);
  float phi = -(map(v_pote, 0, 1023, 300, 0) - 144);
  

  //Periodic sequence of pulses
  if(millis() - t >= 8000 && flag == 1){
    phi_ref = 0;
    flag = 2;
    t = millis();
  }

  if(millis() - t >= 8000 && flag == 2){
    phi_ref = 20;
    flag = 3;
    t = millis();
  }

  if(millis() - t >= 8000 && flag == 3){
    phi_ref = 45;
    flag = 4;
    t = millis();
  }

  if(millis() - t >= 8000 && flag == 4){
    phi_ref = 10;
    flag = 1;
    t = millis();
  }

  float error_theta = angulo_x - theta_est_ant;
  float error_phi = phi - phi_est_ant;

  //To measure observer without controlling
  //phi_entrada = phi_ref;
  
  phi_est = phi_est_ant + phi_p_est_ant*T + l11*error_theta + l12*error_phi;
  phi_p_est = -k1*T*phi_est_ant + phi_p_est_ant*(-a*T+1) + l21*error_theta + l22*error_phi + k1*T*phi_entrada;
  theta_est = theta_est_ant + T*theta_p_est_ant + l31*error_theta + l32*error_phi;
  theta_p_est = -k1*k2*T*phi_est_ant - a*k2*T*phi_p_est_ant - c*T*theta_est_ant + (-b*T+1)*theta_p_est_ant + l41*error_theta + l42*error_phi + k1*k2*T*phi_entrada;

  //State-space feedback control
  //phi_entrada = c1*phi_est + c2*phi_p_est + c3*theta_est + c4*theta_p_est;

  //State-space feedback control with feedforward
  //phi_entrada = f1*theta_ref f2*phi_ref +  c1*phi_est + c2*phi_p_est + c3*theta_est + c4*theta_p_est;

  //State-space feedback control with integral action
  q = q_ant + phi_ref-phi;
  phi_entrada = K_int[0]*phi_est + K_int[1]*phi_p_est + K_int[2]*theta_est + K_int[3]*theta_p_est + H*q;
 
  mover_servo(phi_entrada);
  
  matlab_send(angulo_x, theta_est, phi, phi_est, g.gyro.x*180/PI, theta_p_est, phi_p_est, phi_ref);

  phi_est_ant = phi_est;
  phi_p_est_ant = phi_p_est;
  theta_est_ant = theta_est;
  theta_p_est_ant = theta_p_est;
  q_ant = q;

  delay(10 + t_prev - millis());

}
