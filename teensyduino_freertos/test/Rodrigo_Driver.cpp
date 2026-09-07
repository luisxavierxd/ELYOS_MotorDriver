#include <SimpleFOC.h>


// ---------------- MOTOR ----------------
BLDCMotor motor = BLDCMotor(15, 0.124, 70.0);

// ---------------- DRIVER ----------------
BLDCDriver6PWM driver = BLDCDriver6PWM(4, 33, 6, 9, 36, 37);

// ---------------- HALL SENSOR ----------------
HallSensor sensor = HallSensor(10, 12, 21, 15);

void doA(){ sensor.handleA(); }
void doB(){ sensor.handleB(); }
void doC(){ sensor.handleC(); }

// ---------------- CURRENT SENSE ----------------
// ✅ Correct shunt value
InlineCurrentSense current_sense = InlineCurrentSense(0.005, 20, 26, 27, 38);

// ---------------- POTENTIOMETER ----------------
const int potPin = 24;


// ---------------- COMMANDER ----------------
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  // ---------------- POTENTIOMETER INIT ----------------
  pinMode(potPin, INPUT);
  analogReadResolution(12);

  // ---------------- SENSOR INIT ----------------
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);

  // ---------------- DRIVER INIT ----------------
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 30000;

  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  // ---------------- CURRENT SENSE INIT ----------------
  current_sense.linkDriver(&driver);   // ✅ IMPORTANT
  if(!current_sense.init()){
    Serial.println("Current sense init failed!");
    return;
  }
  motor.linkCurrentSense(&current_sense);

  // ---------------- SAFETY LIMITS ----------------
  motor.voltage_limit = 24;   // start conservative
  motor.current_limit = 15;

  // ---------------- CONTROL MODE ----------------
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  // ---------------- STABLE CURRENT PI ----------------
  motor.PID_current_q.P = 3;
  motor.PID_current_q.I = 500; //BEST VALUE YET:P5 I1000 D0.001  BEST NO DIF: 1 500
  motor.PID_current_q.D = 0.0;

  motor.PID_current_d.P = 0.1;
  motor.PID_current_d.I = 100; //BEST VALUE YET:P5 I1000 D0.001   BEST NO DIF: 0.1 50
  motor.PID_current_q.D = 0.0;
  motor.PID_current_d.D = 0.0;

  // ---------------- FILTERING ----------------
  motor.LPF_current_q.Tf = 0.01;
  motor.LPF_current_d.Tf = 0.01;

  // ---------------- MONITORING ----------------
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100;
  motor.monitor_variables = _MON_TARGET | _MON_CURR_Q | _MON_CURR_D;

  // ---------------- MOTOR INIT ----------------
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // ✅ MANUAL ELECTRICAL OFFSET (NO ALIGNMENT SPIKE)
  motor.zero_electric_angle = 5.24;           // 5.24<-- adjust if needed
  motor.sensor_direction = Direction::CW;    // try CCW if wrong

  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  motor.target = 0;

  command.add('M', doMotor, "Motor");

  Serial.println("Motor ready - torque control.");
}

void loop() {

  motor.loopFOC();

  // ---------------- READ POTENTIOMETER ----------------
  int potValue = analogRead(potPin);

  // 0 → 2A
  float targetCurrent = ((potValue / 4095.0f) * 20.0f);

  motor.target = targetCurrent;

  motor.move();
  motor.monitor();
  command.run();
}