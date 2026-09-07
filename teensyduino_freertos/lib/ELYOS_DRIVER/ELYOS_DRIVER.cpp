#include "ELYOS_DRIVER.h"

static ELYOS_DRIVER* elyos_instance = nullptr;

// Constructor
ELYOS_DRIVER::ELYOS_DRIVER() : motor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING),
                               driver(A_PHASE_HIGH_PIN, A_PHASE_LOW_PIN,
                                      B_PHASE_HIGH_PIN, B_PHASE_LOW_PIN,
                                      C_PHASE_HIGH_PIN, C_PHASE_LOW_PIN),
                               current_sense(SHUNT_VALUE, CURRENT_SENSOR_GAIN, 
                                            CURRENT_SENSE_A_PIN, CURRENT_SENSE_B_PIN, CURRENT_SENSE_C_PIN),
                               commander(Serial),
                               telemetry_vbus_mV(0),
                               telemetry_ibus_mA(0),
                               telemetry_ibus_mA_filtered(0.0f),
                               telemetry_rpm(0),
                               telemetry_iq_mA(0),
                               telemetry_id_mA(0),
                               hall_sensor(nullptr),
                               smooth_sensor(nullptr),
                               throttle(ThrottleFOC::Config{}) {
}

////////// HALL CALLLBACKS //////////
void hall_A_Handle(){
    if(elyos_instance && elyos_instance->hall_sensor)
        elyos_instance->hall_sensor->handleA();
}

void hall_B_Handle(){
    if(elyos_instance && elyos_instance->hall_sensor)
        elyos_instance->hall_sensor->handleB();
}

void hall_C_Handle(){
    if(elyos_instance && elyos_instance->hall_sensor)
        elyos_instance->hall_sensor->handleC();
}

///////// COMMANDER CALLBACK /////////

void onIdPWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Id_P(cmd);
}

void onIdIWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Id_I(cmd);
}

void onIqPWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Iq_P(cmd);
}

void onIqIWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Iq_I(cmd);
}

void onTargetWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_target(cmd);
}

//////////////////////////////////////


int ELYOS_DRIVER::driver_Init(){
    elyos_instance = this;

    // Initialize hardware (objects already created in constructor)
    
    // Sensors 
    hall_sensor = new HallSensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, POLE_PAIRS);

    // Commander, telemetry and Serial initialization
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);
    elyos::getTelemetryUart()->begin(115200);  // must match ESP companion side

    telemetry.begin(elyos::getTelemetryUart());
    telemetry.vbus_mV = &telemetry_vbus_mV;
    telemetry.ibus_mA = &telemetry_ibus_mA;
    telemetry.rpm = &telemetry_rpm;
    telemetry.iq_mA = &telemetry_iq_mA;
    telemetry.id_mA = &telemetry_id_mA;
    telemetry.allFast = &fast_data;

    commander.add('a', onIdPWrapper, "Id P");
    commander.add('b', onIdIWrapper, "Id I");
    commander.add('c', onIqPWrapper, "Iq P");
    commander.add('d', onIqIWrapper, "Iq I");
    commander.add('t', onTargetWrapper, "Target");

    // Throttle 
    elyos::Gpio::setMode(THROTTLE_PIN, elyos::GpioMode::Input);
    elyos::Adc::setResolution(12);

    // Hall sensor 
    hall_sensor->init();
    hall_sensor->enableInterrupts(hall_A_Handle, hall_B_Handle, hall_C_Handle);
    // motor.linkSensor(hall_sensor);

    // Smoothing sensor (optional, can be used for better angle estimation with hall sensors)
    smooth_sensor = new SmoothingSensor(*hall_sensor, motor);   // Smoothing lowers considerably motor vibrations
    motor.linkSensor(smooth_sensor);

    // Driver init
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.pwm_frequency = PWM_FREQUENCY;

    // Check if init is successful
    if(!driver.init()){
        return ELYOS_DRIVER_ERROR;
    }
    motor.linkDriver(&driver);

    // Current sense init 
    current_sense.linkDriver(&driver);
    if(!current_sense.init()){
        return ELYOS_DRIVER_ERROR;
    }
    // Skip current sense align 
    current_sense.skip_align = true;
    motor.linkCurrentSense(&current_sense);

    // Safety 
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = CURRENT_LIMIT;

    // Configure controller gains 
    control_Init();

    if(!motor.init()){
        return ELYOS_DRIVER_ERROR;
    }

    // Monitor data
    motor.useMonitoring(Serial);
    motor.monitor_downsample = 100;
    motor.monitor_variables = _MON_TARGET |_MON_CURR_Q | _MON_CURR_D;
    // motor.monitor_variables = _MON_TARGET;

    ///// TUNE THIS
    motor.zero_electric_angle = ZERO_ALIGN_VALUE;
    motor.sensor_direction = Direction::CW;

    // Init Field Oriented Controller
    if(!motor.initFOC()){
        return ELYOS_DRIVER_ERROR;
    }
    
    // Safe startup
    motor.target = 0.0f;

    // Throttle FOC init
    throttle.begin();

    return ELYOS_DRIVER_OK;
}


int ELYOS_DRIVER::control_Init(){
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::torque;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Establish PID gains direct component 
    motor.PID_current_d.P = ID_KP;
    motor.PID_current_d.I = ID_KI;
    motor.PID_current_d.D = ID_KD;
    motor.PID_current_d.output_ramp = 250;   // Limit the rate of change of Id to avoid big spikes in current during sudden throttle changes, adjust as needed

    // Establish PID gains quadrature component 
    motor.PID_current_q.P = IQ_KP;
    motor.PID_current_q.I = IQ_KI;
    motor.PID_current_q.D = IQ_KD;
    motor.PID_current_q.output_ramp = 150;   // This is in A/s

    // Low pass filter 
    motor.LPF_current_q.Tf = IQ_TF;          // Small Tf = fast response, large Tf = smooth, laggy
    motor.LPF_current_d.Tf = ID_TF;

    return ELYOS_DRIVER_OK;
}

// This is not used yet 
void ELYOS_DRIVER::calculateTelemetry(){
    uint16_t raw_vbus = elyos::Adc::read(VBUS_SENSE_PIN, 12);
    float vbus = (raw_vbus / 4095.0f) * 60.9f;

    telemetry_vbus_mV = (uint16_t)(vbus * 1000.0f);
    telemetry_rpm = (int32_t)(motor.shaft_velocity * 60.0f / (2.0f * PI));

    float Id = motor.current.d;
    float Iq = motor.current.q;
    float I_mag = sqrt(Id*Id + Iq*Iq);

    float Ud = motor.voltage.d;
    float Uq = motor.voltage.q;
    float U_mag = sqrt(Ud*Ud + Uq*Uq);

    float modulation = U_mag / 48.0f; // Assuming 48V supply, adjust if different

    float raw_Idc = 0.75f * modulation * I_mag;
    float raw_ibus_mA = raw_Idc * 1000.0f;

    // One-pole low-pass filter on the DC current for telemetry
    const float alpha = 0.9f; // 0.0 = very slow, 1.0 = no filtering
    telemetry_ibus_mA_filtered += alpha * (raw_ibus_mA - telemetry_ibus_mA_filtered);
    telemetry_ibus_mA = (int32_t)telemetry_ibus_mA_filtered;

    fast_data.vbus_mV      = telemetry_vbus_mV;
    fast_data.ibus_mA      = telemetry_ibus_mA;
    fast_data.rpm          = telemetry_rpm;
    fast_data.throttle_raw = throttle.getRaw();
    fast_data.fault_flags  = 0;
}


void ELYOS_DRIVER::stepFOC(float iq_cmd) {
    motor.loopFOC();
    motor.move(iq_cmd);
}

float ELYOS_DRIVER::updateThrottle() {
    float speed_rad_s = motor.shaftVelocity();
    return throttle.update(speed_rad_s);
}

void ELYOS_DRIVER::processTelemetry() {
    calculateTelemetry();
    telemetry.process();
}

void ELYOS_DRIVER::processCommander() {
    motor.monitor();
    commander.run();
}

void ELYOS_DRIVER::populateLoggerData(BLDC_Logger_Data &log_data) {
    log_data.timestamp = millis();
    log_data.raw_throttle = throttle.getRaw();
    log_data.VBat = telemetry_vbus_mV;
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    log_data.currentA = currents.a;
    log_data.currentB = currents.b;
    log_data.currentC = currents.c;
    log_data.rpm = (telemetry_rpm >= 0) ? static_cast<uint16_t>(telemetry_rpm) : static_cast<uint16_t>(-telemetry_rpm);
}

float ELYOS_DRIVER::getShaftVelocity() {
    return motor.shaftVelocity();
}

void ELYOS_DRIVER::runFOC(){
    float iq_cmd = updateThrottle();
    stepFOC(iq_cmd);

    // Calculate telemetry values
    static uint32_t lastTelemetryTime = 0;
    uint32_t now = millis();
    if (now - lastTelemetryTime >= 100) { // Send telemetry every 100 ms
        processTelemetry();
        lastTelemetryTime = now;   
    }

    processCommander();
}



//////////////////////////////////////////////////////
////////////////////// COMMANDS //////////////////////
//////////////////////////////////////////////////////

/////// SET PID GAINS Id
void ELYOS_DRIVER::cmd_set_Id_P(char* cmd){
    float val = atof(cmd);
    motor.PID_current_d.P = val;
    Serial.print("Id P set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Id_I(char* cmd){
    float val = atof(cmd);
    motor.PID_current_d.I = val;
    Serial.print("Id I set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Id_D(char* cmd){
    float val = atof(cmd);
    motor.PID_current_d.D = val;
    Serial.print("Id D set to ");
    Serial.println(val);
}

/////// SET PID GAINS Iq
void ELYOS_DRIVER::cmd_set_Iq_P(char* cmd){
    float val = atof(cmd);
    motor.PID_current_q.P = val;
    Serial.print("Iq P set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Iq_I(char* cmd){
    float val = atof(cmd);
    motor.PID_current_q.I = val;
    Serial.print("Iq I set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Iq_D(char* cmd){
    float val = atof(cmd);
    motor.PID_current_q.D = val;
    Serial.print("Iq D set to ");
    Serial.println(val);
}

/////// SET MOTOR TARGET
void ELYOS_DRIVER::cmd_set_target(char* cmd){
    float val = atof(cmd);
    motor.target = val;
    Serial.print("Target set to ");
    Serial.println(val);
}