//--------------------------------------------------------------------------
// SYSTEM INITIALIZATION AND CONTROL PARAMETERS
//
// This section defines all variables related to the digital control system,
// including controller gains, sampling time, signal scaling, filtering,
// and experimental setup.
//
// The system operates with a voltage-based control signal (0–5 V),
// mapped to a PWM output. The pressure measurement is acquired in volts
// and related to physical pressure units (bar) through calibration.
//
// Controller:
//   - Incremental PI controller (see PID() function)
//   - Kp: proportional gain
//   - Ti: integral time
//   - dt: sampling time
//
// NOTE:
// These parameters were defined based on experimental tuning and may be
// adjusted depending on the system configuration and operating conditions.
const int PWM_PIN = 9;
float Kp = 11.25; //22 //692.30;   // Proportional gain 
float Ti = 2.13; //3.33;     // Integrative time (s)
float dt = 0.062;     	// sample time(s)
const long interval = dt*1000; //sample time(ms) minimo de 1000 para um intervalo de 1 segundo
float p=0; 		//Control signal
float p_old = 0; 	// Previous Control signal
float error=0; 		//Current error 
float error_old=0;	// Previous error
float Measure = 0; 	//  Pressure Measure(V)
float delta_p= 0; 	// Signal control Delta 
float SP1 = 0.042; // Set point pressure [bar]
float SP1_volt = (SP1*5)/0.344; //Setpoint Pressure [volts]
float SP1_voltDEG = 0.71;
float Measure_old = 0; 	//  Pressure Measure(V)
float delta_Measure = 0.3; // epsilon==tolerance
unsigned long previousMillis = 0; //Previous Measure Time
unsigned long currentMillis = 0;
float Measure_1 = 0; float Measure_2 =0; float Measure_3 =0; float Measure_4 =0; float Measure_5 =0; float Measure_6 = 0; float Measure_7 =0; float Measure_8 =0; float Measure_9 = 0; float Measure_10 =0;
const long controlDelayTime = 10000; // Control starts after this time 
float pwm_min = 0;
float pwmOutput = 0;//map(p, 0, 5, pwm_min, 255);
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------
//SETUP
void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  }
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------
// DIGITAL PID CONTROL LAW (INCREMENTAL FORM)
//
// This function implements a discrete-time PI controller using the
// incremental formulation:
//
//   Δp_k = Kp * (e_k - e_{k-1}) + (Kp * dt / Ti) * e_k
//   p_k  = p_{k-1} + Δp_k
//
// where:
//   e_k  -> current error (setpoint - measured value)
//   p_k  -> control signal (output voltage)
//
// NOTE (IMPORTANT FOR EXPERIMENTAL PROCEDURE):
// To perform open-loop step tests for controller tuning, the PID
// action can be disabled by:
//   (1) commenting the control update line:   p = p_old + delta_p;
//   (2) uncommenting the line:               p = 5;
//
// This forces a manual step input to the system, allowing data
// acquisition for parameter identification using the workflow
// "Optimazed algorithm".
void PID() {
  error = SP1_voltDEG - Measure; // Control error (V)
  if (millis() < controlDelayTime) {
    p = 0.0;  // Output forced to 0 V during startup delay
  } 
  else {
    delta_p = Kp * (error - error_old) + (Kp * dt / Ti) * error;
    p = p_old + delta_p;
    // p = 5; // Uncomment to apply a manual step input (open-loop test)
  }
  p = constrain(p, 0, 5); // Saturation (actuator limits)
  p_old = p;      // Update previous control signal
  error_old = error; // Update previous error
}
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------
// MAIN LOOP: EXPERIMENTAL VALIDATION OF THE TUNED CONTROLLER
//
// This loop was used to experimentally validate the PI parameters obtained
// from the tuning script. After the controller gains were determined from
// the system identification and tuning procedure, they were inserted into
// the digital control law implemented in the PID() function.
//
// To assess whether the real system reproduced the behavior predicted by
// simulation, a sequence of setpoint changes was applied during runtime.
// These step variations were used to evaluate the closed-loop response in
// terms of tracking performance, transient behavior, and consistency with
// the simulated results.
//
// At each sampling instant, the measured pressure signal is acquired,
// filtered using a moving average, and then supplied to the control law.
// The resulting control signal is converted to PWM and sent to the actuator.
//
// Setpoint schedule used for validation:
//   - 0 s   to 30 s  -> 1.00 × 0.69 V
//   - 30 s  to 45 s  -> 0.50 × 0.69 V
//   - 45 s  to 60 s  -> 0.25 × 0.69 V
//   - 60 s onward    -> 0.80 × 0.69 V
//
// NOTE:
// The serial output logs time, measured signal, setpoint, and control signal,
// allowing direct comparison between experimental and simulated responses.
void loop() {
   currentMillis = millis();
  // Setpoint sequence applied for experimental validation
  if (currentMillis >= 60000) {
    SP1_voltDEG = 0.8 * 0.69;
  } else if (currentMillis >= 45000) {
    SP1_voltDEG = 0.25 * 0.69;
  } else if (currentMillis >= 30000) {
    SP1_voltDEG = 0.5 * 0.69;
  } else if (currentMillis >= 0) {
    SP1_voltDEG = 1.0 * 0.69;
  }
  if (currentMillis - previousMillis >= interval) {
    Measure = (analogRead(A5) * 5.0) / 1023.0; // Sensor output (0–5 V)
    // Moving average filter
    Measure_1 = Measure_2;
    Measure_2 = Measure_3;
    Measure_3 = Measure_4;
    Measure_4 = Measure_5;
    Measure_5 = Measure_6;
    Measure_6 = Measure_7;
    Measure_7 = Measure_8;
    Measure_8 = Measure_9;
    Measure_9 = Measure_10;
    Measure_10 = Measure;
    Measure = (Measure_1 + Measure_2 + Measure_3 + Measure_4 + Measure_5 +
               Measure_6 + Measure_7 + Measure_8 + Measure_9 + Measure_10) / 10;
    PID();
    previousMillis = currentMillis;
    pwmOutput = (p * 255) / 5;
    pwmOutput = constrain(pwmOutput, 0, 255);
    analogWrite(PWM_PIN, pwmOutput);
    // Serial output for data logging and post-processing
    Serial.print("T: ");
    Serial.print(currentMillis);
    Serial.print(", Measure(v): ");
    Serial.print(Measure);
    Serial.print(", SP: ");
    Serial.print(SP1_voltDEG);
    Serial.print(", sinalcontrole: ");
    Serial.println(p);
  }
}
//--------------------------------------------------------------------------