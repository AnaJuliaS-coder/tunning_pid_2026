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

float Measure_old = 0; 	//  Pressure Measure(V) EWMA
float alpha = 0.5;// EWMA filter Weight: Exponential Smoothing Weighted Moving Average

float flag1=0; //Noise-spike filter flag
float delta_Measure = 0.3; // epsilon==tolerance

unsigned long previousMillis = 0; //Previous Measure Time
unsigned long currentMillis = 0;

float pwmOutput_1 = 0;
float pwmOutput_2 =0;
float pwmOutput_3 =0;
float pwmOutput_4 =0;
float pwmOutput_5 =0;
float pwmOutput_6 = 0;
float pwmOutput_7 =0;
float pwmOutput_8 =0;
float pwmOutput_9 = 0;
float pwmOutput_10 =0;

float Measure_1 = 0;
float Measure_2 =0;
float Measure_3 =0;
float Measure_4 =0;
float Measure_5 =0;
float Measure_6 = 0;
float Measure_7 =0;
float Measure_8 =0;
float Measure_9 = 0;
float Measure_10 =0;

const long controlDelayTime = 10000; // Control starts after this time 
float pwm_min = 0;
float pwmOutput = 0;//map(p, 0, 5, pwm_min, 255);

float valor_recebido = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  }

void PID() {
  error = SP1_voltDEG - Measure; // Error(V)

    if (millis() < controlDelayTime) {
      p = 0.0;  // 0V antes dos 10s
    } 
    else {
      delta_p = Kp * (error - error_old) + (Kp * dt / Ti) * error;
      p = p_old + delta_p;
      //p = 3.5;
    }
      
  // Saturação
  p = constrain(p, 0, 5);	//Saturation
  p_old = p; 			//update p
  error_old = error;		//update error
}

void loop() {

  // --- Parte 1: Recebimento de Dados do Python ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("VALOR:")) {
      valor_recebido = input.substring(6).toFloat();
      
      // Exemplo: Atualiza o SetPoint (SP1) com o valor recebido
      SP1 = valor_recebido;  // [MODIFIQUE CONFORME SUA LÓGICA]
      SP1_volt = (SP1 * 5) / 0.344;  // Recalcula o SP em volts
      
      // Opcional: envia confirmação
      Serial.print("RECEBIDO:");
      Serial.println(valor_recebido, 2);
    }
  }

   currentMillis = millis();
  //unsigned long elapsedTime = currentMillis - controlDelayTime;  // Tempo decorrido
        // Lógica do setpoint com degraus
  if (currentMillis >= 60000) {
    SP1_voltDEG = 0.8 * 0.69;  // y.iloc[-1] representa o último valor de y
  } else if (currentMillis >= 45000) {
    SP1_voltDEG = 0.25 * 0.69;
  } else if (currentMillis >= 30000) {
    SP1_voltDEG = 0.5 * 0.69;
  } else if (currentMillis >= 0) {
    SP1_voltDEG = 1.0 * 0.69;
  }

  if (currentMillis - previousMillis >= interval) {

    Measure = (analogRead(A5)*5.0 )/ 1023.0; // sensor output (0-5V)
  
    //MOVING AVERAGE FILTER
    Measure_1 = Measure_2;
    Measure_2 = Measure_3;
    Measure_3 =Measure_4;
    Measure_4 = Measure_5;
    Measure_5 = Measure_6;
    Measure_6 = Measure_7;
    Measure_7 =Measure_8;
    Measure_8 = Measure_9;
    Measure_9 = Measure_10;
    Measure_10 = Measure;
    Measure = (Measure_1+Measure_2+Measure_3+Measure_4+Measure_5+Measure_6+Measure_7+Measure_8+Measure_9+Measure_10)/10;

    //EXPONENTIAL FILTER
    //Measure=alpha*Measure+(1-alpha)*Measure_old
    //Measure_old=Measure

    //NOISE-SPIKE FILTER
    //if (flag1==0){
    //	Measure_0ld=Measure;
    //	flag1=1;
    //	}
    //if (Measure-Measure_old > delta_Measure){
    //	Measure=Measure_old+delta_Measure;
    //	}
    //if (Measure_old-Measure > delta_Measure){
    //	Measure=Measure_old-delta_Measure;
    //	}
    //Measure_old=Measure

    PID();
    previousMillis = currentMillis;

    //  Aplica degrau após 50 segundos de forma manual
    //  if (elapsedTime < stepTime) {
    //    uLV = 0.0;  // 0V antes dos 50s
    //  } else {
    // Verifica se é o momento do pulso
    //     if (pulseStartTime == 0) {
      //   pulseStartTime = currentMillis; // Marca início do pulso
      // }
      
      // // Mantém 5V durante pulseDuration ms
      // if (currentMillis - pulseStartTime <= pulseDuration) {
      //   uLV = 5; // PWM máximo (255)
      // } else {
      //   uLV = 5; // Volta para 2V após pulso
      // }
    //}

    //pwm_min = 0;
    pwmOutput = (p*255)/5;//map(p, 0, 5, pwm_min, 255);

    // pwmOutput_1 = pwmOutput_2;
    // pwmOutput_2 = pwmOutput_3;
    // pwmOutput_3 =pwmOutput_4;
    // pwmOutput_4 = pwmOutput_5;
    // pwmOutput_5 = pwmOutput_6;
    // pwmOutput_6 = pwmOutput_7;
    // pwmOutput_7 =pwmOutput_8;
    // pwmOutput_8 = pwmOutput_9;
    // pwmOutput_9 = pwmOutput_10;
    // pwmOutput_10 = pwmOutput;
    // pwmOutput = (pwmOutput_1+pwmOutput_2+pwmOutput_3+pwmOutput_4+pwmOutput_5+pwmOutput_6+pwmOutput_7+pwmOutput_8+pwmOutput_9+pwmOutput_10)/10;

    pwmOutput = constrain(pwmOutput,0,255);
    analogWrite(PWM_PIN, pwmOutput);
 
    // Exibe informações para debug
    Serial.print("T: ");
    Serial.print(currentMillis);

    Serial.print(", Measure(v): ");
    Serial.print(Measure);

    // Serial.print(", ContSig(PWM): ");
    // Serial.print(pwmOutput);

    // Serial.print(", T(ms):  ");
    // Serial.print(millis() );

    // Serial.print(", Error: ");
    // Serial.print(error);

     Serial.print(", SP: ");
     Serial.print(SP1_voltDEG );

    Serial.print(", sinalcontrole: ");
    Serial.println(p);
    
  }
  //delay(1000);
}