
volatile byte quadratureCode, oldquadratureCode;
volatile float quadPos = 0.0;
volatile bool isEncoderChange = false;
float lastquadPos = 0;

bool isModuleEn = false, prevModuleEn = false, isPrinting = false;
bool isProfileEn = false, oldProfileEn = false, isShowStats = false;

const byte  TSAMP_MSEC = 30;
int TIC_MSEC = TSAMP_MSEC;
long timeElapsedTicks = 0;
const float TSAMP = 0.001 * TSAMP_MSEC;
volatile float adcReading = 0;

const int ENC2 = A2;  // d16, PC2 PCINT10 (green wire)
const int ENC3 = A3;  // d17, PC3 PCINT11 (blue wire
const int PWM_M1A_PIN = 3;   // d3,  PD3 PCINT19 digital output OCR2B (green wire)
const int PWM_M1B_PIN = 11;   // d11, PB4 digital output OCR2A (blue wire)

const float VOLTS_PER_PWM = 5. / 255.0; // motor drive voltage per pwm command
const float PWM_PER_VOLT = 1 / VOLTS_PER_PWM; // pwm value to achieve 1 volt output

float Vctl, Varm;
float mtrVel = 0.0, mtrPos = 0.0, errVel = 0.0, errPos = 0.0;
float refAcc = 0.0, refVel = 0.0, refPos = 0.0;
float refDist = 0.0, errDist = 0.0;
int objectDistmm = 0;
unsigned int tick = 0;

float dir = 1.0;
float disp_rad = 0;
float specAcc = 0.0, specVel, specDisp_rad, specVdist;
float trapAcc = 0.0, trapVel = 0.0, trapDisp_rad = 0.0;
int   dwellStartT, accT, platT, dwellEndT;
int   T0, T1, T2, T3, T4, T5, T6, T7, T8;
int   D0, D1, D2, D3, D4, D5, D6, D7, D8;
float dD1, dD2, dD3, dD4, dD5, dD6, dD7, dD8;

int GEAR_RATIO = 120;//Gear ratio of the motor

boolean isTrapezoid = false;
boolean isStepRef = false;
boolean isBallBalance = false;

//---------------------------------------------------------------------
// Motor constants: DF robot motor with encoder model FIT0458
const float K_EMF = 0.00285; //(V/(160*120))*60.0/TWO_PI; // V/rad/sec from V/1000RPM
const float K_TRQ = 0.008 / 2.8; // N-m/Amp
const float R_ARM = 2.14; // ohms
const float D_DRAG = 0 * 1.117e-6;
const float SYS_A = 1795;//1795; //2040//1717
const float SYS_B = 6.21;

boolean isFrictionCompensated = true; boolean isVff = true;
const float V_FRICTION = 0.20;
float Vffwd = 0;
float Vdist = 0;

static float lastmicroseconds = 0;

float filteredSignal = 0;

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

//################################################################
void setup()
{
  Serial.begin(115200);//Initialize the serial port ** Set Serial Monitor to same 115200

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);
  
  displayMenu();
  initMotorPins();
  initEncoderInterrupts();
  initPWMtimer2();
 
  driveMotor(1.5, mtrVel, mtrPos); // reset motor to starting position
  delay(2000);
  driveMotor(0.0, mtrVel, mtrPos);
  delay(100);
}
//################################################################
  
void loop() {

  manageMenu();
  syncSample();
  lastmicroseconds=micros();
  
  isProfileEn = isModuleEn; //engages the module
  if (isModuleEn) {

    isTrapezoid = false; //engages the trapezoid refrence velocity and acceleration
    isStepRef = false;//engages step refrence
    isBallBalance = true;//engages full ball balance
    
    //Trapezoid loop
    if (isTrapezoid){
    trapRefVel(700.0, 500.0, 1500.0); //displacement (radians), veloc (rad/sec), acceler (rad/sec/sec)
    
    calculateError();
    errPos = refPos - mtrPos;
    closedLoopPosPI_PZ(isFrictionCompensated, isVff, 0.015,0.01,.4350,6.0059); // friction enabled? Vff enabled? Kp,Ki, c, d
    }

    //Step Response loop
    else if (isStepRef){
    refPos = 500;
    calculateError();
    errPos = refPos - mtrPos;
    closedLoopPosPI_PZ(isFrictionCompensated, isVff, 0.16,0.1,5,20); // friction enabled? Vff enabled? Kp,Ki, c, d
    }

    //Ball Balance loop
    else if (isBallBalance){
    readDistanceSensor(); // objectDistmm is the object distance from the sensor in mm
    refDist = objectDistmm;
    toggleDistanceSetpoint();
    calculateError(); // location of function call for efficient software execution
   
    float SENSORPI = SENSORclosedLoopDistPI(isFrictionCompensated, isVff, .25, .2);//friction compensation?, Vff?, Sensor KP, Sensor Ki
    refPos = SENSORLeadCompensation(SENSORPI, 0.5556, 7.73899); //position error, c (1/T), d (1/(alpha*T))
    refPos = LPF(refPos);
    //refPos = lag_Compensate(refPos,10,1);//signal for compensation, alpha, T
    refPos = constrain(refPos, -GEAR_RATIO * 1.5, 0); // limit motor position range (in radians) at output shaft
    
    errPos = refPos - mtrPos;
    closedLoopPosPI_PZ(isFrictionCompensated, isVff, 0.16,0.1,5,20); // friction enabled? Vff enabled? Kp,Ki, c, d
    }
  }

  else { // stop motor by sending zero voltage command to pwm
    OCR2B = 0; OCR2A = 0;
    timeElapsedTicks=0;
    quadPos=0;
    lastquadPos = quadPos;
  }


  //Results printing section
  if (isModuleEn && (timeElapsedTicks * TSAMP_MSEC) < 400000) //stop motor after 4 seconds
  {
    printResults();
  }
  if (isPrinting) printResults();

  if (isModuleEn) timeElapsedTicks++;
  prevModuleEn = isModuleEn;

}

//********************************************************************

float readDistanceSensor()
{
  sensor.readSingle(false); //blocking read of distance sensor
  while (!sensor.dataReady()) {
    /*stay here, do nothing until fresh data available*/
  };
  
  objectDistmm = sensor.read(); //takes reading
  if (objectDistmm > 500) {
    objectDistmm = 500;
  }
  if (sensor.timeoutOccurred()) {
    Serial.print("distance sensor TIMEOUT");
  }
  return objectDistmm;
}
//********************************************************************
void initEncoderInterrupts(void)
{
  // Position encoder ISR setup
  // PCINT1_vect ISR triggered for enabled bit changes on PCMSK1

  cli(); // disable global interrupts
  PCMSK1 = 0b00001100; // ENC3,2,1,0 -> A3,A2,A1,A0 -> d17,16,15,14
  PCICR = (1 << PCIE1); // enable pin change interrupts 8..14
  sei(); // enable global interrupts
}

//********************************************************************
void initPWMtimer2(void)
{
  //-----------------------------------------------------------------
  // Use Timer 2 for direct motor drive PWM generation.
  // Prescale = 1, FAST PWM, 8-bit (Mode 3) -> 62.5 kHz PWM
  // Output pins OC2B (d3~) and OC2A (d11~) driven by counter hardware.
  cli();
  TCCR2B = 0;
  TCCR2A = 0;
  TCCR2B =  (0 << WGM22); // start FAST PWM mode 3 setup
  TCCR2A =  (1 << WGM21) | (1 << WGM20); // finish FAST PWM setup
  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20); // clock prescale = 1
  TCCR2A |= (1 << COM2B1) | (0 << COM2B0); // OCR2B pin (d3~) noninverting PWM
  TCCR2A |= (1 << COM2A1) | (0 << COM2A0); // OCR2A pin (d11~) noninverting PWM
  OCR2B = 1; OCR2A = 1;
  sei();
}

//********************************************************************
ISR(PCINT1_vect) // vector to quadrature decoder
{
  //digitalWrite(ALED,!digitalRead(ALED));
  isEncoderChange = true;
  decodeEncoder32();
}

//********************************************************************
void decodeEncoder32(void) // 2 bit quad decoder
{
  const float MTR_RAD_PER_TICK = TWO_PI / 32;
  static byte oldquadratureCode = 0;

  oldquadratureCode = quadratureCode;
  quadratureCode = (PINC & 0b00001100); // inner tracks

  // Quadrature sequence: 0,8,12,4  (update ?CW facing end)
  switch (quadratureCode)
  {
    case 0:
      if (oldquadratureCode ==  4)  quadPos += MTR_RAD_PER_TICK;
      if (oldquadratureCode ==  8)  quadPos -= MTR_RAD_PER_TICK;
      break;
    case 8:
      if (oldquadratureCode ==  0)  quadPos += MTR_RAD_PER_TICK;
      if (oldquadratureCode ==  12) quadPos -= MTR_RAD_PER_TICK;
      break;
    case 12:
      if (oldquadratureCode ==  8) quadPos += MTR_RAD_PER_TICK;
      if (oldquadratureCode ==  4) quadPos -= MTR_RAD_PER_TICK;
      break;
    case 4:
      if (oldquadratureCode ==  12) quadPos += MTR_RAD_PER_TICK;
      if (oldquadratureCode ==  0)  quadPos -= MTR_RAD_PER_TICK;
      break;
  }

} // decodeEncoder32( )


//********************************************************************
void syncSample() // set the sample rate for ADC and therefore for the main loop
{ // sample interval time is set by TIC_MSEC
  const unsigned long TIC_USEC = TIC_MSEC * 1000UL;
  const byte ADCSRA_ISR = 0b11101111; // auto ISR, clkdiv = 128
  static unsigned long tic, stake = 0;
  static boolean first_run = true;
  if (first_run) {
    stake = micros();  // only runs first time to set stake
    first_run = false;
  }

  while ((tic - stake) < TIC_USEC) tic = micros(); // wait here until
  stake = tic;
  ADCSRA = ADCSRA_ISR; // start oversample-average series
}

//********************************************************************
ISR (ADC_vect)
{
  const byte N_ADC_AVE = 80;
  const float INV_N_ADC = 1.0 / N_ADC_AVE;
  static byte nConv = 0;
  static unsigned int loAccum = 0, hiAccum = 0;

  //SET_TP0_HI;
  loAccum += ADCL; // lower 8 bits: must read before ADCH per Atmel
  hiAccum += ADCH; // upper 2 bits

  if (++nConv >= N_ADC_AVE)
  {
    //SET_TP1_HI;
    adcReading = INV_N_ADC * (256UL * hiAccum + loAccum);
    hiAccum = 0; loAccum = 0;
    nConv = 0;
    ADCSRA &= ~bit(ADIE); // stop auto conversions
    //SET_TP1_LO;
  }
  //SET_TP0_LO;

}  // end of ADC_vect

//********************************************************************
void  displayMenu()
{
  Serial.println("\nEnter 'e' to toggle module enable.");
  //Serial.println("Enter 'g' to go.");
}

//********************************************************************
void  manageMenu()
{
  char inChar = Serial.read();
  if (inChar == 'e')
  {
    isModuleEn = !isModuleEn;
    //digitalWrite(PWMVAL_PIN, isModuleEn); // PWM disable low
    if (isModuleEn) {
      Serial.println(F("Module ENABLED"));
      timeElapsedTicks = 0;
      oldProfileEn=false; isProfileEn=true;
    }
    else {
      Serial.println(F("Module DISABLED"));
      quadPos=0;
      lastquadPos=quadPos;
    }

  }
  else if (inChar == 'g')
  {
    timeElapsedTicks = 0;
    dir = 1.0;
    isProfileEn = true;
  }
  else if (inChar == 'p')
  {
    isPrinting = !isPrinting;
  }
  else if (inChar == 'f')
  {
    isFrictionCompensated = !isFrictionCompensated;
    if (isFrictionCompensated) Serial.println(F("Stiction Comp Enabled"));
  }
}

//********************************************************************
void  initMotorPins()
{ // configure pins as input or output
  pinMode(ENC2, INPUT); // Encoder A
  pinMode(ENC3, INPUT); // Encoder B
  pinMode(PWM_M1A_PIN, OUTPUT); // set motor PWM signal to output
  pinMode(PWM_M1B_PIN, OUTPUT); // set motor direction pin to output
}


//*********************************************************************
void stepRefPos(float pos_rad) // step input velocity reference levels
{
  static float refPos_last_time = 0;
  oldProfileEn = isProfileEn;
  if (timeElapsedTicks < 0)  refVel = 0.0;
  else if (timeElapsedTicks < 40000)  refPos = pos_rad;
  else if (timeElapsedTicks < 80)  refPos = -pos_rad;
  else if (timeElapsedTicks < 100) refPos = 0.0;
  else isProfileEn = false;
  //refPos += refVel_last_time * TSAMP;
  //refVel_last_time = refVel;
  isShowStats = (oldProfileEn && !isProfileEn);
}

//********************************************************************
void MTRclosedLoopPosPI(boolean isFrictionCompensated, boolean isVff, float MTRKpGain, float MTRKiGain)
{
  float MTRKp = MTRKpGain;
  float MTRKi = MTRKiGain;

  Vffwd = ((SYS_B / SYS_A) * refVel + (1.0 / SYS_A) * refAcc);
  Vctl = PI_compensate_MOTOR(errPos, MTRKp, MTRKi); // PI control

  if (isFrictionCompensated) {
    Vctl += VfrictionVelocity(refVel);
  }
  if (isVff) {
    Vctl += Vffwd;
  }
  Varm = Vctl;
  if (isModuleEn) driveMotor(Varm, mtrVel, mtrPos);
}

//********************************************************************
void closedLoopPosPI_PZ(boolean isFrictionCompensated, boolean isVff, float KpGain, float KiGain, float CompC, float CompD)
{
  float Kp = KpGain;
  float Ki = KiGain;

  Vffwd = ((SYS_B / SYS_A) * refVel + (1.0 / SYS_A) * refAcc);
  Vctl = PI_compensate_MOTOR(errPos, Kp, Ki); // PI control
  Vctl = MOTORLeadCompensation(Vctl, CompC, CompD); // PD control, signal, CompC, CompD)
  //Vctl= PD_compensate(Kp* errPos, CompC, CompD); // PD control, signal, CompC, CompD)
  if (isFrictionCompensated) {
    Vctl += VfrictionVelocity(refVel);
  }
  if (isVff) {
    Vctl += Vffwd;
  }
  Varm = Vctl;
  if (isModuleEn) driveMotor(Varm, mtrVel, mtrPos);
  //Serial.println(micros()-lastmicroseconds);
}

//********************************************************************
float SENSORclosedLoopDistPI(boolean isFrictionCompensated, boolean isVff, float SENSORKpGain, float SENSORKiGain)
{
  float SENSORKp = SENSORKpGain;
  float SENSORKi = SENSORKiGain;

  float posRefPI = PI_compensate_SENSOR(errDist, SENSORKp, SENSORKi); // PI Distance control

  return posRefPI; 
}

//********************************************************************
float SENSORLeadCompensation(float x, float c, float d)
{

  // Proportional plus Derivative compensator
  // Cancel pole at -c, replace with a pole at -d.
  // Implement (s+c)/(s+d) as 1 + (c-d)/(s+d).
  // and post scale by d/c to get unity gain at DC.
  // PD section
  //d = pole, c = zero
  
  static float yd = 0.0;
  yd += ((c - d) * x - d * yd) * TSAMP;
  float ypd = x + yd;
  ypd = ypd * d / c; // correct scaling to unity gain at DC.
  return ypd;

}

//********************************************************************
float MOTORLeadCompensation(float x, float c, float d)
{

  // Proportional plus Derivative compensator
  // Cancel pole at -c, replace with a pole at -d.
  // Implement (s+c)/(s+d) as 1 + (c-d)/(s+d).
  // and post scale by d/c to get unity gain at DC.
  // PD section
  //d = pole, c = zero
  
  static float yd = 0.0;
  yd += ((c - d) * x - d * yd) * TSAMP;
  float ypd = x + yd;
  ypd = ypd * d / c; // correct scaling to unity gain at DC.
  return ypd;

}

//********************************************************************
float PI_compensate_SENSOR(float x, float Kp, float Ki)
{
  // Implement (Kp s+Ki)/s as Kp + Ki/s.
  static float Integralx = 0.0;
  Integralx += x * TSAMP;
  float ypi = Kp * x + Ki * Integralx;
  // Apply integral compensator windup clipping.
  return ypi;
  
}

//********************************************************************
float PI_compensate_MOTOR(float x, float Kp, float Ki)
{
  // Implement (Kp s+Ki)/s as Kp + Ki/s.
  static float Integralx = 0.0;
  Integralx += x * TSAMP;
  float ypi = Kp * x + Ki * Integralx;
  // Apply integral compensator windup clipping.
  return ypi;

  
}


//********************************************************************
void driveMotor(float Varm, float &vel, float &disp_rad)
{
  // 8 bit PWM, ~5 volt rail
  // note: reversed sign of Varm to get positive speed for positive volts
  float pwm_command = -Varm * PWM_PER_VOLT;

  if (pwm_command < 0) { // negative case -- set direction CW
    pwm_command = - int(pwm_command);
    OCR2A = 0;
    OCR2B = constrain(pwm_command, 0, 255);
    //Serial.println(OCR2B);
  }
  else
  { //positive case -- set direction CW
    pwm_command = int(pwm_command);
    OCR2B = 0;
    OCR2A = constrain(pwm_command, 0, 255);
    //Serial.println(OCR2A);
  }
}

//********************************************************************
void calculateError(void)
{
  mtrVel = (quadPos - lastquadPos) / (TSAMP_MSEC * 0.001); // radians per time interval (rad/sec)
  lastquadPos = quadPos;
  mtrPos = quadPos;
  errVel = refVel - mtrVel;
  errPos = refPos - mtrPos;
  
  errDist = refDist - objectDistmm;
}

//********************************************************************
void printResults(void){

//Motor TrapRef Data
//If isTrapezoid is true print trapezoid outputs
 if (isTrapezoid){
    if (isModuleEn != prevModuleEn)
        {
        //print header
        Serial.print("time (msec): ");
        Serial.print("\tmotorspeed: (rad/sec)");
        Serial.print("\tquadPos: (rad)");
        Serial.print("\t errVel: (rad/sec)");
        Serial.print("\t Vctrl (V)");
        Serial.print("\t Wref (rad/sec)");
        Serial.print("\t Posref (rad)");
        Serial.println();
        //lastquadPos = quadPos;
      }
      Serial.print(timeElapsedTicks * TSAMP_MSEC); Serial.print("\t");
      Serial.print(mtrVel); Serial.print("\t");
      Serial.print(quadPos); Serial.print("\t");
      Serial.print(errVel); Serial.print("\t");
      Serial.print(Vctl); Serial.print("\t");
      Serial.print(refVel); Serial.print("\t");
      Serial.print(refPos);
      Serial.println();
  }

//----------------------------------------------

//Sensor data
//Uncomment for sensor data
/*
else{
Serial.print(objectDistmm);
Serial.print("\n");

}
*/

//----------------------------------------------


//Final Print data
//If isTrapezoid is false print final data

  else{
    if (isModuleEn != prevModuleEn)
        {
        //print header
        Serial.print("time(msec): ");
        Serial.print("\t refDist:(rad/sec)");
        Serial.print("\t objectDistmm:(mm)");
        Serial.print("\t errDist: (rad/sec)");
        Serial.print("\t refPos (Deg)");
        Serial.print("\t mtrPos (rad/sec)");
        Serial.print("\t errPos (rad)");
        Serial.print("\t Varm (rad)");
        Serial.print("\t mtrVel (rad)");
        Serial.println();
      }
        
      Serial.print(timeElapsedTicks * TSAMP_MSEC); Serial.print("\t");
      Serial.print(refDist); Serial.print("\t");
      Serial.print(objectDistmm); Serial.print("\t");
      Serial.print(errDist); Serial.print("\t");
      Serial.print(refPos); Serial.print("\t");
      Serial.print(mtrPos); Serial.print("\t");
      Serial.print(errPos); Serial.print("\t");
      //Serial.print(TorqueCommand); Serial.print(",");
      Serial.print(Varm); Serial.print("\t");
      Serial.print(mtrVel);
      Serial.println();
  }
}

//********************************************************************
float VfrictionVelocity(float Vcmd)
{ // calculates voltage required to overcome friction.
  // direction of voltage depends on commanded direction of velocity
  float frictionV = 0.0;
  //Coulomb friction
  if (Vcmd >  0.001) frictionV =  V_FRICTION;
  if (Vcmd < -0.001) frictionV = -V_FRICTION;
  //static friction
  if ((mtrVel < 1) && (mtrVel > -1) && (Vcmd >  0.001)) frictionV += 0.4;
  if ((mtrVel < 1) && (mtrVel > -1) && (Vcmd <  -0.001)) frictionV -= 0.4;
  return frictionV;
}

//*********************************************************************
void trapRefVel(float specDisp_rad, float specVel, float specAcc) // trap velocity profile
{
  if (!oldProfileEn && isProfileEn)
  {
    //specDisp_rad = 300.0; specVel = 150.0; specAcc = 800.0;
    trapBuildSymAcc(specDisp_rad, specVel, specAcc);
    dwellStartT = 10; dwellEndT = 10; specVdist = 0.0;

    //specDisp_rad = 300.0, specVel = 100.0, specAcc = 200.0;
    //trapBuildSymAcc(specDisp_rad, specVel, specAcc);
    //dwellStartT = 50, dwellEndT = 200; specVdist = 2.0;

    //specDisp_rad = 206.9, specVel = 110.0, specAcc = 180.0;
    //trapBuildSymAcc(specDisp_rad, specVel, specAcc);
    //dwellGoT = 300, dwellUpT = 500, dwellEndT = 200; specVdist = abs(V_DIST);

    // Times when speed changes
    T0 = dwellStartT;
    T1 = T0 + accT; T2 = T1 + platT; T3 = T2 + accT; T4 = T3 + dwellEndT;
    T5 = T4 + accT; T6 = T5 + platT; T7 = T6 + accT; T8 = T7 + dwellEndT;

    //T0 = dwellGoT;
    //T1 = T0 + accT, T2 = T1 + platT, T3 = T2 + accT, T4 = T3 + dwellUpT;
    //T5 = T4 + accT, T6 = T5 + platT, T7 = T6 + accT, T8 = T7 + dwellEndT;

    // Times when disturbances are applied and values
    D1 = T3 + 100; D2 = D1 + 200;
    dD1 = specVdist; dD2 = -specVdist;

    //simJumpOn_simJumpOff();
    //simJumpOn_simFileOff();
    //weightOn_simJumpOff();
    //weightOn_simFileOff();
  }
  tick = timeElapsedTicks;
  // ---- Command profile -------------------------------------
  oldProfileEn = isProfileEn;
  if (tick < T0)       refAcc = 0.0;
  else if (tick < T1)  refAcc = dir * trapAcc;
  else if (tick < T2)  refAcc = 0.0;
  else if (tick < T3)  refAcc = -dir * trapAcc;
  else if (tick < T4)  refAcc = 0.0;
  //else if (tick == T4) isProfileEn = false;

  else if (tick < T5)  refAcc = -dir * trapAcc;
  else if (tick < T6)  refAcc = 0.0;
  else if (tick < T7)  refAcc = dir * trapAcc;
  else if (tick < T8)  refAcc = 0.0;
  else if (tick == T8) isProfileEn = false;

  //---- Disturbance profile -----------------------------------
  if (tick < D1) Vdist = 0.0;
  else if (tick == D1) Vdist += dD1 * specVdist; // module, sim load
  else if (tick == D2) Vdist += dD2 * specVdist;
  else if (tick == D3) Vdist += dD3 * specVdist;
  else if (tick == D4) Vdist += dD4 * specVdist;
  else if (tick == D5) Vdist += dD5 * specVdist;
  else if (tick == D6) Vdist += dD6 * specVdist;
  else if (tick == D7) Vdist += dD7 * specVdist;
  else if (tick == D8) Vdist += dD8 * specVdist;

  //---- Profile integration -----------------------------------
  if (isProfileEn) {
    refVel += refAcc * TSAMP;
    refPos += refVel * TSAMP;
  }
  else {
    refAcc = 0.0;
    refVel = 0.0;
  }

  isShowStats = (oldProfileEn && !isProfileEn);
}

//**************************************************************
void trapBuildSymAcc(float Disp_rad, float Vplat, float Acc)
{
  float tAcc = Vplat / Acc;
  float dAcc = Vplat * tAcc;
  float dPlat = Disp_rad - dAcc;
  float tPlat = dPlat / Vplat;

  trapAcc = Acc; // acceleration rate
  accT = int(round(tAcc / TSAMP)); // time duration of acceleration
  platT = int(round(tPlat / TSAMP)); // time duration of constant speed
  float dTrap = (accT + platT) * TSAMP * Vplat; // total displacement during trapezoid profile

  //  Serial.println();
  //  Serial.print("tAcc = ");  Serial.println(tAcc);
  //  Serial.print("dAcc = ");  Serial.println(dAcc);
  //  Serial.print("dPlat = "); Serial.println(dPlat);
  //  Serial.print("tPlat = "); Serial.println(tPlat);
  //  Serial.print("accT = ");  Serial.println(accT);
  //  Serial.print("platT = "); Serial.println(platT);
  //  Serial.print("dTrap = "); Serial.println(dTrap);
}

//********************************************************************
float LPF(float unfilteredSignal){
  
  filteredSignal = 0.2*unfilteredSignal + 0.8*filteredSignal;
  
  return filteredSignal;
}

//********************************************************************
void toggleDistanceSetpoint(void) {
  if (isModuleEn && ((((int)(timeElapsedTicks * TSAMP_MSEC * 0.001)) % 20) <= 10))
  { //every 10 seconds, change the refDist
    refDist = 150;
  }
  else
  {
    refDist = 250;
  }
}

//********************************************************************
float lag_Compensate (float x, float alpha, float T){
  //x is the signal that requires lag compensation
  //alpha is the magnitude of hte low freq gain compensation
  //T is the frequency at which the gain compensation occurs

  Serial.println(refPos,"\n");
  float lagGain = alpha*(((T*x)+1)/((alpha*T*x)+1));
  Serial.println(refPos,"\n");
  return lagGain;
}
