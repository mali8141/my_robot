// Improved RC Car Controller with Current Protection and Motor Ramping
// Added soft start, direction change delays, and current limiting features

#include <Servo.h>

// Pin connections
#define STEERING_INPUT_PIN 2   
#define THROTTLE_INPUT_PIN 3   
#define SERVO_PIN 5            
#define MOTOR_PWM_PIN 6        
#define MOTOR_DIR_PIN 7        

// Motor protection settings
#define BATTERY_VOLTAGE 11.1   
#define MAX_MOTOR_VOLTAGE 6.0  
#define MAX_PWM_VALUE (int)(255 * MAX_MOTOR_VOLTAGE / BATTERY_VOLTAGE) // ~184

// New protection parameters
#define MOTOR_RAMP_RATE 5      // PWM units per loop (slower = gentler on fuse)
#define DIRECTION_CHANGE_DELAY 100  // ms delay when changing direction
#define MIN_THROTTLE_CHANGE 10      // Minimum change before updating motor
#define MOTOR_TIMEOUT 5000          // Max time at full power (ms)

// Variables
unsigned long start_time_steering, start_time_throttle;
int steering_pulse, throttle_pulse;
int current_motor_speed = 0;    // Current actual motor speed
int target_motor_speed = 0;     // Target speed from RC
bool current_motor_forward = true;
bool target_motor_forward = true;
bool rc_signal_detected = false;
unsigned long last_direction_change = 0;
unsigned long motor_start_time = 0;
unsigned long last_update = 0;
int last_throttle_pulse = 1500;

Servo steeringServo;
int servo_angle = 90;

void setup() {
  Serial.begin(115200);
  
  // Set pins
  pinMode(STEERING_INPUT_PIN, INPUT);
  pinMode(THROTTLE_INPUT_PIN, INPUT);
  
  steeringServo.attach(SERVO_PIN);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  
  // Initialize to safe state
  steeringServo.write(90); 
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  
  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), read_steering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INPUT_PIN), read_throttle, CHANGE);
  
  Serial.println("RC Car Started - Protected Mode with Motor Ramping!");
  Serial.print("Max PWM: "); Serial.print(MAX_PWM_VALUE); 
  Serial.print(" Ramp Rate: "); Serial.println(MOTOR_RAMP_RATE);
  
  last_update = millis();
}

void loop() {
  unsigned long current_time = millis();
  
  // Only update every 20ms to allow for smooth ramping
  if (current_time - last_update < 20) {
    return;
  }
  last_update = current_time;
  
  // Check for valid RC signals
  if (steering_pulse >= 1000 && steering_pulse <= 2000 &&
      throttle_pulse >= 1000 && throttle_pulse <= 2000) {
    
    rc_signal_detected = true;
    
    // Process steering (unchanged)
    servo_angle = map(steering_pulse, 1000, 2000, 0, 180);
    servo_angle = constrain(servo_angle, 0, 180);
    
    // Process throttle with improved logic
    processThrottle();
    
  } else {
    // No valid RC signal - safe mode
    rc_signal_detected = false;
    servo_angle = 90;
    target_motor_speed = 0;
    target_motor_forward = true;
  }
  
  // Apply motor ramping for smooth acceleration
  applyMotorRamping();
  
  // Control outputs
  steeringServo.write(servo_angle);
  controlMotor(current_motor_speed, current_motor_forward);
  
  // Debug output
  if (current_time % 200 < 20) { // Print every 200ms
    printDebugInfo();
  }
}

void processThrottle() {
  // Only process if throttle changed significantly
  if (abs(throttle_pulse - last_throttle_pulse) < MIN_THROTTLE_CHANGE) {
    return;
  }
  last_throttle_pulse = throttle_pulse;
  
  if (throttle_pulse > 1550) {
    // Forward direction
    target_motor_forward = true;
    target_motor_speed = map(throttle_pulse, 1550, 2000, 0, MAX_PWM_VALUE);
  } else if (throttle_pulse < 1450) {
    // Backward direction
    target_motor_forward = false;
    target_motor_speed = map(throttle_pulse, 1450, 1000, 0, MAX_PWM_VALUE);
  } else {
    // Deadband - stop motor
    target_motor_speed = 0;
  }
  
  target_motor_speed = constrain(target_motor_speed, 0, MAX_PWM_VALUE);
}

void applyMotorRamping() {
  unsigned long current_time = millis();
  
  // Check for direction change
  if (current_motor_speed > 0 && target_motor_forward != current_motor_forward) {
    // Direction change requested - first stop the motor
    if (current_time - last_direction_change > DIRECTION_CHANGE_DELAY) {
      if (current_motor_speed > 0) {
        // Still slowing down
        current_motor_speed = max(0, current_motor_speed - MOTOR_RAMP_RATE * 2); // Brake faster
        return;
      } else {
        // Stopped - now change direction
        current_motor_forward = target_motor_forward;
        last_direction_change = current_time;
      }
    } else {
      return; // Wait for direction change delay
    }
  }
  
  // Normal ramping
  if (current_motor_speed < target_motor_speed) {
    // Accelerating
    current_motor_speed = min(target_motor_speed, current_motor_speed + MOTOR_RAMP_RATE);
    if (current_motor_speed == MOTOR_RAMP_RATE) {
      motor_start_time = current_time; // Mark start of acceleration
    }
  } else if (current_motor_speed > target_motor_speed) {
    // Decelerating
    current_motor_speed = max(target_motor_speed, current_motor_speed - MOTOR_RAMP_RATE);
  }
  
  // Safety timeout - reduce power after extended full throttle
  if (current_motor_speed >= MAX_PWM_VALUE * 0.8 && 
      current_time - motor_start_time > MOTOR_TIMEOUT) {
    Serial.println("Motor timeout - reducing power for thermal protection");
    target_motor_speed = MAX_PWM_VALUE * 0.6; // Reduce to 60%
    motor_start_time = current_time; // Reset timer
  }
}

void controlMotor(int speed, bool forward) {
  // Ensure speed doesn't exceed safe limits
  speed = constrain(speed, 0, MAX_PWM_VALUE);
  
  // Set direction
  digitalWrite(MOTOR_DIR_PIN, forward ? HIGH : LOW);
  
  // Set speed using PWM
  analogWrite(MOTOR_PWM_PIN, speed);
}

void printDebugInfo() {
  float effective_voltage = (float)current_motor_speed / 255 * BATTERY_VOLTAGE;
  
  Serial.print("RC: "); Serial.print(rc_signal_detected ? "OK" : "LOST");
  Serial.print(" | St: "); Serial.print(steering_pulse);
  Serial.print("->"); Serial.print(servo_angle); Serial.print("°");
  Serial.print(" | Th: "); Serial.print(throttle_pulse);
  Serial.print(" | Target: "); Serial.print(target_motor_speed);
  Serial.print(" Current: "); Serial.print(current_motor_speed);
  Serial.print(" ("); Serial.print(effective_voltage, 1); Serial.print("V)");
  Serial.print(" Dir: "); Serial.println(current_motor_forward ? "FWD" : "REV");
}

// Interrupt handlers (unchanged)
void read_steering() {
  if (digitalRead(STEERING_INPUT_PIN) == HIGH) {
    start_time_steering = micros();
  } else {
    steering_pulse = micros() - start_time_steering;
  }
}

void read_throttle() {
  if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
    start_time_throttle = micros();
  } else {
    throttle_pulse = micros() - start_time_throttle;
  }
}
