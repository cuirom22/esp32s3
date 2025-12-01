// --- Motor 1 Pins ---
#define M1_PWM   4
#define M1_INA   17
#define M1_INB   16
#define M1_EN    36   // optional: tie to 5V if not connected to ESP32

// --- Motor 2 Pins ---
#define M2_PWM   5
#define M2_INA   18
#define M2_INB   7    // replaced GPIO19 (USB D-) with GPIO7
#define M2_EN    37   // optional: tie to 5V if not connected to ESP32

// Encoder Pins
#define L_ENCA  1
#define L_ENCB  2 
#define R_ENCA  10
#define R_ENCB  11

// Ultrasonic Pins
#define US1_TRIG   6
#define US1_ECHO   3
#define US2_TRIG   14
#define US2_ECHO   15
#define US3_TRIG  47
#define US3_ECHO   21

// For Odometry (MUST MEASURE)
#define WHEEL_DIAMETER_CM   6.5    // your wheel diameter
#define WHEEL_CIRCUM_CM     (WHEEL_DIAMETER_CM * PI)
#define ENCODER_RESOLUTION  36.0   // counts per revolution
#define WHEEL_BASE_CM       15.0   // distance between wheels


// --- PWM Settings ---
#define PWM_FREQ     20000   // 20 kHz
#define PWM_RES      10      // 10-bit resolution (0–1023)

// --- LEDC Channels ---
#define CH_M1  0
#define CH_M2  1

