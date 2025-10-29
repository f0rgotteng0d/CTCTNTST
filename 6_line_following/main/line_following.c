#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

// =========================================================
//                  CONSTANTS & DEFINITIONS
// =========================================================

static const char *TAG = "line_follower";

// Operating mode
#define MODE NORMAL_MODE

// Line Sensor Array (LSA) settings
#define NUM_LSA_SENSORS 5
#define LSA_WHITE_MARGIN 0
#define LSA_BLACK_MARGIN 4095
#define LSA_MAP_LOW 0
#define LSA_MAP_HIGH 1000
#define BLACK_BOUNDARY 450 // Threshold for detecting black
const int SENSOR_WEIGHTS[NUM_LSA_SENSORS] = {-5, -3, 1, 3, 5};

// Motor settings
const int OPTIMUM_DUTY_CYCLE = 57;
const int LOWER_DUTY_CYCLE = 55;
const int HIGHER_DUTY_CYCLE = 62;

// PID settings
#define PID_ERROR_SCALE 10.0
#define PID_INTEGRAL_MIN -30
#define PID_INTEGRAL_MAX 30

// Turn & History settings
#define HISTORY_BUFFER_SIZE 15  // Replaces 'NOR'
#define U_TURN_HISTORY_THRESHOLD 0.1
#define U_TURN_DELAY_MS 300

// End-of-Line detection
#define REQUIRED_WHITE_COUNT 25 // Consec. readings to confirm end

// Pins
#define IR_SENSOR_PIN GPIO_NUM_0

// Maneuver settings for box pushing
#define PUSH_SPEED OPTIMUM_DUTY_CYCLE // Use optimum speed for pushing
#define REVERSE_SPEED OPTIMUM_DUTY_CYCLE
#define REVERSE_DURATION_MS 500     // Tune this: 1 second
#define UTURN_SPEED HIGHER_DUTY_CYCLE
#define UTURN_DURATION_MS 500       // Tune this: 1.5 seconds

// =========================================================
//                     DATA STRUCTURES
// =========================================================

/**
 * @brief Holds all hardware handles for the robot.
 */
typedef struct {
    motor_handle_t motor_left;
    motor_handle_t motor_right;
    adc_handle_t line_sensor;
} Peripherals;

/**
 * @brief Consolidates all dynamic state variables for the robot.
 */
typedef struct {
    // PID state
    float error;
    float prev_error;
    float difference;
    float cumulative_error;

    // Motor outputs
    float left_duty_cycle;
    float right_duty_cycle;

    // Sensor-derived flags
    bool far_left_detected;
    bool far_right_detected;
    bool u_turn_detected;
    bool all_black;
    int white_line_count; // Consec. all-white readings

    // U-turn history
    int left_sensor_history[HISTORY_BUFFER_SIZE];
    int right_sensor_history[HISTORY_BUFFER_SIZE];
    int history_index;
    float left_history_avg;
    float right_history_avg;

} RobotState;

// =========================================================
//                  FUNCTION PROTOTYPES
// =========================================================

// --- Initialization ---
static void init_robot_peripherals(Peripherals *periph);
static void init_robot_state(RobotState *state);

// --- Sensor & State Processing ---
static bool check_obstacle(); // <-- Changed to return bool
static void process_lsa_readings(line_sensor_array *readings);
static bool check_end_of_line(line_sensor_array *readings, RobotState *state);
static void update_robot_state_and_error(line_sensor_array *readings, RobotState *state);

// --- PID & Motor Calculation ---
static float calculate_pid_correction(RobotState *state);
static void calculate_motor_outputs(float correction, RobotState *state);

// --- Turn & History Logic ---
static float calculate_history_average(int sensor_history[]);
static void update_turn_history(line_sensor_array *readings, RobotState *state);

// --- Hardware Control ---
static void execute_motor_control(line_sensor_array *readings, RobotState *state, Peripherals *periph);
static void stop_motors(Peripherals *periph);
static void update_oled();

// =========================================================
//                 INITIALIZATION FUNCTIONS
// =========================================================

/**
 * @brief Initializes all robot hardware peripherals.
 */
static void init_robot_peripherals(Peripherals *periph) {
    // Motors
    ESP_ERROR_CHECK(enable_motor_driver(&periph->motor_left, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&periph->motor_right, MOTOR_A_1));

    // Line Sensor
    ESP_ERROR_CHECK(enable_line_sensor(&periph->line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());

    // IR Obstacle Sensor
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     // Using Pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

#ifdef CONFIG_ENABLE_OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    lv_obj_clean(lv_scr_act());
#endif
}

/**
 * @brief Initializes the robot state structure to default values.
 */
static void init_robot_state(RobotState *state) {
    // Zero out the entire structure
    memset(state, 0, sizeof(RobotState));
    // Any non-zero defaults could be set here
}

// =========================================================
//              SENSOR & STATE PROCESSING
// =========================================================

/**
 * @brief Checks the IR sensor and logs if an obstacle is detected.
 * @return true if obstacle is detected, false otherwise.
 */
static bool check_obstacle() {
    int ir_state = gpio_get_level(IR_SENSOR_PIN);
    if (ir_state == 0) { // Active-low sensor
        // ESP_LOGI(TAG, "Obstacle detected!"); // Can be noisy, log in main loop
        return true; // Yes, there is an obstacle
    }
    return false; // No, the path is clear
}

/**
 * @brief Bounds, maps, and inverts the raw LSA readings.
 */
static void process_lsa_readings(line_sensor_array *readings) {
    for (int i = 0; i < NUM_LSA_SENSORS; i++) {
        readings->adc_reading[i] = bound(readings->adc_reading[i], LSA_WHITE_MARGIN, LSA_BLACK_MARGIN);
        readings->adc_reading[i] = map(readings->adc_reading[i], LSA_WHITE_MARGIN, LSA_BLACK_MARGIN, LSA_MAP_LOW, LSA_MAP_HIGH);
        readings->adc_reading[i] = LSA_MAP_HIGH - readings->adc_reading[i]; // Invert
    }
}

/**
 * @brief Checks if all sensors are on white, tracking consecutive counts.
 * @return true if the end of the line is confirmed, false otherwise.
 */
static bool check_end_of_line(line_sensor_array *readings, RobotState *state) {
    bool all_white = true;
    for (int i = 0; i < NUM_LSA_SENSORS; i++) {
        if (readings->adc_reading[i] >= BLACK_BOUNDARY) {
            all_white = false;
            break;
        }
    }

    if (all_white) {
        state->white_line_count++;
    } else {
        state->white_line_count = 0;
    }

    return (state->white_line_count >= REQUIRED_WHITE_COUNT);
}

/**
 * @brief Calculates PID error and updates turn flags based on sensor state.
 * (Replaces the old calculate_error() function)
 */
static void update_robot_state_and_error(line_sensor_array *readings, RobotState *state) {
    state->all_black = true;
    float weighted_sum = 0, sum = 0, pos = 0;
    
    bool left_reading = readings->adc_reading[0] > BLACK_BOUNDARY;
    bool right_reading = readings->adc_reading[4] > BLACK_BOUNDARY;

    for (int i = 0; i < NUM_LSA_SENSORS; i++) {
        if (readings->adc_reading[i] > BLACK_BOUNDARY) {
            int k = 1;
            weighted_sum += (float)(SENSOR_WEIGHTS[i]) * k;
            sum += k;
            state->all_black = false; // At least one sensor is on black
        }
    }

    // Update turn detection flags
    state->far_left_detected = left_reading;
    state->far_right_detected = right_reading && !left_reading; // Prioritize left
    
    // Update U-turn flag
    if (state->all_black && (state->prev_error > 3 || state->prev_error < -3))
        state->u_turn_detected = true;
    else
        state->u_turn_detected = false;

    // Calculate position and error
    if (sum != 0) {
        pos = (weighted_sum - 1) / sum; // Original logic
    }

    if (state->all_black) {
        // If line is lost, use last known error direction
        state->error = state->prev_error > 0 ? 10 : -10;
    } else {
        state->error = pos;
    }
}

// =========================================================
//                 PID & MOTOR CALCULATION
// =========================================================

/**
 * @brief Calculates the PID correction value.
 * (Replaces the old calculate_correction() function)
 */
static float calculate_pid_correction(RobotState *state) {
    pid_const_t pid = read_pid_const(); // Read constants for tuning
    
    // Scale error (as in original)
    float scaled_error = state->error * PID_ERROR_SCALE;
    
    state->difference = scaled_error - state->prev_error;
    state->cumulative_error += scaled_error;

    // Limit integral windup
    state->cumulative_error = bound(state->cumulative_error, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);

    // Standard PID formula
    return pid.kp * scaled_error +
           pid.ki * state->cumulative_error +
           pid.kd * state->difference;
}

/**
 * @brief Calculates the final bounded duty cycles for left and right motors.
 */
static void calculate_motor_outputs(float correction, RobotState *state) {
    state->left_duty_cycle = bound((OPTIMUM_DUTY_CYCLE + correction),
                                  LOWER_DUTY_CYCLE, HIGHER_DUTY_CYCLE);
    state->right_duty_cycle = bound((OPTIMUM_DUTY_CYCLE - correction),
                                   LOWER_DUTY_CYCLE, HIGHER_DUTY_CYCLE);
}

// =========================================================
//                 TURN & HISTORY LOGIC
// =========================================================

/**
 * @brief Calculates the average of a sensor history buffer.
 */
static float calculate_history_average(int sensor_history[]) {
    int sum = 0;
    for (int i = 0; i < HISTORY_BUFFER_SIZE; i++) {
        sum += sensor_history[i];
    }
    return (float)sum / HISTORY_BUFFER_SIZE;
}

/**
 * @brief Stores current sensor state in history and recalculates averages.
 * (Replaces store_sensor_history() and the avg calls)
 */
static void update_turn_history(line_sensor_array *readings, RobotState *state) {
    // Store current binary reading
    state->left_sensor_history[state->history_index] = (readings->adc_reading[0] > BLACK_BOUNDARY) ? 1 : 0;
    state->right_sensor_history[state->history_index] = (readings->adc_reading[4] > BLACK_BOUNDARY) ? 1 : 0;
    
    // Increment circular buffer index
    state->history_index = (state->history_index + 1) % HISTORY_BUFFER_SIZE;

    // Recalculate averages
    state->left_history_avg = calculate_history_average(state->left_sensor_history);
    state->right_history_avg = calculate_history_average(state->right_sensor_history);
}

// =========================================================
//                   HARDWARE CONTROL
// =========================================================

/**
 * @brief Stops both motors.
 */
static void stop_motors(Peripherals *periph) {
    set_motor_speed(periph->motor_left, MOTOR_STOP, 0);
    set_motor_speed(periph->motor_right, MOTOR_STOP, 0);
}

/**
 * @brief Executes the motor control logic based on the current robot state.
 */
static void execute_motor_control(line_sensor_array *readings, RobotState *state, Peripherals *periph) {
    // Case 1: Special pattern (1 1 0 1 1) -> Move straight
    if (readings->adc_reading[0] > BLACK_BOUNDARY &&
        readings->adc_reading[1] > BLACK_BOUNDARY &&
        readings->adc_reading[2] < BLACK_BOUNDARY &&
        readings->adc_reading[3] > BLACK_BOUNDARY &&
        readings->adc_reading[4] > BLACK_BOUNDARY) {
        
        ESP_LOGI(TAG, "Special pattern - moving straight");
        set_motor_speed(periph->motor_left, MOTOR_FORWARD, state->left_duty_cycle);
        set_motor_speed(periph->motor_right, MOTOR_FORWARD, state->left_duty_cycle); // Note: uses left_duty_cycle for both
    }
    // Case 2: Hard Left Turn (Sensor 0)
    else if (state->far_left_detected) {
        ESP_LOGI(TAG, "Hard left turn");
        set_motor_speed(periph->motor_left, MOTOR_BACKWARD, state->right_duty_cycle);
        set_motor_speed(periph->motor_right, MOTOR_FORWARD, state->right_duty_cycle);
    }
    // Case 3: Right Turn (Sensor 4)
    else if (state->far_right_detected) {
        if (readings->adc_reading[2] > BLACK_BOUNDARY) {
            // Gentle right (center sensor also on)
            ESP_LOGI(TAG, "Gentle right");
            set_motor_speed(periph->motor_left, MOTOR_FORWARD, state->left_duty_cycle);
            set_motor_speed(periph->motor_right, MOTOR_FORWARD, state->right_duty_cycle);
        } else {
            // Hard right (spot turn)
            ESP_LOGI(TAG, "Hard right");
            set_motor_speed(periph->motor_left, MOTOR_FORWARD, state->left_duty_cycle);
            set_motor_speed(periph->motor_right, MOTOR_BACKWARD, state->left_duty_cycle);
        }
    }
    // Case 4: U-Turn (All black + previous large error)
    else if (state->u_turn_detected) {
        // Decide U-turn direction based on sensor history
        if (state->right_history_avg > U_TURN_HISTORY_THRESHOLD && 
            state->left_history_avg < U_TURN_HISTORY_THRESHOLD) {
            ESP_LOGI(TAG, "U-turn right");
            set_motor_speed(periph->motor_left, MOTOR_FORWARD, HIGHER_DUTY_CYCLE);
            set_motor_speed(periph->motor_right, MOTOR_BACKWARD, HIGHER_DUTY_CYCLE);
        } else {
            ESP_LOGI(TAG, "U-turn left");
            set_motor_speed(periph->motor_left, MOTOR_BACKWARD, HIGHER_DUTY_CYCLE);
            set_motor_speed(periph->motor_right, MOTOR_FORWARD, HIGHER_DUTY_CYCLE);
        }
        vTaskDelay(U_TURN_DELAY_MS / portTICK_PERIOD_MS);
    }
    // Case 5: Default PID Line Following
    else {
        // ESP_LOGD(TAG, "PID control"); // Use Debug for default case
        set_motor_speed(periph->motor_left, MOTOR_FORWARD, state->left_duty_cycle);
        set_motor_speed(periph->motor_right, MOTOR_FORWARD, state->right_duty_cycle);
    }
}

/**
 * @brief Updates the OLED display if PID values have changed.
 */
static void update_oled() {
#ifdef CONFIG_ENABLE_OLED
    pid_const_t pid = read_pid_const();
    if (pid.val_changed) {
        display_pid_values(pid.kp, pid.ki, pid.kd);
        reset_val_changed_pid_const();
    }
#endif
}

// =========================================================
//                    MAIN LINE FOLLOW TASK
// =========================================================

void line_follow_task(void *arg) {
    // 1. Initialization
    Peripherals periph;
    RobotState state;
    
    init_robot_peripherals(&periph);
    init_robot_state(&state);

    ESP_LOGI(TAG, "Line follower started with standard PID");

    // 2. Main Control Loop
    while (true) {
        // --- 1. Read ALL sensors ---
        bool obstacle_present = check_obstacle();
        line_sensor_array readings = read_line_sensor(periph.line_sensor);

        // --- 2. Process LSA data (always needed) ---
        process_lsa_readings(&readings);
        bool end_of_line = check_end_of_line(&readings, &state);

        // --- 3. Main Logic Tree ---
        if (obstacle_present) {
            /****************************************
             * STATE 1: OBSTACLE DETECTED (PUSHING)
             ****************************************/
            if (end_of_line) {
                // TASK: DESTINATION REACHED (Obstacle + End of Line)
                ESP_LOGI(TAG, "Obstacle at destination! Executing maneuver.");
                
                // 1. Stop (briefly)
                stop_motors(&periph);
                vTaskDelay(150 / portTICK_PERIOD_MS);

                // 2. Reverse
                ESP_LOGI(TAG, "Reversing...");
                set_motor_speed(periph.motor_left, MOTOR_BACKWARD, REVERSE_SPEED);
                set_motor_speed(periph.motor_right, MOTOR_BACKWARD, REVERSE_SPEED);
                vTaskDelay(REVERSE_DURATION_MS / portTICK_PERIOD_MS);

                // 3. U-Turn (Hard Right)
                ESP_LOGI(TAG, "Making U-Turn...");
                set_motor_speed(periph.motor_left, MOTOR_FORWARD, UTURN_SPEED);
                set_motor_speed(periph.motor_right, MOTOR_BACKWARD, UTURN_SPEED);
                vTaskDelay(UTURN_DURATION_MS / portTICK_PERIOD_MS);

                // 4. Stop and end task
                stop_motors(&periph);
                ESP_LOGI(TAG, "Maneuver complete. Stopping.");
                break; // Exit the while(true) loop

            } else {
                // TASK: STILL PUSHING BOX
                ESP_LOGI(TAG, "Obstacle detected, pushing straight.");
                // Drive straight forward, ignoring the line
                set_motor_speed(periph.motor_left, MOTOR_FORWARD, PUSH_SPEED);
                set_motor_speed(periph.motor_right, MOTOR_FORWARD, PUSH_SPEED);
            }

        } else if (end_of_line) {
            /****************************************
             * STATE 2: END OF LINE (NO OBSTACLE)
             ****************************************/
            stop_motors(&periph);
            ESP_LOGI(TAG, "End of line detected (no obstacle). Stopping bot.");
            break; // Exit the while(true) loop

        } else {
            /****************************************
             * STATE 3: NORMAL LINE FOLLOWING
             ****************************************/
            
            // --- Update State & PID ---
            update_robot_state_and_error(&readings, &state);
            float correction = calculate_pid_correction(&state);
            update_turn_history(&readings, &state);

            // --- Calculate Motor Outputs ---
            calculate_motor_outputs(correction, &state);

            // --- Control Motors ---
            execute_motor_control(&readings, &state, &periph);

            // --- Update UI ---
            update_oled();

            // --- Logging & Loop Management ---
            state.prev_error = state.error; // Update prev_error for next loop

            ESP_LOGD(TAG, "Error: %.2f | Corr: %.2f | L: %.0f | R: %.0f | L_avg: %.2f | R_avg: %.2f",
                     state.error, correction, state.left_duty_cycle, state.right_duty_cycle,
                     state.left_history_avg, state.right_history_avg);
        }
        
        // --- Loop Delay (runs every loop) ---
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }


    // 3. Cleanup
    vTaskDelete(NULL);
}

// =========================================================
//                       APP MAIN
// =========================================================

void app_main() {
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
