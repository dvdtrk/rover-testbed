#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "pico_uart_transports.h"

#include <rmw_microros/rmw_microros.h>

// Side selection -----------------------------------------------------------------
// Build-time selectable via CMake: -DMOTOR_SIDE=LEFT or -DMOTOR_SIDE=RIGHT
// Falls back to LEFT_SIDE if not defined by the build system.
#if !defined(LEFT_SIDE) && !defined(RIGHT_SIDE)
	#define LEFT_SIDE
#endif

#ifdef LEFT_SIDE
	#define NODE_NAME 		"motor_controller_left"
	#define TOPIC_CMD_VEL 		"cmd_vel_left"
	#define TOPIC_CURRENT 		"motor_current_left"
	#define TOPIC_ENCODER_FRONT 	"encoder_front_left"
	#define TOPIC_ENCODER_REAR 	"encoder_rear_left"
	#define TOPIC_FAULT 		"motor_fault_left"
	#define TOPIC_FAULT_CLEAR 	"motor_fault_clear_left"
#else
	#define NODE_NAME 		"motor_controller_right"
	#define TOPIC_CMD_VEL 		"cmd_vel_right"
	#define TOPIC_CURRENT 		"motor_current_right"
	#define TOPIC_ENCODER_FRONT 	"encoder_front_right"
	#define TOPIC_ENCODER_REAR 	"encoder_rear_right"
	#define TOPIC_FAULT 		"motor_fault_right"
	#define TOPIC_FAULT_CLEAR 	"motor_fault_clear_right"
#endif

// Pin definitions ----------------------------------------------------------------
#define PWM_PIN 	2 // GP2 -> MDD20A PWM (motor speed)
#define DIR_PIN 	3 // GP3 -> MDD20A DIR (motor direction)
#define ENC_A_FRONT 	4 // GP4 -> front encoder channel A
#define ENC_B_FRONT 	5 // GP5 -> front encoder channel B
#define ENC_A_REAR 	6 // GP6 -> rear encoder channel A
#define ENC_B_REAR 	7 // GP7 -> rear encoder channel B
#define ACS712_PIN 	26 // GP26 -> ACS712 current sensor output (ADC0)

// Motor parameters ---------------------------------------------------------------
#define MAX_CURRENT_AMPS 	6.0f    // software overcurrent threshold (amps)
#define ACS712_SENSITIVITY 	0.066f  // 66mV/A sensitivity for 30A variant
#define ACS712_ZERO_VOLTS 	2.5f    // ACS712 output voltage at 0A
#define ADC_REF_VOLTAGE 	3.3f    // Pico ADC reference voltage
#define ADC_RESOLUTION 		4095.0f // 12-bit ADC max value
#define VOLTAGE_DIVIDER 	0.48f  // voltage divider ratio: 20k/(10k+20k) = 0.667
					// changed to 0.48 because measured 1.2V at GP26
#define PWM_MAX_COUNT 		12500   // PWM wrap: 125MHz / 12500 = 10kHz
#define CMD_TIMEOUT_MS 		500     // watchdog: stop motors after 500ms silence

// Encoder state ------------------------------------------------------------------
// volatile: these are modified in interrupt context, must not be cached
volatile int32_t encoder_count_front = 0;
volatile int32_t encoder_count_rear  = 0;

// Safety state -------------------------------------------------------------------
// NOTE: last_cmd_time starts at 0 intentionally
// Motor stays stopped until first cmd_vel proves Pi 5 is alive
volatile uint32_t last_cmd_time     = 0;
volatile bool     overcurrent_fault = false;

// micro-ROS handles --------------------------------------------------------------
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t fault_clear_sub;
rcl_publisher_t    current_pub;
rcl_publisher_t    encoder_front_pub;
rcl_publisher_t    encoder_rear_pub;
rcl_publisher_t    fault_pub;

geometry_msgs__msg__Twist  cmd_vel_msg;
std_msgs__msg__Int32       fault_clear_msg;
std_msgs__msg__Float32     current_msg;
std_msgs__msg__Int32       encoder_front_msg;
std_msgs__msg__Int32       encoder_rear_msg;
std_msgs__msg__Int32       fault_msg;

// Error handling macros -----------------------------------------------------------
// RCCHECK: fatal error — return false and stop setup
// RCSOFTCHECK: non-fatal — log and continue (used for publishers in main loop)
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) {} }

// PWM setup -----------------------------------------------------------------------
void pwm_setup() {
	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(PWM_PIN);

	// Set PWM frequency to 10kHz
	// System clock 125MHz / PWM_MAX_COUNT 12500 = 10kHz
	pwm_set_wrap(slice, PWM_MAX_COUNT);
	pwm_set_clkdiv(slice, 1.0f);

	// Explicitly zero duty cycle at startup
	// Motor stays off until first cmd_vel received
	pwm_set_chan_level(slice, pwm_gpio_to_channel(PWM_PIN), 0);

	pwm_set_enabled(slice, true);

	// DIR pin as digital output, default low = forward
	gpio_init(DIR_PIN);
	gpio_set_dir(DIR_PIN, GPIO_OUT);
	gpio_put(DIR_PIN, 0);
}

// Motor control -------------------------------------------------------------------
void set_motor(float speed) {
	// Clamp speed to valid range -1.0 to +1.0
	if (speed > 1.0f)  speed = 1.0f;
	if (speed < -1.0f) speed = -1.0f;

#ifdef RIGHT_SIDE
	speed = -speed;  // right side motor is mounted mirrored, invert to match left
#endif

	// Set direction pin based on sign of speed
	if (speed >= 0) {
		gpio_put(DIR_PIN, 0); // forward
	} else {
		gpio_put(DIR_PIN, 1); // reverse
		speed = -speed; // make positive for PWM duty cycle
	}

	// Convert 0.0-1.0 speed to PWM count and apply
	uint slice     = pwm_gpio_to_slice_num(PWM_PIN);
	uint channel   = pwm_gpio_to_channel(PWM_PIN);
	uint16_t level = (uint16_t)(speed * PWM_MAX_COUNT);
	pwm_set_chan_level(slice, channel, level);
}

void stop_motor() {
	set_motor(0.0f);
}

// ADC setup -----------------------------------------------------------------------
void adc_setup() {
	adc_init();
	adc_gpio_init(ACS712_PIN);
	adc_select_input(0);    // ADC0 = GP26
}

// Current reading -----------------------------------------------------------------
float read_current() {
	// Read raw 12-bit ADC value (0-4095)
	uint16_t raw = adc_read();

	// Convert to voltage at Pico ADC pin (after voltage divider)
	float adc_voltage = (raw / ADC_RESOLUTION) * ADC_REF_VOLTAGE;

	// Scale back up through voltage divider to get ACS712 output voltage
	float acs712_voltage = adc_voltage / VOLTAGE_DIVIDER;

	// Convert ACS712 voltage to current
	// ACS712 outputs 2.5V at 0A, swings +/-66mV per amp
	float current = (acs712_voltage - ACS712_ZERO_VOLTS) / ACS712_SENSITIVITY;

	return current;
}

// Overcurrent check ---------------------------------------------------------------
// Returns true if overcurrent detected, false if safe
// NOTE: also updates current_msg for publishing
bool check_overcurrent() {
	float current    = read_current();
	current_msg.data = current;

	if (current > MAX_CURRENT_AMPS || current < -MAX_CURRENT_AMPS) {
		stop_motor();
		overcurrent_fault = true;   // latch fault — requires explicit clear
		fault_msg.data    = 1;
		return true;
	}
	return false;
}

// Encoder interrupts --------------------------------------------------------------
// Quadrature decoding: read both channels on every edge to determine direction
// Critical section prevents count corruption if both IRQs fire simultaneously
void encoder_front_callback(uint gpio, uint32_t events) {
	bool a = gpio_get(ENC_A_FRONT);
	bool b = gpio_get(ENC_B_FRONT);

	uint32_t irq_state = save_and_disable_interrupts();
	if (gpio == ENC_A_FRONT) {
		// Rising A + B low = forward, Rising A + B high = reverse
		encoder_count_front += (events & GPIO_IRQ_EDGE_RISE) ? (b ? -1 : 1) : (b ? 1 : -1);
	} else {
		// Rising B + A high = forward, Rising B + A low = reverse
		encoder_count_front += (events & GPIO_IRQ_EDGE_RISE) ? (a ? 1 : -1) : (a ? -1 : 1);
	}
	restore_interrupts(irq_state);
}

void encoder_rear_callback(uint gpio, uint32_t events) {
	bool a = gpio_get(ENC_A_REAR);
	bool b = gpio_get(ENC_B_REAR);

	uint32_t irq_state = save_and_disable_interrupts();
	if (gpio == ENC_A_REAR) {
		encoder_count_rear += (events & GPIO_IRQ_EDGE_RISE) ? (b ? -1 : 1) : (b ? 1 : -1);
	} else {
		encoder_count_rear += (events & GPIO_IRQ_EDGE_RISE) ? (a ? 1 : -1) : (a ? -1 : 1);
	}
	restore_interrupts(irq_state);
}

// Encoder setup ---------------------------------------------------------------------
void encoder_setup() {
	// Configure all encoder pins as inputs with pull-ups
	// Pull-ups ensure defined state when encoder not connected
	gpio_init(ENC_A_FRONT); gpio_set_dir(ENC_A_FRONT, GPIO_IN); gpio_pull_up(ENC_A_FRONT);
	gpio_init(ENC_B_FRONT); gpio_set_dir(ENC_B_FRONT, GPIO_IN); gpio_pull_up(ENC_B_FRONT);
	gpio_init(ENC_A_REAR);  gpio_set_dir(ENC_A_REAR,  GPIO_IN); gpio_pull_up(ENC_A_REAR);
	gpio_init(ENC_B_REAR);  gpio_set_dir(ENC_B_REAR,  GPIO_IN); gpio_pull_up(ENC_B_REAR);

	// Register interrupts on both edges for full quadrature resolution
	// 4 edges per cycle = maximum encoder resolution
	gpio_set_irq_enabled_with_callback(ENC_A_FRONT,
		GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_front_callback);
	gpio_set_irq_enabled_with_callback(ENC_B_FRONT,
		GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_front_callback);
	gpio_set_irq_enabled_with_callback(ENC_A_REAR,
		GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_rear_callback);
	gpio_set_irq_enabled_with_callback(ENC_B_REAR,
		GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_rear_callback);
}

// Fault clear callback ----------------------------------------------------------------
// Pi 5 publishes 1 to motor_fault_clear_left/right to attempt fault recovery
// Only clears if current is actually safe — prevents clearing during active fault
void fault_clear_callback(const void * msgin) {
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

	if (msg->data == 1) {
		float current = read_current();
		// Only clear latch if current is actually back within safe range
		if (current < MAX_CURRENT_AMPS && current > -MAX_CURRENT_AMPS) {
			overcurrent_fault = false;
			fault_msg.data    = 0;
		}
	}
}

// cmd_vel callback ---------------------------------------------------------------------
// Called by micro-ROS executor when velocity command received from Pi 5
void cmd_vel_callback(const void * msgin) {
	const geometry_msgs__msg__Twist * msg =
		(const geometry_msgs__msg__Twist *)msgin;

	// Update watchdog timestamp — proves Pi 5 is alive
	last_cmd_time = to_ms_since_boot(get_absolute_time());

	// Ignore velocity commands while fault is latched
	// Pi 5 must explicitly clear fault before motor will move again
	if (overcurrent_fault) return;

	float speed = (float)msg->linear.x;

	// Check current before acting on command
	// NOTE: check_overcurrent() also runs in main loop — this is intentional
	// The callback check prevents acting on stale commands during a fault
	if (!check_overcurrent()) {
		set_motor(speed);
	}
}

// micro-ROS setup ----------------------------------------------------------------------
bool microros_setup() {
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t  support;

	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

	// Subscribers -------------------------------------------------------------------
	// cmd_vel: velocity commands from Pi 5 navigation stack
	RCCHECK(rclc_subscription_init_default(
		&cmd_vel_sub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		TOPIC_CMD_VEL));

	// motor_fault_clear: Pi 5 sends 1 to attempt fault recovery
	RCCHECK(rclc_subscription_init_default(
		&fault_clear_sub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		TOPIC_FAULT_CLEAR));

	// Publishers --------------------------------------------------------------------
	// motor_current: ACS712 reading in amps
	RCCHECK(rclc_publisher_init_default(
		&current_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		TOPIC_CURRENT));

	// encoder_front/rear: accumulated tick counts since boot
	RCCHECK(rclc_publisher_init_default(
		&encoder_front_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		TOPIC_ENCODER_FRONT));

	RCCHECK(rclc_publisher_init_default(
		&encoder_rear_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		TOPIC_ENCODER_REAR));

	// motor_fault: 1 = overcurrent fault latched, 0 = normal
	RCCHECK(rclc_publisher_init_default(
		&fault_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		TOPIC_FAULT));

	// Executor ----------------------------------------------------------------------
	// 2 = number of subscriptions registered
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

	RCCHECK(rclc_executor_add_subscription(
		&executor, &cmd_vel_sub, &cmd_vel_msg,
		&cmd_vel_callback, ON_NEW_DATA));

	RCCHECK(rclc_executor_add_subscription(
		&executor, &fault_clear_sub, &fault_clear_msg,
		&fault_clear_callback, ON_NEW_DATA));

	// Initialize fault message to 0 (no fault)
	fault_msg.data = 0;

	// Main loop ---------------------------------------------------------------------
	while (true) {
		// Process incoming ROS 2 messages (cmd_vel and fault_clear)
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

		// Watchdog: stop motor if no cmd_vel received within timeout
		// Protects against Pi 5 crash, agent disconnect, or comms failure
		if (to_ms_since_boot(get_absolute_time()) - last_cmd_time > CMD_TIMEOUT_MS) {
			stop_motor();
		}

		// Periodic overcurrent check between commands
		// NOTE: also runs inside cmd_vel_callback — both are intentional
		check_overcurrent();

		// Publish current sensor reading
		RCSOFTCHECK(rcl_publish(&current_pub, &current_msg, NULL));

		// Publish encoder counts (separate front/rear for fault diagnosis)
		encoder_front_msg.data = encoder_count_front;
		encoder_rear_msg.data  = encoder_count_rear;
		RCSOFTCHECK(rcl_publish(&encoder_front_pub, &encoder_front_msg, NULL));
		RCSOFTCHECK(rcl_publish(&encoder_rear_pub,  &encoder_rear_msg,  NULL));

		// Publish fault state (0 = normal, 1 = overcurrent latched)
		RCSOFTCHECK(rcl_publish(&fault_pub, &fault_msg, NULL));

		sleep_ms(10);   // 100Hz main loop
	}

	return true;
}


#define CMD_FORWARD  0x01
#define CMD_BACKWARD 0x02
#define CMD_STOP     0x03


// Main ----------------------------------------------------------------------------------
int main() {

	/*
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        for (int i = 0; i < 3; i++) {
                gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
                gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
        }


	stdio_init_all();

	// NOTE: stdio_init_all() intentionally omitted
	// It would claim UART0 (GP0/GP1) conflicting with micro-ROS transport

	// Initialize hardware peripherals
	pwm_setup();
	adc_setup();
	encoder_setup();

	// Configure micro-ROS to use Pico UART transport (GP0/GP1)
	// This is how the Pico communicates with the micro-ROS agent on Pi 5
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

	// Wait for micro-ROS agent to come online
	// Pico will sit here until Pi 5 agent is running
	while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
		sleep_ms(100);
	}


	for (int i = 0; i < 6; i++) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		sleep_ms(150);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		sleep_ms(150);
	}



	// Enter micro-ROS main loop — never returns
	microros_setup();

	return 0;
	*/




        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);

        // Boot confirmation: 3 quick blinks = this firmware flashed and is running
        for (int i = 0; i < 3; i++) {
                gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
                gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
        }

        stdio_init_all();
        sleep_ms(2000);
        printf("Command-driven motor test starting (control-byte protocol)\n");

        pwm_setup();
        adc_setup();
        encoder_setup();

        stop_motor();
        gpio_put(PICO_DEFAULT_LED_PIN, 0);

        uint32_t last_telemetry_ms = 0;
        uint32_t last_cmd_ms = to_ms_since_boot(get_absolute_time());
        const uint32_t CMD_WATCHDOG_MS = 500;  // auto-stop if no command refresh within this window

        while (true) {
                int c = getchar_timeout_us(0);  // non-blocking check every loop

                if (c == CMD_FORWARD) {
                        set_motor(0.5f);
                        gpio_put(PICO_DEFAULT_LED_PIN, 1);
                        last_cmd_ms = to_ms_since_boot(get_absolute_time());
                } else if (c == CMD_BACKWARD) {
                        set_motor(-0.5f);
                        gpio_put(PICO_DEFAULT_LED_PIN, 1);
                        last_cmd_ms = to_ms_since_boot(get_absolute_time());
                } else if (c == CMD_STOP) {
                        stop_motor();
                        gpio_put(PICO_DEFAULT_LED_PIN, 0);
                        last_cmd_ms = to_ms_since_boot(get_absolute_time());
                }

                // Watchdog: if a move command isn't refreshed regularly, force-stop
                uint32_t now = to_ms_since_boot(get_absolute_time());
                if (now - last_cmd_ms > CMD_WATCHDOG_MS) {
                        stop_motor();
                        gpio_put(PICO_DEFAULT_LED_PIN, 0);
                }

                if (now - last_telemetry_ms >= 500) {
                        last_telemetry_ms = now;
                        printf("Current: %.2f A | EncFront: %d EncRear: %d\n",
                                read_current(), encoder_count_front, encoder_count_rear);
                }

                sleep_ms(10);
        }

        return 0;


	// for a single motor side, run:
	// sudo minicom -D /dev/ttyAMA2 -b 115200

	// or run for complete directional motor control (both sides):
	// python3 rover_control.py

}
