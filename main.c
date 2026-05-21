/*
 * main.c
 *
 * Initial version:  EEE 158 AY2025-26 Sem 1 Handlers
 * Modifications by: DE VILLA, Alberto
 *
 * Additional modifications by: SIBUG, Tristan Josh E., 2021-60143, WYZ
 * now for EEE 192
 * Module 5: Universal Synchronous/Asynchronous Receiver/Transmitter
 *           (Example)
 */

#include "platform/init.h"
#include "platform/usart.h"

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct prog_state_type
{
    /**
     * Flags for this program:
     *
     * 0x0001 = Full refresh pending
     * 0x0002 = Message update pending (implied by full refresh)
     * 0x0004 = Change colors (useful only with 0x0001)
     * 0x8000 = Transmission on-going
     */
    // int16_t tx_flags;
    uint16_t tx_flags;

    /*
     * State ID
     *
     *   0 = Idle (initial)
     * 0.5 = Initiate reception
     *   1 = Waiting for input
     *   2 = Waiting for any current transmission to complete
     * 2.5 = Generate output and enqueue it for transmission
     *   3 = Waiting for the output buffer to be fully transmitted
     */
    unsigned int state_id;

    // Transmit stuff
    struct platform_ro_buf_desc tx_desc[2]; // Simplified descriptors
    uint16_t tx_nr_desc;

    // Receiver stuff
    struct platform_usart_recv_data_type rx_info;
    char rx_buf[16];

    // Pointer to hold string we want to send back to phone
    const char *response_msg;
    char debug_buf[64];

    // Timed turn fields
    uint32_t turn_start_tick;
    uint32_t turn_duration_ticks;
    uint8_t turn_active;
    uint8_t recovery_stage;
    uint32_t recovery_wait_tick;
    uint8_t recovery_direction;
    uint8_t recovery_turn_count;

    // Milestone 4: Autonomous state
    uint8_t line_tracing_active;
    uint8_t wall_following_active;

    // SOS state
    uint8_t sos_active;         // 1 = SOS triggered
    uint32_t sos_start_tick;    // When SOS started
    uint32_t sos_blink_pattern; // Current pattern index

    // Lost-line tracking
    uint8_t lost_line_count;   // Counter for consecutive no-sensor readings
#define LOST_LINE_THRESHOLD 50 // ~600 ms at 30 ms update rate

} prog_state_t;

// Standard responses to send back to the Bluetooth App
static const char msg_fwd[] = "Command: Forward\r\n";
static const char msg_rev[] = "Command: Reverse\r\n";
static const char msg_left[] = "Command: Left\r\n";
static const char msg_right[] = "Command: Right\r\n";
static const char msg_stop[] = "Command: STOP\r\n";
static const char msg_line_start[] = "Mode: Line Tracing Started\r\n";
static const char msg_wall_start[] = "Mode: Wall Following Started\r\n";
static const char msg_unknown[] = "Unknown Command\r\n";
static const char msg_ready[] = "\r\n--- Robot Ready ---\r\n";

/*static void sensors_init(void) {
    // Port A: 16, 17, 18 (Left), 19, 22 (Middle), 23 (Right)
    uint32_t pinsA = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19) | (1 << 22) | (1 << 23);
    PORT_REGS->GROUP[0].PORT_DIRCLR = pinsA;

    // Port B: 22, 23 (Right)
    uint32_t pinsB = (1 << 22) | (1 << 23);
    PORT_REGS->GROUP[1].PORT_DIRCLR = pinsB;

    // Enable internal Pull-ups for Port A pins
    for (int i = 16; i <= 23; i++) {
        if (pinsA & (1 << i)) {
            PORT_REGS->GROUP[0].PORT_PINCFG[i] = 0x2; // Pull-enable
            PORT_REGS->GROUP[0].PORT_OUTSET = (1 << i); // Pull-up select
        }
    }
    // Enable internal Pull-ups for Port B pins
    PORT_REGS->GROUP[1].PORT_PINCFG[22] = 0x2;
    PORT_REGS->GROUP[1].PORT_OUTSET = (1 << 22);
    PORT_REGS->GROUP[1].PORT_PINCFG[23] = 0x2;
    PORT_REGS->GROUP[1].PORT_OUTSET = (1 << 23);
} */

static void prog_loop_do_sos(prog_state_t *ps)
{
    if (!ps->sos_active)
        return;

    /* Active buzzer + LED are connected to PA16.
       Keep PA16 HIGH for the whole SOS state. */
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 16);
}

// Trigger SOS
static void prog_trigger_sos(prog_state_t *ps, const char *reason)
{
    ps->sos_active = 1;
    ps->sos_start_tick = platform_systick_count();

    // PA16 HIGH when SOS triggers
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 16);

    // Stop all motors
    platform_motor_stop();

    // Disable autonomous modes
    ps->line_tracing_active = 0;
    ps->wall_following_active = 0;

    // Send emergency message
    ps->response_msg = reason;
    ps->tx_flags |= 0x0001;

    printf("[SOS] %s\n", reason);
}

// Exit SOS (manual override or reset)
static void prog_exit_sos(prog_state_t *ps)
{
    ps->sos_active = 0;
    ps->lost_line_count = 0;

    // PA16 LOW when SOS is cleared
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 16);
}

static void prog_loop_do_line_tracing(prog_state_t *ps)
{
    if (!ps->line_tracing_active)
        return;
    if (ps->turn_active)
        return;

    static uint32_t last_update_tick = 0;
    static uint8_t last_action = 0;
    static uint8_t lost_count = 0;
    // 0 = stop
    // 1 = forward
    // 2 = hard left
    // 3 = soft left
    // 4 = hard right
    // 5 = soft right

    uint32_t now = platform_systick_count();

    // Do not change motor command too fast.
    // PLATFORM_MS_PER_SYSTICK is 10 ms in your init.h.
    // 3 ticks = around 30 ms.
    if (platform_tick_delta(now, last_update_tick) < 3)
    {
        return;
    }

    last_update_tick = now;

    uint16_t ANALOG_LINE_THRESHOLD = 960;

    uint16_t left_1 = platform_adc_read(0); // outer left
    uint16_t left_2 = platform_adc_read(1); // inner left

    uint16_t mid_1 = platform_adc_read(6);
    uint16_t mid_2 = platform_adc_read(7);
    uint16_t mid_3 = platform_adc_read(8);
    // uint16_t mid_4 = platform_adc_read(9);

    uint16_t right_1 = platform_adc_read(10); // inner right
    uint16_t right_2 = platform_adc_read(9);  // outer right

    uint8_t left_outer_see_line = (left_1 > ANALOG_LINE_THRESHOLD);
    uint8_t left_inner_see_line = (left_2 > ANALOG_LINE_THRESHOLD);

    uint8_t right_inner_see_line = (right_1 > ANALOG_LINE_THRESHOLD);
    uint8_t right_outer_see_line = (right_2 > ANALOG_LINE_THRESHOLD);

    uint8_t mid_see_line =
        (mid_1 > ANALOG_LINE_THRESHOLD) ||
        (mid_2 > ANALOG_LINE_THRESHOLD) ||
        (mid_3 > ANALOG_LINE_THRESHOLD);
    // (mid_4 > ANALOG_LINE_THRESHOLD);

    uint8_t left_side_see_line =
        left_outer_see_line || left_inner_see_line;

    uint8_t right_side_see_line =
        right_outer_see_line || right_inner_see_line;

    uint8_t any_sensor_see_line =
        left_outer_see_line ||
        left_inner_see_line ||
        mid_see_line ||
        right_inner_see_line ||
        right_outer_see_line;

    uint8_t action = last_action;

    // ===== LOST TRACK DETECTION =====
    if (!any_sensor_see_line)
    {
        // No sensor sees the line
        ps->lost_line_count++;
        // Brief recovery window: keep last action
        lost_count++;

        if (ps->lost_line_count >= LOST_LINE_THRESHOLD)
        {

            ps->recovery_turn_count = 0;
            ps->recovery_stage = 1;

            if (ps->recovery_direction == 1)
            {

                platform_motor_left();
            }
            else
            {

                platform_motor_right();
            }

            platform_motor_set_pwm(100, 100);

            ps->turn_duration_ticks = (45 * 2) / 5;
            ps->turn_start_tick = platform_systick_count();
            ps->turn_active = 1;

            ps->lost_line_count = 0;

            return;
        }

        if (lost_count < 10)
        {
            action = last_action;
        }
        else
        {
            action = 0; // Stop if still no line
        }
    }
    else if (any_sensor_see_line)
    {

        // Line detected again - reset counter
        ps->lost_line_count = 0;
        lost_count = 0;

        // cancel recovery only if line found again
        ps->recovery_stage = 0;
        ps->turn_active = 0;

        /*
         * Priority:
         * 1. Outer sensors first  -> hard correction
         * 2. Inner sensors next   -> soft correction
         * 3. Middle sensors last  -> forward
         * 4. No sensor sees line  -> briefly continue previous action
         */

        if (left_side_see_line && mid_see_line && right_side_see_line)
        {
            action = 1; // all/most sensors see black
        }

        else if (left_outer_see_line && !right_outer_see_line)
        {
            action = 2; // hard left
            ps->recovery_direction = 1;
        }

        else if (right_outer_see_line && !left_outer_see_line)
        {
            action = 4; // hard right
            ps->recovery_direction = 2;
        }
        else if (left_inner_see_line && !right_inner_see_line)
        {
            action = 3; // soft left
            ps->recovery_direction = 1;
        }
        else if (right_inner_see_line && !left_inner_see_line)
        {
            action = 5; // soft right
            ps->recovery_direction = 2;
        }

        else if (mid_see_line)
        {
            action = 1; // forward
        }
        else
        {
            /*
             * Confusing case:
             * Example: both far left and far right see black.
             * Keep previous action instead of twitching.
             */
            action = last_action;
        }
    }
    last_action = action;

    if (action == 1)
    {
        platform_motor_forward();
        platform_motor_set_pwm(35, 35);
    }
    else if (action == 2)
    { // hard left
        platform_motor_left();
        platform_motor_set_pwm(100, 100); // 70 70
    }
    else if (action == 3)
    { // soft left
        platform_motor_left();
        platform_motor_set_pwm(65, 85); // 40 70
    }
    else if (action == 4)
    { // hard right
        platform_motor_right();
        platform_motor_set_pwm(77, 77); // 70 70
    }
    else if (action == 5)
    { // soft right
        platform_motor_right();
        platform_motor_set_pwm(75, 55); // 75 45
    }
    else
    {
        platform_motor_stop();
    }
}

static void prog_loop_check_turn_timer(prog_state_t *ps)
{
    uint32_t now = platform_systick_count();

    // =====================================
    // TURN TIMER
    // =====================================

    if (ps->turn_active)
    {

        uint32_t elapsed =
            platform_tick_delta(now, ps->turn_start_tick);

        if (elapsed >= ps->turn_duration_ticks)
        {

            platform_motor_stop();

            ps->turn_active = 0;

            ps->recovery_wait_tick = now;
        }
    }

    // =====================================
    // WAIT BETWEEN RECOVERY TURNS
    // =====================================

    if (!ps->turn_active && ps->recovery_stage == 1)
    {

        uint32_t wait_elapsed =
            platform_tick_delta(now, ps->recovery_wait_tick);

        // 1 second wait
        if (wait_elapsed >= 100)
        {

            ps->recovery_turn_count++;

            // =================================
            // DONE 3 TURNS -> SOS
            // =================================

            if (ps->recovery_turn_count >= 3)
            {

                ps->recovery_stage = 0;

                prog_trigger_sos(
                    ps,
                    "Line Lost After Recovery - SOS!\r\n");

                return;
            }

            // =================================
            // CONTINUE SAME DIRECTION TURN
            // =================================

            if (ps->recovery_direction == 1)
            {

                platform_motor_left();
            }
            else
            {

                platform_motor_right();
            }

            platform_motor_set_pwm(100, 100);

            ps->turn_duration_ticks = (45 * 2) / 5;
            ps->turn_start_tick = now;
            ps->turn_active = 1;
        }
    }
}

static void prog_loop_do_wall_following(prog_state_t *ps)
{
    if (!ps->wall_following_active)
        return;

    // Do not interfere during active turns
    if (ps->turn_active)
        return;

    static uint32_t last_update_tick = 0;

    uint32_t now = platform_systick_count();

    // ~30 ms update rate
    if (platform_tick_delta(now, last_update_tick) < 30)
        return;

    last_update_tick = now;

    // =========================
    // Read ultrasonic sensors
    // =========================

    uint32_t front_cm = platform_ultrasonic_read_front_cm();
    uint32_t left_cm = platform_ultrasonic_read_left_cm();
    uint32_t right_cm = platform_ultrasonic_read_right_cm();

    // =========================
    // Thresholds
    // =========================

#define FRONT_WALL_CM 15
#define SIDE_WALL_CM 15

    bool front_blocked = (front_cm > 0 && front_cm < FRONT_WALL_CM);
    bool left_blocked = (left_cm > 0 && left_cm < SIDE_WALL_CM);
    bool right_blocked = (right_cm > 0 && right_cm < SIDE_WALL_CM);

    // =========================
    // FRONT CLEAR
    // =========================

    if (!front_blocked)
    {
        platform_motor_forward();
        platform_motor_set_pwm(45, 45);
        return;
    }

    // =========================
    // FRONT BLOCKED
    // =========================

    // Stop briefly before turning
    platform_motor_stop();

    // =========================
    // PRIORITY:
    // RIGHT > LEFT > TURN AROUND
    // =========================

    // ---------- TURN RIGHT ----------
    if (!right_blocked)
    {
        ps->line_tracing_active = 0;
        ps->wall_following_active = 1;

        platform_motor_right();
        platform_motor_set_pwm(100, 100);

        ps->turn_duration_ticks = (90 * 2) / 5;
        ps->turn_start_tick = platform_systick_count();
        ps->turn_active = 1;

        return;
    }

    // ---------- TURN LEFT ----------
    if (!left_blocked)
    {
        ps->line_tracing_active = 0;
        ps->wall_following_active = 1;

        platform_motor_left();
        platform_motor_set_pwm(100, 100);

        ps->turn_duration_ticks = (90 * 2) / 5;
        ps->turn_start_tick = platform_systick_count();
        ps->turn_active = 1;

        return;
    }

    // ---------- DEAD END ----------
    ps->line_tracing_active = 0;
    ps->wall_following_active = 1;

    platform_motor_right();
    platform_motor_set_pwm(100, 100);

    ps->turn_duration_ticks = (180 * 2) / 5;
    ps->turn_start_tick = platform_systick_count();
    ps->turn_active = 1;
}

// Main-loop task handling data transmission
static void prog_loop_do_one_tx(prog_state_t *ps)
{
    // check if transmission on-going
    if (platform_usart_cdc_tx_busy())
    {
        ps->tx_flags |= 0x8000;
        return;
    }

    // clear busy flag
    ps->tx_flags &= ~0x8000;

    // check if anything to transmit || a message string to send
    if (ps->tx_flags == 0 || ps->response_msg == NULL)
        return;

    if ((ps->tx_flags & 0x0001) != 0)
    {
        ps->tx_desc[0].buf = ps->response_msg;             // load pointer to msg string into transmit descriptor
        ps->tx_desc[0].len = strlen(ps->response_msg);     // calculate len of msg
        platform_usart_cdc_send_async(&ps->tx_desc[0], 1); // usart hardware begins transmitting
        ps->tx_flags = 0x8000;
        ps->response_msg = NULL; // clear after sending
    }
}

// Main-loop task handling data reception
static void prog_loop_do_one_rx(prog_state_t *ps)
{
    // Process the reception-side.
    switch (ps->state_id)
    {
    case 0:
        // IDLE; Start a new reception, if not done so
        if (platform_usart_cdc_recv_async(ps->rx_buf, sizeof(ps->rx_buf)))
        {
            ps->state_id += 1;
        }
        break;
    case 1:
        // Waiting to receive a message
        if (platform_usart_cdc_recv_get(&ps->rx_info))
        {
            ps->state_id += 1;
            // Fall through
        }
        else
        {
            // No message received yet
            break;
        }
    case 2:
    {
        if (ps->rx_info.len > 0)
        {
            char cmd = ps->rx_info.buf[0];
            uint16_t angle = 0;
            uint8_t has_angle = 0;

            // Check if an angle was sent e.g. "A090" or "D180"
            if (ps->rx_info.len >= 4 &&
                (ps->rx_info.buf[1] >= '0' && ps->rx_info.buf[1] <= '9') &&
                (ps->rx_info.buf[2] >= '0' && ps->rx_info.buf[2] <= '9') &&
                (ps->rx_info.buf[3] >= '0' && ps->rx_info.buf[3] <= '9'))
            {
                angle = (ps->rx_info.buf[1] - '0') * 100 + (ps->rx_info.buf[2] - '0') * 10 + (ps->rx_info.buf[3] - '0');

                if (angle > 360)
                    angle = 360;
                if (angle > 0)
                    has_angle = 1;
            }

            if (cmd == 'w' || cmd == 'W')
            {
                ps->line_tracing_active = 0;
                ps->wall_following_active = 0;
                platform_motor_forward();
                platform_motor_set_pwm(100, 100);
                ps->response_msg = msg_fwd;
                ps->turn_active = 0;
            }
            else if (cmd == 's' || cmd == 'S')
            {
                ps->line_tracing_active = 0;
                ps->wall_following_active = 0;
                platform_motor_reverse();
                platform_motor_set_pwm(100, 100);
                ps->response_msg = msg_rev;
                ps->turn_active = 0;
            }
            else if (cmd == 'a' || cmd == 'A')
            {
                ps->line_tracing_active = 0;
                ps->wall_following_active = 0;
                platform_motor_left();
                platform_motor_set_pwm(100, 100);
                ps->response_msg = msg_left;
                if (has_angle)
                {
                    ps->turn_duration_ticks = (angle * 2) / 5;
                    ps->turn_start_tick = platform_systick_count();
                    ps->turn_active = 1;
                }
                else
                {
                    ps->turn_active = 0;
                }
            }
            else if (cmd == 'd' || cmd == 'D')
            {
                ps->line_tracing_active = 0;
                ps->wall_following_active = 0;
                platform_motor_right();
                platform_motor_set_pwm(100, 100);
                ps->response_msg = msg_right;
                if (has_angle)
                {
                    ps->turn_duration_ticks = (angle * 2) / 5;
                    ps->turn_start_tick = platform_systick_count();
                    ps->turn_active = 1;
                }
                else
                {
                    ps->turn_active = 0;
                }
            }

            else if (cmd == 't' || cmd == 'T')
            {
                // Read all three ultrasonic sensors
                uint16_t left_cm = platform_ultrasonic_read_left_cm();
                uint16_t right_cm = platform_ultrasonic_read_right_cm();
                uint16_t front_cm = platform_ultrasonic_read_front_cm();

                sprintf(ps->debug_buf, "Ultrasonic: L=%u R=%u F=%u cm\r\n",
                        left_cm, right_cm, front_cm);

                ps->response_msg = ps->debug_buf;
                ps->tx_flags |= 0x0001;
            }

            else if (cmd == 'x' || cmd == ' ')
            {
                ps->line_tracing_active = 0;
                ps->wall_following_active = 0;
                platform_motor_stop();
                ps->response_msg = msg_stop;
                ps->turn_active = 0;
            }
            else if (cmd == 'l' || cmd == 'L')
            {
                ps->wall_following_active = 0;
                ps->line_tracing_active = 1; // Enter Milestone 4 mode
                ps->response_msg = msg_line_start;
                ps->turn_active = 0;
            }

            else if (cmd == 'u' || cmd == 'U')
            {
                ps->line_tracing_active = 0;
                ps->wall_following_active = 1;
                ps->response_msg = msg_wall_start;
                ps->turn_active = 0;
            }

            else if (cmd == 'c' || cmd == 'C')
            {
                // Calibration mode
                sprintf(ps->debug_buf, "Place object at EXACTLY 10cm from RIGHT sensor\r\n");
                ps->response_msg = ps->debug_buf;
                ps->tx_flags |= 0x0001;

                for (volatile int i = 0; i < 5000000; i++)
                    ; // 5 sec to position

                uint16_t reading = platform_ultrasonic_read_right_cm();

                sprintf(ps->debug_buf, "RIGHT reads: %u cm (should be ~10 cm)\r\n", reading);
                ps->response_msg = ps->debug_buf;
                ps->tx_flags |= 0x0001;

                // If it reads 100, divisor should be 10x smaller (58/10=5.8)
                // If it reads 5, divisor should be 2x larger (58*2=116)

                if (reading < 5)
                {
                    sprintf(ps->debug_buf, "Reading TOO LOW - decrease divisor in formula\r\n");
                }
                else if (reading > 15)
                {
                    sprintf(ps->debug_buf, "Reading TOO HIGH - increase divisor in formula\r\n");
                }
                else
                {
                    sprintf(ps->debug_buf, "Reading OK! Divisor is correct\r\n");
                }
                ps->response_msg = ps->debug_buf;
                ps->tx_flags |= 0x0001;
            }

            else if (cmd == 'p' || cmd == 'P')
            {
                // left sensors
                // uint16_t raw_M1 = platform_adc_read(0);
                uint16_t raw_M2 = platform_adc_read(1);

                // middle sensors
                // uint16_t raw_M3 = platform_adc_read(6);
                uint16_t raw_M4 = platform_adc_read(7);
                // uint16_t raw_M5 = platform_adc_read(8);
                // uint16_t raw_M6 = platform_adc_read(9);

                // right sensors
                uint16_t raw_M7 = platform_adc_read(10);
                // uint16_t raw_M8 = platform_adc_read(11);

                uint16_t ANALOG_LINE_THRESHOLD = 960;

                // convert to 1 (Black) or 0 (White)
                // uint8_t bin_M1 = (raw_M1 > ANALOG_LINE_THRESHOLD) ? 1 : 0;
                uint8_t bin_M2 = (raw_M2 > ANALOG_LINE_THRESHOLD) ? 1 : 0;

                // uint8_t bin_M3 = (raw_M3 > ANALOG_LINE_THRESHOLD) ? 1 : 0;
                uint8_t bin_M4 = (raw_M4 > ANALOG_LINE_THRESHOLD) ? 1 : 0;
                // uint8_t bin_M5 = (raw_M5 > ANALOG_LINE_THRESHOLD) ? 1 : 0;
                // uint8_t bin_M6 = (raw_M6 > ANALOG_LINE_THRESHOLD) ? 1 : 0;

                uint8_t bin_M7 = (raw_M7 > ANALOG_LINE_THRESHOLD) ? 1 : 0;
                // uint8_t bin_M8 = (raw_M8 > ANALOG_LINE_THRESHOLD) ? 1 : 0;

                // format the numbers into a string to see both raw and binary
                sprintf(ps->debug_buf, "Raw L:%u M:%u R:%u | Bin L:%u M:%u R:%u\r\n", raw_M2, raw_M4, raw_M7, bin_M2, bin_M4, bin_M7);

                // point the transmitter to new dynamic string
                ps->response_msg = ps->debug_buf;
                ps->turn_active = 0;
            }

            else if (cmd == 'r' || cmd == 'R')
            {
                if (ps->sos_active)
                {
                    prog_exit_sos(ps);
                    ps->response_msg = "SOS Cleared\r\n";
                }
                else
                {
                    ps->response_msg = "Not in SOS mode\r\n";
                }
                ps->tx_flags |= 0x0001;
            }

            else if (cmd == '\r' || cmd == '\n' || cmd == '\0' || cmd == 0xFF)
            {
                ps->response_msg = NULL;
            }
            else
            {
                ps->response_msg = msg_unknown;
            }

            // Only queue a TX response if there is one to send
            if (ps->response_msg != NULL)
            {
                ps->tx_flags |= 0x0001;
            }
        }
        ps->state_id++;
        break;
    }
    case 3:
        // Only TX response waits here — motors are already acting
        if ((ps->tx_flags & 0x8000) != 0)
            break;
        ps->state_id = 0;
        break;
    }
}

// main() -- the heart of the program
int main(void)
{
    // Variables here
    prog_state_t ps;

    // Early initialization
    memset(&ps, 0, sizeof(ps));
    platform_init_early();

    // Other initialization
    // sensors_init();
    platform_gpio_init();
    platform_usart_cdc_init();
    platform_ADC_init();
    platform_ultrasonic_init();

    // Final initialization
    platform_init_late();

    /*
     * Microcontroller main()'s are supposed to never return (welp, they
     * have none to return to); hence the intentional infinite loop.
     */

    // Send a ready message to the phone upon startup
    ps.response_msg = msg_ready;
    ps.tx_flags = 0x0001;
    for (;;)
    {
        // SOS takes priority
        if (ps.sos_active)
        {
            prog_loop_do_sos(&ps);
        }
        else
        {
            prog_loop_check_turn_timer(&ps);
            prog_loop_do_line_tracing(&ps);
            prog_loop_do_wall_following(&ps);
        }
        prog_loop_do_one_tx(&ps);
        prog_loop_do_one_rx(&ps);
    }

    // The following statement must not be reached
    return 1;
}
