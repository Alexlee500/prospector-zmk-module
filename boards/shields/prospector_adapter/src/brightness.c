#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/printk.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/split_central_status_changed.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(als, 4);

static const struct device *pwm_leds_dev = DEVICE_DT_GET_ONE(pwm_leds);
#define DISP_BL DT_NODE_CHILD_IDX(DT_NODELABEL(disp_bl))

static uint8_t current_brightness = 100;

#if CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0

#define SCREEN_IDLE_TIMEOUT_MS (CONFIG_PROSPECTOR_IDLE_TIMEOUT_S * 1000)

static int64_t last_activity = 0;

static void apply_brightness(uint8_t value)
{
    led_set_brightness(pwm_leds_dev, DISP_BL, value);
    LOG_INF("Screen brightness set to %d", value);
}

// Threaded fade logic
// Contains starting and target brightness levels to be animated
struct fade_request_t
{
    uint8_t from; // Starting brightness level
    uint8_t to;   // Target brightness level
};

#define FADE_QUEUE_SIZE 4

// Message queue used to send fade requests to the fade handler thread.
// It holds up to 4 fade_request_t elements and ensures brightness updates are handled sequentially.
K_MSGQ_DEFINE(fade_msgq, sizeof(struct fade_request_t), FADE_QUEUE_SIZE, 4);

// Cubic ease-in-out function to smooth the interpolation curve.
// Provides a natural "S-curve" animation effect: starts slow, accelerates, then slows again.
// Helps avoid abrupt changes in perceived brightness.
static float ease_in_out(float t)
{
    if (t < 0.5f)
        return 4.0f * t * t * t;
    float f = -2.0f * t + 2.0f;
    return 1.0f - (f * f * f) / 2.0f;
}

// Dedicated thread responsible for handling all fade animations.
// Receives fade requests from the queue and applies brightness changes over time using easing.
void fade_thread(void)
{
    struct fade_request_t req;

    while (1)
    {
        // Wait indefinitely for the next fade request to arrive in the queue
        if (k_msgq_get(&fade_msgq, &req, K_FOREVER) == 0)
        {

            // Skip animation entirely if brightness difference is too small
            if (req.from == req.to || abs(req.to - req.from) <= 1)
            {
                apply_brightness(req.to);
                continue;
            }

            // Calculate brightness difference and use it to determine number of steps
            int diff = abs(req.to - req.from);
            int steps = CLAMP(diff * 2, 6, 32); // More steps for smoother fades over large differences

            // Set total animation time: scale with difference but clamp between 500ms and 1000ms
            int total_duration_ms = CLAMP(diff * 20, 500, 1000); // 20ms per level as baseline
            int delay_us = (total_duration_ms * 1000) / steps;   // Delay between steps in microseconds

            uint8_t last_applied = 255; // Used to prevent redundant LED updates to save performance

            // Interpolate brightness across 'steps' frames using easing
            for (int i = 0; i <= steps; i++)
            {
                float t = (float)i / steps;                                  // Normalized time in [0, 1]
                float eased = ease_in_out(t);                                // Eased time for smoother progression
                float interpolated = req.from + (req.to - req.from) * eased; // Interpolated value
                uint8_t brightness = (uint8_t)(interpolated + 0.5f);         // Rounded to nearest integer

                // Only send update if brightness actually changed
                if (brightness != last_applied)
                {
                    apply_brightness(brightness);
                    last_applied = brightness;
                }

                k_usleep(delay_us); // Sleep before next step to pace the fade
            }

            // safeguard to ensure the target value is set at the end
            if (last_applied != req.to)
            {
                apply_brightness(req.to);
            }
        }
    }
}

// Launch the fade thread with 768 bytes of stack, medium priority (6)
// 512 was too small for logging, math (float, int), small loop, few stack-local variables
// 768 is just a guess, optimization is possible, probably
K_THREAD_DEFINE(fade_tid, 768, fade_thread, NULL, NULL, NULL, 6, 0, 0);

// Function to submit a brightness fade request
// Ensures that only the most recent fade request is applied by purging the queue first for changes in between animations
static void fade_to_brightness(uint8_t from, uint8_t to)
{
    struct fade_request_t req = {.from = from, .to = to};
    k_msgq_purge(&fade_msgq);                // Clear any pending fades to avoid outdated transitions
    k_msgq_put(&fade_msgq, &req, K_NO_WAIT); // Submit the new fade request without blocking
}

// --- Brightness logic ---
static bool screen_on = true;
// --- Screen on/off ---

static void screen_set_on(bool on)
{
    if (on && !screen_on)
    {
        fade_to_brightness(0, current_brightness);
        screen_on = true;
        LOG_INF("Screen on (smooth)");
    }
    else if (!on && screen_on)
    {
        fade_to_brightness(current_brightness, 0);
        screen_on = false;
        LOG_INF("Screen off (smooth)");
    }
    else
    {
        LOG_DBG("Screen state is already %s, no action taken.", on ? "on" : "off");
    }
}

// --- Idle thread ---

void screen_idle_thread(void)
{
    while (1)
    {
        if (screen_on)
        {
            int64_t now = k_uptime_get();
            int64_t elapsed = now - last_activity;
            int64_t remaining = SCREEN_IDLE_TIMEOUT_MS - elapsed;

            if (remaining <= 0)
            {
                screen_set_on(false);
                // After turning off, sleep until next activity (key event will wake screen)
                k_sleep(K_FOREVER);
            }
            else
            {
                // Sleep exactly as long as needed until timeout or next key event
                k_sleep(K_MSEC(remaining));
            }
        }
        else
        {
            // If Screen is off, sleep forever (will be interrupted by key event)
            k_sleep(K_FOREVER);
        }
    }
}

K_THREAD_DEFINE(screen_idle_tid, 512, screen_idle_thread, NULL, NULL, NULL, 7, 0, 0);

void brightness_wake_screen_on_reconnect(void)
{
    if (!screen_on)
    {
        LOG_INF("Peripheral reconnected, waking screen");

        screen_set_on(true);

        // Reset idle timer
        last_activity = k_uptime_get();

        k_wakeup(screen_idle_tid);
    }
    else
    {
        LOG_DBG("Peripheral reconnected but screen already on");
    }
}

// --- Key event listener ---

static int key_listener(const zmk_event_t *eh)
{
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev && ev->state)
    { // Only on key down
        LOG_DBG("Key pressed: keycode=%d", ev->keycode);
    }

    last_activity = k_uptime_get();
    if (!screen_on)
    {
        screen_set_on(true);
        k_wakeup(screen_idle_tid);
    }
    return 0;
}

ZMK_LISTENER(screen_idle, key_listener);
ZMK_SUBSCRIPTION(screen_idle, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(screen_idle, zmk_layer_state_changed);

static int reconnect_listener(const zmk_event_t *eh)
{
    const struct zmk_split_central_status_changed *ev = as_zmk_split_central_status_changed(eh);
    if (ev && ev->connected)
    {
        brightness_wake_screen_on_reconnect();
    }
    return 0;
}

ZMK_LISTENER(screen_reconnect, reconnect_listener);
ZMK_SUBSCRIPTION(screen_reconnect, zmk_split_central_status_changed);

static int init_screen_idle(void)
{
    last_activity = k_uptime_get();
    // Wake up the idle thread at boot
    k_wakeup(screen_idle_tid);
    return 0;
}

SYS_INIT(init_screen_idle, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif // CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0

#ifdef CONFIG_PROSPECTOR_USE_AMBIENT_LIGHT_SENSOR

#define SENSOR_MIN      0       // Minimum sensor reading
#define SENSOR_MAX      100   // Maximum sensor reading
#define PWM_MIN         1       // Minimum PWM duty cycle (%) - keep display visible
#define PWM_MAX         100     // Maximum PWM duty cycle (%)

#define FADE_STEP                        1
#define FADE_SLEEP_BRIGHTEN_MS           3
#define FADE_SLEEP_DARKEN_MS             10
#define FADE_THRESHOLD                   10

#define NORMAL_SAMPLE_SLEEP_MS           100

#define BURST_SAMPLE_SLEEP_MS            30
#define BURST_SAMPLE_TIMEOUT             10
#define BURST_SAMPLE_CONSECUTIVE         3

uint8_t map_light_to_pwm(int32_t sensor_reading) {
    // Handle invalid/error readings
    if (sensor_reading < SENSOR_MIN) {
        return PWM_MIN;  // Default to minimum brightness on error
    }

    // Clamp to maximum
    if (sensor_reading > SENSOR_MAX) {
        sensor_reading = SENSOR_MAX;
    }

    // Linear mapping
    uint8_t pwm_value = (uint8_t)(
        PWM_MIN + ((PWM_MAX - PWM_MIN) *
        (sensor_reading - SENSOR_MIN)) / (SENSOR_MAX - SENSOR_MIN)
    );

    return pwm_value;
}

uint8_t bl_fade(uint8_t source, uint8_t target) {
    bool increasing = target > source;

    while ((increasing && current_brightness < target) ||
           (!increasing && current_brightness > target)) {

        if (led_set_brightness(pwm_leds_dev, DISP_BL, current_brightness)) {
            LOG_ERR("Failed to set brightness");
        }

        current_brightness += increasing ? FADE_STEP : -FADE_STEP;

        // Ensure we don't overshoot bounds
        if (current_brightness > 100) {
            current_brightness = 100;
        } else if (current_brightness < 0) {
            current_brightness = 0;
        }

        k_msleep(increasing ? FADE_SLEEP_BRIGHTEN_MS : FADE_SLEEP_DARKEN_MS);
    }

    return 0;
}

extern void als_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    const struct device *dev;
    struct sensor_value intensity;
    uint8_t mapped_brightness;

    dev = DEVICE_DT_GET_ONE(avago_apds9960);
    if (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
    }

    // led_set_brightness(pwm_leds_dev, DISP_BL, 100);

    while (1) {

        k_msleep(NORMAL_SAMPLE_SLEEP_MS);


        if (sensor_sample_fetch(dev)) {
            LOG_ERR("sensor_sample fetch failed\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
            LOG_ERR("Cannot read ALS data.\n");
        }

        // LOG_INF("ambient light intensity %d", intensity.val1);

        mapped_brightness = map_light_to_pwm(intensity.val1);
        // LOG_INF("NORMAL: mapped PWM duty cycle %d\n", mapped_brightness);

        if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
            uint8_t integrator = 0;

            for (int i = 0; i < BURST_SAMPLE_TIMEOUT; i++) {
                k_msleep(BURST_SAMPLE_SLEEP_MS);

                if (sensor_sample_fetch(dev)) {
                    LOG_ERR("sensor_sample fetch failed\n");
                }
                if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
                    LOG_ERR("Cannot read ALS data.\n");
                }

                mapped_brightness = map_light_to_pwm(intensity.val1);
                // LOG_INF("BURST: mapped PWM duty cycle %d\n", mapped_brightness);

                if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
                    integrator++;
                    // printk("integrator at: %d", integrator);
                    if (integrator >= BURST_SAMPLE_CONSECUTIVE) {
#if CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0
                        if (screen_on)
#endif
                        bl_fade(current_brightness, mapped_brightness);
                        current_brightness = mapped_brightness;
                        // LOG_INF("SETTING NEW BRIGHTNESS: %d", mapped_brightness);
                        break;
                    }
                }
            }
        }
        // led_set_brightness(pwm_leds_dev, DISP_BL, map_light_to_pwm(intensity.val1));
    }
}

K_THREAD_DEFINE(als_tid, 1024, als_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
                0);

#else

static int init_fixed_brightness(void) {
    led_set_brightness(pwm_leds_dev, DISP_BL, CONFIG_PROSPECTOR_FIXED_BRIGHTNESS);
    current_brightness = CONFIG_PROSPECTOR_FIXED_BRIGHTNESS;

    return 0;
}

SYS_INIT(init_fixed_brightness, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif