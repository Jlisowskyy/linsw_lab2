#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include "gpio.h"

// ------------------------------
// defines
// ------------------------------

#define NUM_BUTTONS 4
#define NUM_LEDS 4
#define NUM_ARGS 2
#define GPIO_SYS_PATH "/dev/gpiochip0"

#define PRESENTATION_SHINE_RETRIES 12
#define PRESENTATION_SHINE_TIME_MS 100
#define PRESENTATION_SHINE_BLANK_TIME_MS 150
#define PRESENTATION_BLANK_LEDS_MS 300
#define PRESENTATION_BIT_TIME_MS 2000

#define CHECKED_RUN(run) if ((run) < 0) { \
    TRACE("Error running %s!", #run); \
    CleanUp(); \
    exit(EXIT_FAILURE); \
}

const int kButtonPins[NUM_BUTTONS] = {
    25, 10, 17, 18
};

const int kLedPins[NUM_LEDS] = {
    24, 22, 23, 27
};

typedef enum CalculatorPhase {
    ARG_INPUT_FIRST = 0,
    ARG_INPUT_SECOND,
    ARG_INPUT_OPERATION,
    ARG_DISPLAY,
    LAST_PHASE
} calculator_phase_t;

typedef enum Operation {
    ADDITION = 0,
    SUBTRACTION,
    MULTIPLICATION,
    DIVISION,
    LAST_OPERATION
} operation_t;

/* returns next state for poll function */
typedef bool (*button_callback_t)(void);

typedef struct IoState {
    gpio_t *buttons[NUM_BUTTONS];
    gpio_t *leds[NUM_LEDS];

    struct pollfd fds[NUM_BUTTONS];
    button_callback_t callbacks[NUM_BUTTONS];
} io_state_t;

typedef struct Args {
    uint64_t args[NUM_ARGS];
    size_t cur_arg;
    size_t arg_bit_idx;
} args_t;

typedef struct AppState {
    calculator_phase_t phase;
    bool should_run;
    io_state_t io;
    args_t args;
    operation_t operation;
} app_state_t;

// ------------------------------
// Macros
// ------------------------------

#define ENABLE_OUTPUT
#ifdef ENABLE_OUTPUT
#define TRACE(...) printf(__VA_ARGS__)
#else
#define TRACE(...) ((void)0)
#endif // ENENABLE_OUTPUT

// ------------------------------
// Global state
// ------------------------------

static app_state_t app_state = {
    .phase = ARG_INPUT_FIRST,
    .should_run = true,
    .io = {},
    .args = {},
    .operation = ADDITION,
};

// ------------------------------
// Function definitions
// ------------------------------

static void InitializeButtons();

static void InitializeLeds();

static void CleanupButtons();

static void CleanupLeds();

static void CleanUp();

static void RunStateMachine();

static calculator_phase_t ProcessArgInputState(int arg_num);

static calculator_phase_t ProcessOpInputState();

static calculator_phase_t ProcessDisplayInputState();

static void PollButtons();

static void SetLedState(size_t led_num, int state);

static void DisableLed(size_t led_num);

static void EnableLed(size_t led_num);

static bool ArgInputButton0Callback();

static bool ArgInputButton1Callback();

static bool ArgInputButton2Callback();

static bool ArgInputButton3Callback();

static bool OpInputButton0Callback();

static bool OpInputButton1Callback();

static uint64_t Calculate();

static void ShineLeds();

static void Signal0Bit();

static void Signal1Bit();

static void DisableAllLeds();

static void EnableAllLeds();

static void DisplayLast4Bits();

static void DisplayOperation();

// ------------------------------
// Test functions
// ------------------------------

static bool TestPollCallback() {
    TRACE("Button pressed!");
    return true;
}

static void TestPoll() {
    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        app_state.io.callbacks[i] = TestPollCallback;
    }

    PollButtons();
}

// ------------------------------
// Function implementations
// ------------------------------

void InitializeButtons() {
    TRACE("Initializing buttons...\n");

    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        app_state.io.buttons[i] = gpio_new();

        if (gpio_open(app_state.io.buttons[i], GPIO_SYS_PATH, kButtonPins[i], GPIO_DIR_IN) < 0) {
            TRACE("Failed to open button (idx: %lu) %d: %s!\n", i, kButtonPins[i],
                  gpio_errmsg(app_state.io.buttons[i]));

            for (int j = 0; j < i; j++) {
                gpio_close(app_state.io.buttons[j]);
                gpio_free(app_state.io.buttons[j]);
            }

            exit(EXIT_FAILURE);
        }

        app_state.io.fds[i].fd = gpio_fd(app_state.io.buttons[i]);
        app_state.io.fds[i].events = POLLIN | POLLPRI;
    }

    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        if (gpio_set_edge(app_state.io.buttons[i], GPIO_EDGE_BOTH) < 0) {
            TRACE("Failed to configure edge (idx: %lu) %d: %s!\n", i, kButtonPins[i],
                  gpio_errmsg(app_state.io.buttons[i]));

            CleanupButtons();
            exit(EXIT_FAILURE);
        }
    }

    TRACE("Correctly initialized buttons!\n");
}

void InitializeLeds() {
    TRACE("Initializing leds...\n");

    for (size_t i = 0; i < NUM_LEDS; i++) {
        app_state.io.leds[i] = gpio_new();

        if (gpio_open(app_state.io.leds[i], GPIO_SYS_PATH, kLedPins[i], GPIO_DIR_OUT) < 0) {
            TRACE("Error initializing LED on pin %d: %s\n", kLedPins[i], gpio_errmsg(app_state.io.leds[i]));

            CleanupButtons();

            for (size_t j = 0; j < i; j++) {
                gpio_close(app_state.io.leds[j]);
                gpio_free(app_state.io.leds[j]);
            }

            exit(EXIT_FAILURE);
        }
    }

    DisableAllLeds();

    TRACE("Leds initialized!\n");
}

void CleanupButtons() {
    TRACE("Cleaning up buttons...\n");
    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        gpio_close(app_state.io.buttons[i]);
        gpio_free(app_state.io.buttons[i]);
    }
    TRACE("Buttons closed!\n");
}

void CleanupLeds() {
    TRACE("Cleaning up leds...\n");

    for (size_t i = 0; i < NUM_LEDS; i++) {
        gpio_close(app_state.io.leds[i]);
        gpio_free(app_state.io.leds[i]);
    }

    TRACE("Leds closed!\n");
}

void CleanUp() {
    CleanupButtons();
    CleanupLeds();
}

static void RunStateMachine() {
    while (app_state.should_run) {
        switch (app_state.phase) {
            case ARG_INPUT_FIRST:
                TRACE("Entering ARG_INPUT_FIRST state\n");
                app_state.phase = ProcessArgInputState(0);
                break;
            case ARG_INPUT_SECOND:
                TRACE("Entering ARG_INPUT_SECOND state\n");
                app_state.phase = ProcessArgInputState(1);
                break;
            case ARG_INPUT_OPERATION:
                TRACE("Entering ARG_INPUT_OPERATION state\n");
                app_state.phase = ProcessOpInputState();
                break;
            case ARG_DISPLAY:
                TRACE("Entering ARG_DISPLAY state\n");
                app_state.phase = ProcessDisplayInputState();
                break;
            case LAST_PHASE:
                TRACE("Reached last phase. Restarting calculation!\n");
                app_state.phase = ARG_INPUT_FIRST;
                break;
        }
    }
}

calculator_phase_t ProcessArgInputState(const int arg_num) {
    app_state.args.cur_arg = (size_t) arg_num;
    app_state.args.arg_bit_idx = 0;
    app_state.args.args[arg_num] = 0;
    DisableAllLeds();

    app_state.io.callbacks[0] = ArgInputButton0Callback;
    app_state.io.callbacks[1] = ArgInputButton1Callback;
    app_state.io.callbacks[2] = ArgInputButton2Callback;
    app_state.io.callbacks[3] = ArgInputButton3Callback;

    PollButtons();

    return arg_num == 0 ? ARG_INPUT_SECOND : ARG_INPUT_OPERATION;
}

calculator_phase_t ProcessOpInputState() {
    app_state.operation = ADDITION;
    DisableAllLeds();

    app_state.io.callbacks[0] = OpInputButton0Callback;
    app_state.io.callbacks[1] = OpInputButton1Callback;
    app_state.io.callbacks[2] = NULL;
    app_state.io.callbacks[3] = NULL;

    PollButtons();

    return ARG_DISPLAY;
}

calculator_phase_t ProcessDisplayInputState() {
    const uint64_t result = Calculate();

    ShineLeds();

    // TODO: end on last 1
    for (size_t cur = 0; cur < 64; cur++) {
        const uint64_t bit = result & ((uint64_t) 1 << cur);

        if (bit) {
            Signal1Bit();
        } else {
            Signal0Bit();
        }

        CHECKED_RUN(usleep(PRESENTATION_BLANK_LEDS_MS * 1000));
    }

    ShineLeds();

    return LAST_PHASE;
}

void PollButtons() {
    bool should_poll = true;

    while (should_poll) {
        int ret = poll(app_state.io.fds, NUM_BUTTONS, -1);

        if (ret < 0) {
            TRACE("Polling failed!\n");
            CleanUp();
            exit(EXIT_FAILURE);
        }

        for (size_t i = 0; i < NUM_BUTTONS; i++) {
            if (app_state.io.fds[i].revents & (POLLIN | POLLPRI)) {
                gpio_edge_t event;
                if (gpio_read_event(app_state.io.buttons[i], &event, NULL) < 0) {
                    TRACE("Error reading event from button_%lu: %s\n", i, gpio_errmsg(app_state.io.buttons[i]));

                    CleanUp();
                    exit(EXIT_FAILURE);
                }

                const bool button_pressed = event == GPIO_EDGE_FALLING ? true : false;

                if (button_pressed && app_state.io.callbacks[i] != NULL) {
                    should_poll = app_state.io.callbacks[i]();
                }
            }
        }
    }
}

void SetLedState(const size_t led_num, const int state) {
    if (gpio_write(app_state.io.leds[led_num], state) < 0) {
        TRACE("Error enabling LED: %s\n", gpio_errmsg(app_state.io.leds[led_num]));

        CleanUp();
        exit(EXIT_FAILURE);
    }
}

void DisableLed(const size_t led_num) {
    SetLedState(led_num, 0);
}

void EnableLed(const size_t led_num) {
    SetLedState(led_num, 1);
}

bool ArgInputButton0Callback() {
    /* Move to next phase */
    return false;
}

bool ArgInputButton1Callback() {
    /* Add 0 bit - simply move cursor */
    if (app_state.args.arg_bit_idx < 64) {
        app_state.args.arg_bit_idx++;
    }

    DisplayLast4Bits();

    return true;
}

bool ArgInputButton2Callback() {
    /* Add 1 bit */
    if (app_state.args.arg_bit_idx < 64) {
        app_state.args.args[app_state.args.cur_arg] |= ((uint64_t) 1 << app_state.args.arg_bit_idx);
        app_state.args.arg_bit_idx++;
    }

    DisplayLast4Bits();

    return true;
}

bool ArgInputButton3Callback() {
    /* remove last added bit */
    if (app_state.args.arg_bit_idx > 0) {
        app_state.args.arg_bit_idx--;

        const uint64_t mask = ~((uint64_t) 1 << app_state.args.arg_bit_idx);
        app_state.args.args[app_state.args.cur_arg] &= mask;
    }

    DisplayLast4Bits();

    return true;
}

bool OpInputButton0Callback() {
    /* Move to next step */
    return false;
}

bool OpInputButton1Callback() {
    /* Pick next operation */
    app_state.operation++;
    app_state.operation %= LAST_OPERATION;

    DisplayOperation();

    return true;
}

uint64_t Calculate() {
    switch (app_state.operation) {
        case ADDITION:
            return app_state.args.args[0] + app_state.args.args[1];
        case SUBTRACTION:
            return app_state.args.args[0] - app_state.args.args[1];
        case MULTIPLICATION:
            return app_state.args.args[0] * app_state.args.args[1];
        case DIVISION:
            if (app_state.args.args[1] == 0) {
                TRACE("Division by zero!\n");
                return 0;
            }
            return app_state.args.args[0] / app_state.args.args[1];
        case LAST_OPERATION:
            CleanUp();
            exit(EXIT_FAILURE);
    }

    CleanUp();
    exit(EXIT_FAILURE);
}

void ShineLeds() {
    for (size_t i = 0; i < PRESENTATION_SHINE_RETRIES; i++) {
        EnableAllLeds();
        CHECKED_RUN(usleep(PRESENTATION_SHINE_TIME_MS * 1000));

        DisableAllLeds();
        CHECKED_RUN(usleep(PRESENTATION_SHINE_BLANK_TIME_MS * 1000));
    }
}

void Signal0Bit() {
    DisableAllLeds();

    EnableLed(2);
    EnableLed(3);

    CHECKED_RUN(usleep(PRESENTATION_BIT_TIME_MS * 1000));

    DisableAllLeds();
}

void Signal1Bit() {
    DisableAllLeds();

    EnableLed(0);
    EnableLed(1);

    CHECKED_RUN(usleep(PRESENTATION_BIT_TIME_MS * 1000));

    DisableAllLeds();
}

void DisableAllLeds() {
    for (size_t i = 0; i < NUM_LEDS; i++) {
        DisableLed(i);
    }
}

void EnableAllLeds() {
    for (size_t i = 0; i < NUM_LEDS; i++) {
        EnableLed(i);
    }
}

void DisplayLast4Bits() {
    /* get bit mask */
    const uint64_t base_mask = 0b1111;
    const uint64_t shift = app_state.args.arg_bit_idx > 4 ? (app_state.args.arg_bit_idx - 4) : 0;
    const uint64_t adjusted_mask = base_mask << shift;
    const uint64_t shifted_bits = app_state.args.args[app_state.args.cur_arg] & adjusted_mask;
    const uint64_t bits = shifted_bits >> shift;

    SetLedState(0, (bits & 0b1000) > 0);
    SetLedState(1, (bits & 0b0100) > 0);
    SetLedState(2, (bits & 0b0010) > 0);
    SetLedState(3, (bits & 0b0001) > 0);
}

void DisplayOperation() {
    const uint64_t bits = (uint64_t) app_state.operation;

    SetLedState(0, (bits & 0b1000) > 0);
    SetLedState(1, (bits & 0b0100) > 0);
    SetLedState(2, (bits & 0b0010) > 0);
    SetLedState(3, (bits & 0b0001) > 0);
}

// ------------------------------
// Entry point
// ------------------------------

int main(void) {
    TRACE("Welcome to binary calculator project for linsw - lab2!\n");
    InitializeButtons();
    InitializeLeds();
    RunStateMachine();
    TRACE("Goodbye, that was a good time...\n");

    CleanUp();

    return 0;
}
