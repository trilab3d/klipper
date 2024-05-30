// Commands for controlling GPIO analog-to-digital input pins
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_adc
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "trsync.h" // trsync_do_trigger

struct analog_in {
    struct timer timer;
    uint32_t rest_time, sample_time, next_begin_time;
    uint16_t value, min_value, max_value;
    struct gpio_adc pin;
    uint8_t invalid_count, range_check_count;
    uint8_t state, sample_count;
};

static struct task_wake analog_wake;

static uint_fast8_t
analog_in_event(struct timer *timer)
{
    struct analog_in *a = container_of(timer, struct analog_in, timer);
    uint32_t sample_delay = gpio_adc_sample(a->pin);
    if (sample_delay) {
        a->timer.waketime += sample_delay;
        return SF_RESCHEDULE;
    }
    uint16_t value = gpio_adc_read(a->pin);
    uint8_t state = a->state;
    if (state >= a->sample_count) {
        state = 0;
    } else {
        value += a->value;
    }
    a->value = value;
    a->state = state+1;
    if (a->state < a->sample_count) {
        a->timer.waketime += a->sample_time;
        return SF_RESCHEDULE;
    }
    if (likely(a->value >= a->min_value && a->value <= a->max_value)) {
        a->invalid_count = 0;
    } else {
        a->invalid_count++;
        if (a->invalid_count >= a->range_check_count) {
            try_shutdown("ADC out of range");
            a->invalid_count = 0;
        }
    }
    sched_wake_task(&analog_wake);
    a->next_begin_time += a->rest_time;
    a->timer.waketime = a->next_begin_time;
    return SF_RESCHEDULE;
}

void
command_config_analog_in(uint32_t *args)
{
    struct gpio_adc pin = gpio_adc_setup(args[1]);
    struct analog_in *a = oid_alloc(
        args[0], command_config_analog_in, sizeof(*a));
    a->timer.func = analog_in_event;
    a->pin = pin;
    a->state = 1;
}
DECL_COMMAND(command_config_analog_in, "config_analog_in oid=%c pin=%u");

void
command_query_analog_in(uint32_t *args)
{
    struct analog_in *a = oid_lookup(args[0], command_config_analog_in);
    sched_del_timer(&a->timer);
    gpio_adc_cancel_sample(a->pin);
    a->next_begin_time = args[1];
    a->timer.waketime = a->next_begin_time;
    a->sample_time = args[2];
    a->sample_count = args[3];
    a->state = a->sample_count + 1;
    a->rest_time = args[4];
    a->min_value = args[5];
    a->max_value = args[6];
    a->range_check_count = args[7];
    if (! a->sample_count)
        return;
    sched_add_timer(&a->timer);
}
DECL_COMMAND(command_query_analog_in,
             "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c"
             " rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c");

void
analog_in_task(void)
{
    if (!sched_check_wake(&analog_wake))
        return;
    uint8_t oid;
    struct analog_in *a;
    foreach_oid(oid, a, command_config_analog_in) {
        if (a->state != a->sample_count)
            continue;
        irq_disable();
        if (a->state != a->sample_count) {
            irq_enable();
            continue;
        }
        uint16_t value = a->value;
        uint32_t next_begin_time = a->next_begin_time;
        a->state++;
        irq_enable();
        sendf("analog_in_state oid=%c next_clock=%u value=%hu"
              , oid, next_begin_time, value);
    }
}
DECL_TASK(analog_in_task);

void
analog_in_shutdown(void)
{
    uint8_t i;
    struct analog_in *a;
    foreach_oid(i, a, command_config_analog_in) {
        gpio_adc_cancel_sample(a->pin);
        if (a->sample_count) {
            a->state = a->sample_count + 1;
            a->next_begin_time += a->rest_time;
            a->timer.waketime = a->next_begin_time;
            sched_add_timer(&a->timer);
        }
    }
}
DECL_SHUTDOWN(analog_in_shutdown);

// Analog Endstop ---------------

#define BUFFER_SIZE 128

struct moving_average {
    uint16_t buffer[BUFFER_SIZE];
    int index;
    int count;
    uint32_t sum;
};

struct analog_endstop {
    struct timer timer;
    uint32_t rest_time, sample_time, nextwake;
    uint16_t value, treshold;
    struct gpio_adc pin;
    struct trsync *ts;
    uint8_t oversample_count, trigger_reason;
    struct moving_average ma;
};

void moving_average_init(struct moving_average *ma) {
    ma->index = 0;
    ma->count = 0;
    ma->sum = 0;
}

uint16_t moving_average_add_value(struct moving_average *ma, uint16_t value) {
    // If the buffer is full, subtract the oldest value from the sum
    if (ma->count == BUFFER_SIZE) {
        ma->sum -= ma->buffer[ma->index];
    } else {
        ma->count++;
    }

    // Add the new value to the buffer and sum
    ma->buffer[ma->index] = value;
    ma->sum += value;

    // Update the index
    ma->index = (ma->index + 1) % BUFFER_SIZE;

    if (ma->count == BUFFER_SIZE) {
        // Return the moving average
        // return (uint16_t)(ma->sum / ma->count);
        // Return the moving average using bit shift for division
        return (uint16_t)(ma->sum >> 7);  // Equivalent to ma->sum / 128 if BUFFER_SIZE is 128
    } else {
        return 0;
    }
}

static uint_fast8_t analog_endstop_oversample_event(struct timer *t);

// Timer callback for an analog end stop
static uint_fast8_t
analog_endstop_event(struct timer *t)
{
    struct analog_endstop *a = container_of(t, struct analog_endstop, timer);
    uint32_t sample_delay = gpio_adc_sample(a->pin);
    if (sample_delay) {
        a->timer.waketime += sample_delay;
        return SF_RESCHEDULE;
    }
    uint16_t value = gpio_adc_read(a->pin);

    moving_average_add_value(&a->ma, value);

    uint32_t nextwake = a->timer.waketime + a->rest_time;
    if (value < a->treshold) {
        // No match - reschedule for the next attempt
        a->timer.waketime = nextwake;
        return SF_RESCHEDULE;
    }
    a->nextwake = nextwake;
    a->timer.func = analog_endstop_oversample_event;
    return analog_endstop_oversample_event(t);
}

// Timer callback for an analog end stop that is sampling extra times
static uint_fast8_t
analog_endstop_oversample_event(struct timer *t)
{
    struct analog_endstop *a = container_of(t, struct analog_endstop, timer);
    uint32_t sample_delay = gpio_adc_sample(a->pin);
    if (sample_delay) {
        a->timer.waketime += sample_delay;
        return SF_RESCHEDULE;
    }
    uint16_t value = moving_average_add_value(&a->ma, gpio_adc_read(a->pin));

    if (value > a->treshold) {
        trsync_do_trigger(a->ts, a->trigger_reason);
        return SF_DONE;
    }

    a->timer.waketime += a->sample_time;
    return SF_RESCHEDULE;
}

void
command_config_analog_endstop(uint32_t *args)
{
    struct gpio_adc pin = gpio_adc_setup(args[1]);
    struct analog_endstop *a = oid_alloc(
        args[0], command_config_analog_endstop, sizeof(*a));
    a->timer.func = analog_endstop_event;
    a->pin = pin;
}
DECL_COMMAND(command_config_analog_endstop,
    "config_analog_endstop oid=%c pin=%u");

void
command_analog_endstop_home(uint32_t *args)
{
    struct analog_endstop *e = oid_lookup(args[0],
        command_config_analog_endstop);
    sched_del_timer(&e->timer);
    e->timer.waketime = args[1];
    e->sample_time = args[2];
    e->oversample_count = args[3];
    if (! e->oversample_count){
        // disable endstop checking
        e->ts = NULL;
        return;
    }
    e->rest_time = args[4];
    e->timer.func = analog_endstop_event;
    e->treshold = args[5];
    e->ts = trsync_oid_lookup(args[6]);
    e->trigger_reason = args[7];
    moving_average_init(&e->ma);
    sched_add_timer(&e->timer);
}
DECL_COMMAND(command_analog_endstop_home,
             "analog_endstop_home oid=%c clock=%u sample_ticks=%u "
             "oversample_count=%c rest_ticks=%u treshold=%u trsync_oid=%c "
             "trigger_reason=%c");

void
command_analog_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct analog_endstop *e = oid_lookup(oid, command_config_analog_endstop);

    //irq_disable(); - no need for sync within single statement, I think...
    uint32_t nextwake = e->nextwake;
    //irq_enable();

    //wait for ADC.
    uint32_t sample_delay = 0;
    do {
        sample_delay = gpio_adc_sample(e->pin);
    } while (sample_delay);

    sendf("analog_endstop_state oid=%c next_clock=%u pin_value=%u treshold=%u"
          , oid, nextwake, gpio_adc_read(e->pin), e->treshold);
}
DECL_COMMAND(command_analog_endstop_query_state,
    "analog_endstop_query_state oid=%c");
