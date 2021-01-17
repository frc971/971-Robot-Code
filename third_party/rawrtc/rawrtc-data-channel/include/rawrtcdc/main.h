#pragma once
#include <rawrtcc/code.h>
#include <re.h>

/**
 * Timer handler.
 *
 * @param on If set to `true`, a timer needs to be scheduled to call
 * rawrtcdc_timer_tick() every `interval` milliseconds.
 * If set to `false`, the timer needs to be cancelled.
 * @param interval the interval for the timer. Should be ignored in
 * case `on` is `false`.
 */
typedef enum rawrtc_code (*rawrtcdc_timer_handler)(bool const on, uint_fast16_t const interval);

/**
 * Initialise RAWRTCDC. Must be called before making a call to any
 * other function.
 *
 * Note: In case you override the default mutex used by re it's vital
 *       that you create a recursive mutex or you will get deadlocks!
 */
enum rawrtc_code rawrtcdc_init(bool const init_re, rawrtcdc_timer_handler const timer_handler);

/**
 * Close RAWRTCDC and free up all resources.
 *
 * Note: In case `close_re` is not set to `true`, you MUST close
 *       re yourselves.
 */
enum rawrtc_code rawrtcdc_close(bool const close_re);

/**
 * Handle timer tick.
 * `delta` contains the delta milliseconds passed between calls.
 */
void rawrtcdc_timer_tick(uint_fast16_t const delta);
