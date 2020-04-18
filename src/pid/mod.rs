//! Software PID controller
//!
//! This crate implements a PID controller. It has seen some amount of
//! real-world usage driving 100+ kW electrical motors, but has not been tested
//! to death. Use with caution (but do use it and file bug reports!).
//!
//! Any change in behaviour that may make calculations behave differently will
//! result in a major version upgrade; your tunings are safe as long as you
//! stay on the same major version.
//!
//! Owes a great debt to:
//!
//! * https://en.wikipedia.org/wiki/PID_controller
//! * http://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
//! * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

// FIXME: it may be worth to explore http://de.mathworks.com/help/simulink/slref/pidcontroller.html
//        for additional features/inspiration
pub mod util;

use core::f32;

/// A generic controller interface.
///
/// A controller is fed timestamped values and calculates an adjusted value
/// based on previous readings.
///
/// Many controllers possess a set of adjustable parameters as well as a set
/// of input-value dependant state variables.
pub trait Controller {
    /// Record a measurement from the plant.
    ///
    /// Records a new values. `delta_t` is the time since the last update in
    /// seconds.
    fn update(&mut self, value: f32, delta_t: f32) -> f32;

    /// Adjust set target for the plant.
    ///
    /// The controller will usually try to adjust its output (from `update`) in
    /// a way that results in the plant approaching `target`.
    fn set_target(&mut self, target: f32);

    /// Retrieve target value.
    fn target(&self) -> f32;

    /// Reset internal state.
    ///
    /// Resets the internal state of the controller; not to be confused with
    /// its parameters.
    fn reset(&mut self);
}


/// PID controller derivative modes.
///
/// Two different ways of calculating the derivative can be used with the PID
/// controller, allowing to avoid "derivative kick" if needed (see
/// http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
/// for details information on the implementation that inspired this one).
///
/// Choosing `OnMeasurement` will avoid large bumps in the controller output
/// when changing the setpoint using `set_target()`.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum DerivativeMode {
    /// Calculate derivative of error (classic PID-Controller)
    OnError,
    /// Calculate derivative of actual changes in value.
    OnMeasurement,
}

/// PID Controller.
///
/// A PID controller, supporting the `Controller` interface. Any public values
/// are safe to modify while in operation.
///
/// `p_gain`, `i_gain` and `d_gain` are the respective gain values. The
/// controlller internally stores an already adjusted integral, making it safe
/// to alter the `i_gain` - it will *not* result in an immediate large jump in
/// controller output.
///
/// `i_min` and `i_max` are the limits for the internal integral storage.
/// Similarly, `out_min` and `out_max` clip the output value to an acceptable
/// range of values. By default, all limits are set to +/- infinity.
///
/// `d_mode` The `DerivativeMode`, the default is `OnMeasurement`.
#[derive(Debug, Clone)]
pub struct PIDController {
    /// Proportional gain
    pub p_gain: f32,

    /// Integral gain
    pub i_gain: f32,

    /// Differential gain,
    pub d_gain: f32,

    target: f32,

    // Integral range limits
    pub i_min: f32,
    pub i_max: f32,

    // Output range limits
    pub out_min: f32,
    pub out_max: f32,

    pub d_mode: DerivativeMode,

    // The PIDs internal state. All other attributes are configuration values
    err_sum: f32,
    prev_value: f32,
    prev_error: f32,
}

impl PIDController {
    /// Creates a new PID Controller.
    pub fn new(p_gain: f32, i_gain: f32, d_gain: f32) -> PIDController {
        PIDController{
            p_gain: p_gain,
            i_gain: i_gain,
            d_gain: d_gain,

            target: 0.0,

            err_sum: 0.0,
            prev_value: f32::NAN,
            prev_error: f32::NAN,

            i_min:  f32::INFINITY,
            i_max: f32::INFINITY,

            out_min:  f32::INFINITY,
            out_max: f32::INFINITY,

            d_mode: DerivativeMode::OnMeasurement,
        }
    }

    /// Convenience function to set `i_min`/`i_max` and `out_min`/`out_max`
    /// to the same values simultaneously.
    pub fn set_limits(&mut self, min: f32, max: f32) {
        self.i_min = min;
        self.i_max = max;
        self.out_min = min;
        self.out_max = max;
    }
}

impl Controller for PIDController {
    fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    fn target(&self) -> f32 {
        self.target
    }

    fn update(&mut self, value: f32, delta_t: f32) -> f32 {
        let error = self.target - value;

        // PROPORTIONAL
        let p_term = self.p_gain * error;

        // INTEGRAL
        self.err_sum = util::limit_range(
            self.i_min, self.i_max,
            self.err_sum + self.i_gain * error * delta_t
        );
        let i_term = self.err_sum;

        // DIFFERENTIAL
        let d_term = if self.prev_value.is_nan() || self.prev_error.is_nan() {
            // we have no previous values, so skip the derivative calculation
            0.0
        } else {
            match self.d_mode {
                DerivativeMode::OnMeasurement => {
                    // we use -delta_v instead of delta_error to reduce "derivative kick",
                    // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
                    self.d_gain * (self.prev_value - value) / delta_t
                },
                DerivativeMode::OnError => {
                    self.d_gain * (error - self.prev_error) / delta_t
                }
            }
        };

        // store previous values
        self.prev_value = value;
        self.prev_error = error;

        util::limit_range(
            self.out_min, self.out_max,
            p_term + d_term + i_term
        )
    }

    fn reset(&mut self) {
        self.prev_value = f32::NAN;
        self.prev_error = f32::NAN;

        // FIXME: http://brettbeauregard.com/blog/2011/04/improving-the-beginner
        //               %E2%80%99s-pid-initialization/
        //        suggests that this should not be there. however, it may miss
        //        the fact that input and output can be two completely
        //        different domains
        self.err_sum = 0.0;
    }
}