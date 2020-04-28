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
use crate::util;
use num_traits;

/// A generic controller interface.
///
/// A controller is fed timestamped values and calculates an adjusted value
/// based on previous readings.
///
/// Many controllers possess a set of adjustable parameters as well as a set
/// of input-value dependant state variables.
pub trait Controller<T> {
    /// Record a measurement from the plant.
    ///
    /// Records a new values. `delta_t` is the time since the last update in
    /// seconds.
    fn update(&mut self, value: T, delta_t: T) -> T;

    /// Adjust set target for the plant.
    ///
    /// The controller will usually try to adjust its output (from `update`) in
    /// a way that results in the plant approaching `target`.
    fn set_target(&mut self, target: T);

    /// Retrieve target value.
    fn target(&self) -> T;

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
pub struct PIDController<T> {
    /// Proportional gain
    pub p_gain: T,

    /// Integral gain
    pub i_gain: T,

    /// Differential gain,
    pub d_gain: T,

    target: T,

    // Integral range limits
    pub i_min: T,
    pub i_max: T,

    // Output range limits
    pub out_min: T,
    pub out_max: T,

    pub d_mode: DerivativeMode,

    // The PIDs internal state. All other attributes are configuration values
    err_sum: T,
    prev_value: Option<T>,
    prev_error: Option<T>,
}

impl<T> PIDController<T>
where
    T: num_traits::Bounded + num_traits::identities::Zero + Copy,
{
    /// Creates a new PID Controller.
    pub fn new(p_gain: T, i_gain: T, d_gain: T) -> PIDController<T> {
        PIDController {
            p_gain: p_gain,
            i_gain: i_gain,
            d_gain: d_gain,

            target: T::zero(),

            err_sum: T::zero(),
            prev_value: None,
            prev_error: None,

            i_min: T::max_value(),
            i_max: T::max_value(),

            out_min: T::max_value(),
            out_max: T::max_value(),

            d_mode: DerivativeMode::OnMeasurement,
        }
    }
    /// Convenience function to set `i_min`/`i_max` and `out_min`/`out_max`
    /// to the same values simultaneously.
    pub fn set_limits(&mut self, min: T, max: T) {
        self.i_min = min;
        self.i_max = max;
        self.out_min = min;
        self.out_max = max;
    }
}

impl<T> Controller<T> for PIDController<T>
where
    T: num_traits::CheckedAdd
        + num_traits::CheckedSub
        + num_traits::CheckedMul
        + num_traits::CheckedDiv
        + num_traits::identities::Zero
        + PartialOrd
        + Copy,
{
    fn set_target(&mut self, target: T) {
        self.target = target;
    }

    fn target(&self) -> T {
        self.target
    }

    fn update(&mut self, value: T, delta_t: T) -> T {
        let error = self.target - value;

        // PROPORTIONAL
        let p_term = self.p_gain * error;

        // INTEGRAL
        self.err_sum = util::clamp(
            self.i_min,
            self.i_max,
            self.err_sum + self.i_gain * error * delta_t,
        );
        let i_term = self.err_sum;

        // DIFFERENTIAL
        let d_term =
            if let (Some(prev_value), Some(prev_error)) = (self.prev_value, self.prev_error) {
                match self.d_mode {
                    DerivativeMode::OnMeasurement => {
                        // we use -delta_v instead of delta_error to reduce "derivative kick",
                        // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
                        self.d_gain * (prev_value - value) / delta_t
                    }
                    DerivativeMode::OnError => self.d_gain * (error - prev_error) / delta_t,
                }
            } else {
                T::zero()
            };

        // store previous values
        self.prev_value = Some(value);
        self.prev_error = Some(error);

        p_term + d_term + i_term
        // util::limit_range(
        //     self.out_min, self.out_max,
        //     p_term + d_term + i_term
        // )
    }

    fn reset(&mut self) {
        self.prev_value = None;
        self.prev_error = None;

        // FIXME: http://brettbeauregard.com/blog/2011/04/improving-the-beginner
        //               %E2%80%99s-pid-initialization/
        //        suggests that this should not be there. however, it may miss
        //        the fact that input and output can be two completely
        //        different domains
        self.err_sum = T::zero();
    }
}
