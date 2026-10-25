//! What happens to a robot: a file of faults, each at a set time or at
//! random, beside the robot file, which says what the robot is.
//!
//! ```yaml
//! seed: 7                     # optional: the same seed, the same run
//! faults:
//!   - joint: openarm_joint4   # exactly then
//!     fault: dead
//!     at: 6 s
//!   - joint: any              # at random, about once a minute
//!     fault: freeze
//!     every: 60 s
//!     lasts: 0.5 s            # then it recovers; without it, it doesn't
//!   - supply: off             # the motors' supply: off, or a voltage (`18 V`)
//!     at: 20 s
//!     lasts: 1 s
//! ```
//!
//! A joint's fault is one of [`Breakdown`]'s; the supply's sets its
//! voltage (`power.rs`). `every` is the mean time between two
//! (they come as a Poisson process: any moment as likely as the next), so
//! it doesn't depend on the scene's timestep. Each fault draws from its own
//! generator, seeded from the file's seed and its place in the list: adding
//! a fault leaves the others' times as they were. Times are simulated
//! seconds; `at` and `lasts` take `s` or `ms`.
//!
//! What happened, when, is the answer key for whatever runs against
//! drydock: it says so in its log, and in `--trace`'s file.

use std::path::Path;

use serde::Deserialize;

use crate::effects::{Breakdown, Rng};

/// A fault starting or ending, `at` simulated seconds into the run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Happening {
    pub at: f64,
    pub fault: Fault,
    /// It starts (when it ends, if it does) or it ends.
    pub what: Change,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fault {
    /// A motor breaks: which, counted across buses in the robot file's
    /// order, and how.
    Motor { motor: usize, breakdown: Breakdown },
    /// The supply goes to this voltage, V: 0 when it's off.
    Supply { volts: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Change {
    Breaks { until: Option<f64> },
    Recovers,
}

pub struct Scenario {
    faults: Vec<Rule>,
}

struct Rule {
    target: Target,
    when: When,
    lasts: Option<f64>,
    rng: Rng,
    /// When it next happens; none once it won't again.
    next: Option<f64>,
}

#[derive(Clone, Copy)]
enum Target {
    /// A motor, or any, drawn each time, and how it breaks.
    Motor(Option<usize>, Breakdown),
    /// The supply, to a voltage.
    Supply(f64),
}

#[derive(Clone, Copy)]
enum When {
    At(f64),
    Every(f64),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    #[serde(default)]
    seed: u64,
    faults: Vec<FaultSpec>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct FaultSpec {
    joint: Option<String>,
    fault: Option<Breakdown>,
    supply: Option<Volts>,
    at: Option<Seconds>,
    every: Option<Seconds>,
    lasts: Option<Seconds>,
}

/// `6 s`, `500 ms`, or a number of seconds.
#[derive(Clone, Copy, Deserialize)]
#[serde(try_from = "serde_yaml::Value")]
struct Seconds(f64);

impl TryFrom<serde_yaml::Value> for Seconds {
    type Error = String;

    fn try_from(v: serde_yaml::Value) -> Result<Self, String> {
        let s = match &v {
            serde_yaml::Value::Number(n) => n.as_f64().ok_or("not a number")?,
            serde_yaml::Value::String(text) => {
                let text = text.trim();
                let (number, scale) = match text.strip_suffix("ms") {
                    Some(n) => (n, 1e-3),
                    None => (text.strip_suffix('s').unwrap_or(text), 1.0),
                };
                number.trim().parse::<f64>().map_err(|_| format!("{text:?}: seconds, as `6 s` or `500 ms`"))? * scale
            }
            _ => return Err("seconds, as `6 s` or `500 ms`".into()),
        };
        if !s.is_finite() || s < 0.0 {
            return Err(format!("{s} s: zero or more seconds"));
        }
        Ok(Seconds(s))
    }
}

/// `off`, `18 V`, or a number of volts.
#[derive(Clone, Copy, Deserialize)]
#[serde(try_from = "serde_yaml::Value")]
struct Volts(f64);

impl TryFrom<serde_yaml::Value> for Volts {
    type Error = String;

    fn try_from(v: serde_yaml::Value) -> Result<Self, String> {
        let volts = match &v {
            serde_yaml::Value::Number(n) => n.as_f64().ok_or("not a number")?,
            serde_yaml::Value::String(text) if text.trim() == "off" => 0.0,
            serde_yaml::Value::String(text) => {
                let text = text.trim();
                let number = text.strip_suffix('V').unwrap_or(text).trim();
                number.parse::<f64>().map_err(|_| format!("{text:?}: `off`, or volts, as `18 V`"))?
            }
            _ => return Err("`off`, or volts, as `18 V`".into()),
        };
        if !volts.is_finite() || volts < 0.0 {
            return Err(format!("{volts} V: zero or more volts"));
        }
        Ok(Volts(volts))
    }
}

impl Scenario {
    /// A scenario's file, for a robot whose motors drive `joints`, in the
    /// robot file's order.
    pub fn load(path: impl AsRef<Path>, joints: &[String]) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| format!("can't read {}: {e}", path.display()))?;
        Self::parse(&text, joints).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str, joints: &[String]) -> Result<Self, String> {
        let file: File = serde_yaml::from_str(text).map_err(|e| e.to_string())?;
        let mut faults = Vec::new();
        for (n, f) in file.faults.into_iter().enumerate() {
            let target = match (f.joint, f.fault, f.supply) {
                (Some(joint), Some(fault), None) => {
                    let motor = match joint.as_str() {
                        "any" => None,
                        name => Some(joints.iter().position(|j| j == name).ok_or_else(|| {
                            format!(
                                "fault {}: no motor drives {name:?}; the robot's are {}, or `any`",
                                n + 1,
                                joints.join(", ")
                            )
                        })?),
                    };
                    Target::Motor(motor, fault)
                }
                (None, None, Some(volts)) => Target::Supply(volts.0),
                _ => return Err(format!("fault {}: either a `joint` and its `fault`, or the `supply`", n + 1)),
            };
            let when = match (f.at, f.every) {
                (Some(at), None) => When::At(at.0),
                (None, Some(every)) if every.0 > 0.0 => When::Every(every.0),
                (None, Some(_)) => return Err(format!("fault {}: `every` must be more than 0 s", n + 1)),
                _ => return Err(format!("fault {}: either `at` (when) or `every` (how often, at random)", n + 1)),
            };
            let mut rng = Rng::new(file.seed ^ ((n as u64 + 1) << 40));
            let next = Some(match when {
                When::At(at) => at,
                When::Every(mean) => mean * exponential(&mut rng),
            });
            faults.push(Rule { target, when, lasts: f.lasts.map(|s| s.0), rng, next });
        }
        Ok(Scenario { faults })
    }

    /// The faults that start by `now`, among `motors` motors, in the order
    /// they do. Each is returned once.
    pub fn due(&mut self, now: f64, motors: usize) -> Vec<Happening> {
        let mut due = Vec::new();
        for rule in &mut self.faults {
            while let Some(at) = rule.next.filter(|&t| t <= now) {
                let fault = match rule.target {
                    Target::Motor(motor, breakdown) => {
                        let motor = motor.unwrap_or_else(|| (rule.rng.unit() * motors as f64) as usize % motors);
                        Fault::Motor { motor, breakdown }
                    }
                    Target::Supply(volts) => Fault::Supply { volts },
                };
                let until = rule.lasts.map(|l| at + l);
                due.push(Happening { at, fault, what: Change::Breaks { until } });
                rule.next = match rule.when {
                    When::At(_) => None,
                    When::Every(mean) => Some(at + mean * exponential(&mut rule.rng)),
                };
            }
        }
        due.sort_by(|a, b| a.at.total_cmp(&b.at));
        due
    }
}

/// An exponential draw, mean 1: the time to the next of a Poisson process.
fn exponential(rng: &mut Rng) -> f64 {
    -(1.0 - rng.unit()).ln()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn joints() -> Vec<String> {
        ["shoulder", "elbow"].map(String::from).to_vec()
    }

    #[test]
    fn a_fault_at_a_time_happens_then_once() {
        let mut s =
            Scenario::parse("faults: [{ joint: elbow, fault: dead, at: 6 s, lasts: 500 ms }]", &joints()).unwrap();
        assert!(s.due(5.999, 2).is_empty());
        let h = s.due(6.001, 2);
        assert_eq!(
            h,
            [Happening {
                at: 6.0,
                fault: Fault::Motor { motor: 1, breakdown: Breakdown::Dead },
                what: Change::Breaks { until: Some(6.5) }
            }]
        );
        assert!(s.due(100.0, 2).is_empty());
    }

    #[test]
    fn random_faults_come_about_as_often_as_asked_the_same_with_the_same_seed() {
        let text = "seed: 3\nfaults: [{ joint: any, fault: freeze, every: 10 }]";
        let run = || Scenario::parse(text, &joints()).unwrap().due(10_000.0, 2);
        let (a, b) = (run(), run());
        assert_eq!(a, b);
        // About a thousand, give or take a few standard deviations (√1000 ≈ 32).
        assert!((850..1150).contains(&a.len()), "{}", a.len());
        let on = |m| a.iter().any(|h| matches!(h.fault, Fault::Motor { motor, .. } if motor == m));
        assert!(on(0) && on(1), "any joint");
        let mut other =
            Scenario::parse("seed: 4\nfaults: [{ joint: any, fault: freeze, every: 10 }]", &joints()).unwrap();
        assert_ne!(a, other.due(10_000.0, 2));
    }

    #[test]
    fn adding_a_fault_leaves_the_others_as_they_were() {
        let one = "faults: [{ joint: elbow, fault: weak, every: 5 s }]";
        let two = "faults: [{ joint: elbow, fault: weak, every: 5 s }, { joint: shoulder, fault: noisy, every: 1 s }]";
        let weak = |text| {
            let h = Scenario::parse(text, &joints()).unwrap().due(100.0, 2);
            let weak = |h: &Happening| matches!(h.fault, Fault::Motor { breakdown: Breakdown::Weak, .. });
            h.into_iter().filter(weak).map(|h| h.at).collect::<Vec<_>>()
        };
        assert_eq!(weak(one), weak(two));
    }

    #[test]
    fn mistakes_are_named() {
        let err = |text| Scenario::parse(text, &joints()).err().unwrap();
        assert!(err("faults: [{ joint: wrist, fault: dead, at: 1 }]").contains("no motor drives \"wrist\""));
        assert!(err("faults: [{ joint: elbow, fault: dead }]").contains("either `at`"));
        assert!(err("faults: [{ joint: elbow, fault: dead, at: 1, every: 2 }]").contains("either `at`"));
        assert!(err("faults: [{ joint: elbow, fault: dark, at: 1 }]").contains("dark"));
        assert!(err("faults: [{ joint: elbow, fault: dead, at: soon }]").contains("seconds"));
        assert!(err("faults: [{ joint: elbow, fault: dead, every: 0 }]").contains("more than 0"));
        assert!(err("faults: [{ joint: elbow, supply: off, at: 1 }]").contains("or the `supply`"));
        assert!(err("faults: [{ supply: low, at: 1 }]").contains("volts"));
    }

    #[test]
    fn the_supply_goes_off_or_to_a_voltage() {
        let text = "faults: [{ supply: off, at: 1 s, lasts: 1 s }, { supply: 18 V, at: 5 s }]";
        let h = Scenario::parse(text, &joints()).unwrap().due(10.0, 2);
        assert_eq!(
            h.iter().map(|h| h.fault).collect::<Vec<_>>(),
            [Fault::Supply { volts: 0.0 }, Fault::Supply { volts: 18.0 }]
        );
    }
}
