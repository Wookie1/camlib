//! Named controller dispatch. Every controller-dependent hook matches the
//! variant exhaustively: adding a controller must never inherit another's words.
use super::{carvera::Carvera, decode, haas::HaasNGC, machine::Machine, post};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "name", deny_unknown_fields)]
pub enum Controller {
    #[serde(rename = "haas_ngc")]
    HaasNGC(HaasNGC),
    #[serde(rename = "carvera")]
    Carvera(Carvera),
}
impl Controller {
    pub fn validate(&self, machine: &Machine) -> Result<(), String> {
        match self {
            Self::HaasNGC(h) => h.validate(machine),
            Self::Carvera(c) => c.validate(machine),
        }
    }
}
pub struct Identity {
    pub post: &'static str,
    pub post_version: &'static str,
    pub decoder: &'static str,
    pub decoder_version: &'static str,
}
pub fn identity(controller: Option<&Controller>) -> Identity {
    match controller {
        None => Identity {
            post: "camlib::gcode::post::DefaultPost",
            post_version: post::VERSION,
            decoder: "camlib::gcode::decode::Decoder",
            decoder_version: decode::VERSION,
        },
        Some(Controller::Carvera(_)) => Identity {
            post: "camlib::gcode::post::Carvera",
            post_version: post::CARVERA_VERSION,
            decoder: "camlib::gcode::decode::Carvera",
            decoder_version: decode::CARVERA_VERSION,
        },
        Some(Controller::HaasNGC(_)) => Identity {
            post: "camlib::gcode::post::HaasNGC",
            post_version: post::HAAS_VERSION,
            decoder: "camlib::gcode::decode::HaasNGC",
            decoder_version: decode::HAAS_VERSION,
        },
    }
}
