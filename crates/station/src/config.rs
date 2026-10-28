//! Which nodes merge: one rule per scene graph layer.

use dsg_convert::LayerKey;

pub const OBJECTS: LayerKey = (2, 0);
pub const PLACES: LayerKey = (3, 0);
pub const ROOMS: LayerKey = (4, 0);

/// Two robots' nodes of `layer` are the same node when their centres are within `radius`
/// metres (and, if `same_label`, they have the same semantic label).
#[derive(Clone, Copy, Debug)]
pub struct MergeRule {
    pub layer: LayerKey,
    pub radius: f64,
    pub same_label: bool,
}

pub struct Config {
    pub rules: Vec<MergeRule>,
    /// A kept node absorbs at most one node of each later robot.
    pub one_to_one: bool,
    /// Pair nodes so the most merge (maximum matching), not just nearest first.
    /// One-to-one only.
    pub max_matching: bool,
}

impl Default for Config {
    fn default() -> Self {
        let rule = |layer, radius, same_label| MergeRule {
            layer,
            radius,
            same_label,
        };
        Config {
            rules: vec![
                rule(OBJECTS, 0.5, true),
                rule(PLACES, 1.0, false),
                rule(ROOMS, 1.0, false),
            ],
            one_to_one: true,
            max_matching: false,
        }
    }
}

impl Config {
    /// Merge objects only.
    pub fn objects_only() -> Self {
        let mut c = Config::default();
        c.rules.retain(|r| r.layer == OBJECTS);
        c
    }
}
