//! Frames and ids: where a robot's map sits in the station frame, and how robot node ids are
//! made unique across robots.

/// Rigid transform into the station frame: rotation about z (yaw), then translation.
#[derive(Clone, Copy, Debug, Default)]
pub struct Transform {
    pub yaw: f64,
    pub translation: [f64; 3],
}

impl Transform {
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let (s, c) = self.yaw.sin_cos();
        [
            c * p[0] - s * p[1] + self.translation[0],
            s * p[0] + c * p[1] + self.translation[1],
            p[2] + self.translation[2],
        ]
    }
}

/// Station id of robot `robot`'s node `id`: Hydra's category byte stays on top, the robot
/// number goes in the next byte. Hydra's own prefixes do not tell robots apart in every layer.
pub fn global_id(robot: u8, id: u64) -> u64 {
    let category = id >> 56;
    let index = id & ((1 << 48) - 1);
    (category << 56) | ((robot as u64) << 48) | index
}

/// The robot a station id came from.
pub fn robot_of(global: u64) -> u8 {
    ((global >> 48) & 0xff) as u8
}
