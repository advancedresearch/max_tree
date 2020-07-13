/*

An experiment to use a greedy optimizer for landing a spaceship on the Moon from the Earth.
Hopefully, without crashing. :)

This is example is currently working, but is far from realistic.

TODO:

- [x] Add rigid body physics
- [ ] Add gravity
- [ ] Add force control of spaceship (instead of acceleration)
- [ ] Add realistic scales to planets (mass, radius, distances)
- [ ] Add Moon orbit (the Moon is moving relative to the Earth)
- [ ] Add Earth atmosphere (air drag)
- [ ] Add realistic spaceship control scales
- [ ] Add spaceport source GPS coordinates
- [ ] Add moonbase target GPS coordinates
- [ ] Add spaceship geometry
- [ ] Add Moon landscape geometry
- [ ] Add more realistic spaceship thrusters (off-center)
- [ ] Add rocket fuel physics
- [ ] Add rocket stage separation carrying spaceship (change of mass)

*/

use max_tree::prelude::*;

/// Stores information about a planet.
pub struct Planet {
    /// Name of planet.
    pub name: String,
    /// Position.
    pub pos: [f64; 3],
    /// Mass.
    pub mass: f64,
    /// Radius.
    pub radius: f64,
}

impl Planet {
    /// Calculates distance to the planet's surface.
    ///
    /// If negative, the position is below the surface.
    pub fn distance(&self, pos: [f64; 3]) -> f64 {
        use vecmath::vec3_len as len;
        use vecmath::vec3_sub as sub;

        len(sub(self.pos, pos)) - self.radius
    }
}

/// The type of attitude for orientation, torque and wrench.
pub type Attitude = (f64, [f64; 3]);

/// Used as node data.
#[derive(Clone, Debug)]
pub struct Spaceship {
    /// Position.
    pub pos: [f64; 3],
    /// Velocity and direction.
    pub vel: [f64; 3],
    /// Linear acceleration.
    pub acc: [f64; 3],
    /// Orientation axis.
    pub ori: Attitude,
    /// Torque axis.
    pub tor: Attitude,
    /// Wrench axis.
    pub wre: Attitude,
    /// Mass.
    pub mass: f64,
}

/// Creates a rotation matrix from attitude.
///
/// This matrix is column major.
pub fn rot(att: Attitude) -> [[f64; 3]; 3] {
    let ang = att.0;
    let axis = att.1;
    let cos = ang.cos();
    let sin = ang.sin();
    let inv_cos = 1.0 - cos;
    [
      [
        cos + axis[0] * axis[0] * inv_cos,
        axis[0] * axis[1] * inv_cos - axis[2] * sin,
        axis[0] * axis[2] * inv_cos + axis[1] * sin,
      ],
      [
        axis[1] * axis[0] * inv_cos + axis[2] * sin,
        cos + axis[1] * axis[1] * inv_cos,
        axis[1] * axis[2] * inv_cos - axis[0] * sin,
      ],
      [
        axis[2] * axis[0] * inv_cos - axis[1] * sin,
        axis[2] * axis[1] * inv_cos + axis[0] * sin,
        cos + axis[2] * axis[2] * inv_cos,
      ],
    ]
}

/// Solves the analogue of `s' = s + v * t` for attitude.
pub fn angular(a: Attitude, b: Attitude, t: f64) -> Attitude {
    use vecmath::vec3_dot as dot;
    use vecmath::col_mat3_transform as transform;

    let angle = a.0 + dot(a.1, b.1) * b.0 * t;
    let rot = rot((b.0 * t, b.1));
    let axis = transform(rot, a.1);
    (angle, axis)
}

impl Spaceship {
    /// Updates spaceship's linear coordinates by moving it through time.
    pub fn update_linear(&mut self, dt: f64) {
        use vecmath::vec3_add as add;
        use vecmath::vec3_scale as scale;

        self.vel = add(self.vel, scale(self.acc, 0.5 * dt));
        self.pos = add(self.pos, scale(self.vel, dt));
        self.vel = add(self.vel, scale(self.acc, 0.5 * dt));
    }

    /// Updates spaceship's angular coordinates by moving it through time.
    pub fn update_angular(&mut self, dt: f64) {
        self.tor = angular(self.tor, self.wre, 0.5 * dt);
        self.ori = angular(self.ori, self.tor, dt);
        self.tor = angular(self.tor, self.wre, 0.5 * dt);
    }

    /// Update spaceship's coordinates by moving it through time.
    pub fn update(&mut self, dt: f64) {
        self.update_linear(dt);
        self.update_angular(dt);
    }

    /// Calculates the speed.
    pub fn speed(&self) -> f64 {
        use vecmath::vec3_len as len;

        len(self.vel)
    }
}

/// Represents objects in space.
pub struct Space {
    /// Fixed timestep.
    pub dt: f64,
    /// List of planets.
    pub planets: Vec<Planet>,
    /// State of spaceship.
    pub spaceship: Spaceship,
    /// The planet of destination.
    pub target_planet: usize,
    /// The target orientation.
    pub target_orientation: Attitude,
}

/// Calculates the angle between vectors.
pub fn angle_between_vectors(a: [f64; 3], b: [f64; 3]) -> f64 {
    use vecmath::vec3_dot as dot;
    use vecmath::vec3_len as len;

    (dot(a, b) / (len(a) * len(b))).acos()
}

impl Space {
    /// Calculates utility for getting close to the surface of a planet.
    pub fn utility_get_close_to_surface(&self, planet: usize) -> f64 {
        -self.planets[planet].distance(self.spaceship.pos).abs()
    }

    /// Calculates utility for stopping spaceship.
    pub fn utility_full_stop(&self) -> f64 {
        -self.spaceship.speed()
    }

    /// Calculates utility for spaceship orientation.
    pub fn utility_orientation(&self) -> f64 {
        let spaceship_angle = self.spaceship.ori.0;
        let target_angle = self.target_orientation.0;
        let spaceship_dir = [spaceship_angle.cos(), spaceship_angle.sin(), 0.0];
        let target_dir = [target_angle.cos(), target_angle.sin(), 0.0];
        let utility_angle = -angle_between_vectors(target_dir, spaceship_dir).abs();

        let spaceship_axis = self.spaceship.ori.1;
        let target_axis = self.target_orientation.1;
        let utility_axis = -angle_between_vectors(target_axis, spaceship_axis).abs();

        utility_angle + utility_axis
    }
}

#[derive(Debug, Clone)]
pub enum Action {
    /// Acceleration.
    Acc([f64; 3]),
    /// Wrench.
    Wre(Attitude),
}

pub const EARTH: usize = 0;
pub const MOON: usize = 1;

fn main() {
    let mut space = Space {
        planets: vec![
            Planet {
                pos: [0.0, 0.0, 0.0],
                mass: 1.0,
                radius: 1.0,
                name: "Earth".into(),
            },
            Planet {
                pos: [3.0, 0.0, 0.0],
                mass: 1.0,
                radius: 1.0,
                name: "Moon".into(),
            },
        ],
        spaceship: Spaceship {
            pos: [0.0, 0.0, 0.0],
            vel: [0.0, 0.0, 0.0],
            acc: [0.0, 0.0, 0.0],
            ori: (0.0, [1.0, 0.0, 0.0]),
            tor: (0.0, [1.0, 0.0, 0.0]),
            wre: (0.0, [1.0, 0.0, 0.0]),
            mass: 1.0,
        },
        dt: 0.5,
        target_planet: MOON,
        target_orientation: (1.0, [1.0, 0.0, 0.0]),
    };

    let max_depth = 10;
    let eps_depth = 0.0;
    let mut settings = AiSettings::new(max_depth, eps_depth);
    settings.analysis = true;
    settings.max_mib = Some(10.0);
    let mut ai = Ai {
        actions: actions_x,
        execute: execute,
        settings: settings,
        undo: undo,
        utility: utility2,
        analysis: AiAnalysis::new(),
    };
    let mut root = Node::root(space.spaceship.clone());
    ai.greedy(&mut root, 0, &mut space);

    let mut wrench_count = 0;
    let mut node = &root;
    loop {
        let utility = (ai.utility)(&node.data, &space);
        println!(
            "Pos: {:?}\nVel: {:?}\nOri: {:?}\nTor: {:?}\nUtility: {}\n",
            space.spaceship.pos,
            space.spaceship.vel,
            space.spaceship.ori,
            space.spaceship.tor,
            utility
        );
        if let Some(i) = ai.update(node, &mut space) {
            let action = &node.children[i].0;
            if let Action::Wre(_) = action {
                wrench_count += 1;
            }
            println!("Action: {:?}", node.children[i].0);
            node = &node.children[i].1;
        } else {
            break;
        }
    }

    println!("Wrench count: {}", wrench_count);

    let analysis = &ai.analysis;
    println!("GiB: {}", analysis.gib(ai.node_size()));
    println!("MiB: {}", analysis.mib(ai.node_size()));
    println!("KiB: {}", analysis.kib(ai.node_size()));
}

pub fn acc_xyz(v: f64, arr: &mut Vec<Action>) {
    arr.push(Action::Acc([v, 0.0, 0.0]));
    arr.push(Action::Acc([-v, 0.0, 0.0]));
    arr.push(Action::Acc([0.0, v, 0.0]));
    arr.push(Action::Acc([0.0, -v, 0.0]));
    arr.push(Action::Acc([0.0, 0.0, v]));
    arr.push(Action::Acc([0.0, 0.0, -v]));
}

pub fn acc_x(v: f64, arr: &mut Vec<Action>) {
    arr.push(Action::Acc([v, 0.0, 0.0]));
    arr.push(Action::Acc([-v, 0.0, 0.0]));
}

pub fn wre_x(v: f64, arr: &mut Vec<Action>) {
    arr.push(Action::Wre((v, [1.0, 0.0, 0.0])));
    arr.push(Action::Wre((-v, [1.0, 0.0, 0.0])));
}

pub fn actions_x(_: &Spaceship, _: &Space) -> Vec<Action> {
    let mut arr = vec![];
    acc_x(0.1, &mut arr);
    acc_x(0.2, &mut arr);
    acc_x(0.3, &mut arr);
    acc_x(0.5, &mut arr);
    acc_x(0.6, &mut arr);
    acc_x(1.0, &mut arr);
    acc_x(1.2, &mut arr);
    acc_x(1.3, &mut arr);
    wre_x(0.1, &mut arr);
    arr
}

pub fn actions_xyz(_: &Spaceship, _: &Space) -> Vec<Action> {
    let mut arr = vec![];
    acc_xyz(0.1, &mut arr);
    acc_xyz(0.2, &mut arr);
    acc_xyz(0.3, &mut arr);
    acc_xyz(0.5, &mut arr);
    acc_xyz(0.6, &mut arr);
    acc_xyz(1.0, &mut arr);
    acc_xyz(1.2, &mut arr);
    acc_xyz(1.3, &mut arr);
    arr
}

fn execute(_: &Spaceship, acc: &Action, space: &mut Space) -> Result<Spaceship, ()> {
    let old = space.spaceship.clone();
    match acc {
        Action::Acc(acc) => {
            // Set spaceship acceleration.
            space.spaceship.acc = *acc;
        }
        Action::Wre(wre) => {
            // Set spaceship wrench.
            space.spaceship.wre = *wre;
        }
    }
    space.spaceship.update(space.dt);
    Ok(old)
}

fn undo(old: &Spaceship, space: &mut Space) {
    // Reset spaceship position.
    space.spaceship = old.clone();
}

/// Computes utility of getting close to surface.
fn utility_get_close_to_surface(space: &Space) -> f64 {
    space.utility_get_close_to_surface(space.target_planet)
}

/// Computes utility of stopping spaceship.
fn utility_full_stop(space: &Space) -> f64 {
    let dist = space.planets[space.target_planet].distance(space.spaceship.pos).abs();
    absoid(0.2, 1.0, dist) * space.utility_full_stop()
}

fn utility_orientation(space: &Space) -> f64 {
    let dist = space.planets[space.target_planet].distance(space.spaceship.pos).abs();
    absoid(0.2, 1.0, dist) * space.utility_orientation()
}

/// Used with `full`.
pub fn utility1(_: &Spaceship, space: &Space) -> f64 {
    utility_get_close_to_surface(space) +
    space.utility_full_stop()
}

/// Used with `greedy`.
pub fn utility2(_: &Spaceship, space: &Space) -> f64 {
    utility_get_close_to_surface(space) +
    utility_full_stop(space) +
    utility_orientation(space)
}

/// Used to control transition to full stop.
///
/// For more information, see paper about "Absoid Functions"
/// (https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/absoid-functions.pdf).
pub fn absoid(z: f64, n: f64, x: f64) -> f64 {
    1.0 / ((x / z).powf(n) + 1.0)
}
