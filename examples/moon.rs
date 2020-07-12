/*

An experiment to use a greedy optimizer for landing a spaceship on the Moon from the Earth.
Hopefully, without crashing. :)

This is example is currently working, but is far from realistic.

TODO:

- [ ] Add torque physics
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

/// Used as node data.
#[derive(Clone, Debug)]
pub struct Spaceship {
    /// Position.
    pub pos: [f64; 3],
    /// Velocity and direction.
    pub vel: [f64; 3],
    /// Linear acceleration.
    pub acc: [f64; 3],
    /// Torque (rotational acceleration).
    pub torq: [f64; 3],
    /// Mass.
    pub mass: f64,
}

impl Spaceship {
    /// Updates spaceship by moving it through time.
    pub fn update(&mut self, dt: f64) {
        use vecmath::vec3_add as add;
        use vecmath::vec3_scale as scale;

        self.vel = add(self.vel, scale(self.acc, 0.5 * dt));
        self.pos = add(self.pos, scale(self.vel, dt));
        self.vel = add(self.vel, scale(self.acc, 0.5 * dt));
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
}

pub const EARTH: usize = 0;
pub const MOON: usize = 1;

fn main() {
    let mut space = Space {
        dt: 1.0,
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
            torq: [0.0, 0.0, 0.0],
            mass: 1.0,
        }
    };

    let max_depth = 5;
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

    let mut node = &root;
    loop {
        let utility = (ai.utility)(&node.data, &space);
        println!(
            "Pos: {:?}, Vel: {:?} = {}",
            space.spaceship.pos,
            space.spaceship.vel,
            utility
        );
        if let Some(i) = ai.update(node, &mut space) {
            println!("  Action: {:?}", node.children[i].0);
            node = &node.children[i].1;
        } else {
            break;
        }
    }

    let analysis = &ai.analysis;
    println!("GiB: {}", analysis.gib(ai.node_size()));
    println!("MiB: {}", analysis.mib(ai.node_size()));
    println!("KiB: {}", analysis.kib(ai.node_size()));
}

pub fn acc_xyz(v: f64, arr: &mut Vec<[f64; 3]>) {
    arr.push([v, 0.0, 0.0]);
    arr.push([-v, 0.0, 0.0]);
    arr.push([0.0, v, 0.0]);
    arr.push([0.0, -v, 0.0]);
    arr.push([0.0, 0.0, v]);
    arr.push([0.0, 0.0, -v]);
}

pub fn acc_x(v: f64, arr: &mut Vec<[f64; 3]>) {
    arr.push([v, 0.0, 0.0]);
    arr.push([-v, 0.0, 0.0]);
}

pub fn actions_x(_: &Spaceship, _: &Space) -> Vec<[f64; 3]> {
    let mut arr = vec![];
    acc_x(0.1, &mut arr);
    acc_x(0.2, &mut arr);
    acc_x(0.3, &mut arr);
    acc_x(0.5, &mut arr);
    acc_x(0.6, &mut arr);
    acc_x(1.0, &mut arr);
    acc_x(1.2, &mut arr);
    acc_x(1.3, &mut arr);
    arr
}

pub fn actions_xyz(_: &Spaceship, _: &Space) -> Vec<[f64; 3]> {
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

fn execute(_: &Spaceship, acc: &[f64; 3], space: &mut Space) -> Result<Spaceship, ()> {
    let old = space.spaceship.clone();
    // Set spaceship acceleration.
    space.spaceship.acc = *acc;
    space.spaceship.update(space.dt);
    Ok(old)
}

fn undo(old: &Spaceship, space: &mut Space) {
    // Reset spaceship position.
    space.spaceship = old.clone();
}

/// Used with `full`.
pub fn utility1(_: &Spaceship, space: &Space) -> f64 {
    space.utility_get_close_to_surface(MOON) +
    space.utility_full_stop()
}

/// Used with `greedy`.
pub fn utility2(_: &Spaceship, space: &Space) -> f64 {
    let dist = space.planets[MOON].distance(space.spaceship.pos).abs();
    space.utility_get_close_to_surface(MOON) +
    absoid(0.2, 1.0, dist) * space.utility_full_stop()
}

/// Used to control transition to full stop.
///
/// For more information, see paper about "Absoid Functions"
/// (https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/absoid-functions.pdf).
pub fn absoid(z: f64, n: f64, x: f64) -> f64 {
    1.0 / ((x / z).powf(n) + 1.0)
}
