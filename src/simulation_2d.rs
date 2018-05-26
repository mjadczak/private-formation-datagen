use base::*;
use dubins::MultiDubinsPath;
use specs::prelude::*;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::ops::AddAssign;
use std::ops::Index;
use tf_record::ToFloatFeatures;

// Extra maths
trait MathExt {
    fn fmodr(self, other: Self) -> Self;
    fn mod2pi(self) -> Self;
}

impl MathExt for f64 {
    fn fmodr(self, other: f64) -> f64 {
        self - other * (self / other).floor()
    }

    fn mod2pi(self) -> f64 {
        self.fmodr(2. * PI)
    }
}

const D: Metres = 0.1;
const A1: f64 = 0.25;
const A2: f64 = 0.25;

type UniformDynamicTrajectory = Vec<(Seconds, NonHolonomicDynamics)>;

/// Contains the uniform resolution index, as well as the actual clock time, for assertions
#[derive(Debug, Default)]
struct GlobalUniformTime {
    resolution: Seconds,
    ticks: usize,
}

impl GlobalUniformTime {
    pub fn sim_delta(&self) -> Seconds {
        self.resolution
    }

    pub fn sim_time(&self) -> Seconds {
        (self.ticks as f64) * self.resolution
    }

    pub fn sim_index(&self) -> usize {
        self.ticks
    }

    pub fn tick(&mut self) {
        self.ticks += 1;
    }

    pub fn new(resolution: Seconds) -> Self {
        GlobalUniformTime {
            resolution,
            ticks: 0,
        }
    }
}

#[derive(Debug, Component, Copy, Clone)]
pub struct NonHolonomicDynamics {
    pub position: Metres2D,
    pub heading: Radians,
    pub speed: MetresPerSecond,
    pub angular_velocity: Radians,
}

impl NonHolonomicDynamics {
    pub fn update(&mut self, delta: Seconds) {
        self.heading += (delta * self.angular_velocity).mod2pi();
        self.position += PolarMetres2D::new(self.speed * delta, self.heading).to_cartesian();
    }

    pub fn caster_position(&self) -> Metres2D {
        self.position + PolarMetres2D::new(D, self.heading).to_cartesian()
    }

    /// Calculates l, psi and gamma between self and a follower
    pub fn calculate_control_parameters(&self, follower: &Self) -> (Metres, Radians, Radians) {
        let l_vec = follower.caster_position() - self.position;
        let l = l_vec.length();
        let psi = (l_vec.angle().mod2pi() - self.heading).mod2pi();
        let gamma = (self.heading + psi - follower.heading).mod2pi();
        (l, psi, gamma)
    }
}

impl ToFloatFeatures for NonHolonomicDynamics {
    fn repr() -> &'static [&'static str] {
        &["x", "y", "r", "v", "w"]
    }

    fn to_float_features(&self) -> Vec<f64> {
        vec![
            self.position.x,
            self.position.y,
            self.heading,
            self.speed,
            self.angular_velocity,
        ]
    }
}

impl ToFloatFeatures for (Seconds, NonHolonomicDynamics) {
    fn repr() -> &'static [&'static str] {
        <NonHolonomicDynamics as ToFloatFeatures>::repr()
    }

    fn to_float_features(&self) -> Vec<f64> {
        self.1.to_float_features()
    }
}

#[derive(Debug, Component)]
struct LPsiControl {
    leader: Entity,
    l_12_d: Metres,
    psi_12_d: Radians,
}

impl LPsiControl {
    pub fn leader(&self) -> Entity {
        self.leader
    }

    pub fn calculate_control(
        &self,
        follower: &NonHolonomicDynamics,
        leader: &NonHolonomicDynamics,
    ) -> (MetresPerSecond, RadiansPerSecond) {
        let (l_12, psi_12, gamma_1) = leader.calculate_control_parameters(follower);
        let l_12_d = self.l_12_d;
        let psi_12_d = self.psi_12_d;
        let rho_12 = (A1 * (l_12_d - l_12) + leader.speed * psi_12.cos()) / gamma_1.cos();
        let angular_velocity = (gamma_1.cos() / D)
            * (A2 * l_12 * (psi_12_d - psi_12) - leader.speed * psi_12.sin()
                + l_12 * leader.angular_velocity
                + rho_12 * gamma_1.sin());
        let speed = rho_12 - D * angular_velocity * gamma_1.tan();
        (speed, angular_velocity)
    }

    pub fn from_positions(
        follower: &NonHolonomicDynamics,
        leader: &NonHolonomicDynamics,
        leader_entity: Entity,
    ) -> Self {
        let (l_12_d, psi_12_d, _) = leader.calculate_control_parameters(follower);
        LPsiControl {
            leader: leader_entity,
            l_12_d,
            psi_12_d,
        }
    }
}

#[derive(Debug, Component)]
struct LLControl {
    leader1: Entity,
    leader2: Entity,
    l_13_d: Metres,
    l_23_d: Metres,
}

impl LLControl {
    pub fn from_positions(
        follower: &NonHolonomicDynamics,
        (leader1, leader2): (&NonHolonomicDynamics, &NonHolonomicDynamics),
        (leader1_e, leader2_e): (Entity, Entity),
    ) -> Self {
        let (l_13_d, _, _) = leader1.calculate_control_parameters(follower);
        let (l_23_d, _, _) = leader2.calculate_control_parameters(follower);
        LLControl {
            l_13_d,
            l_23_d,
            leader1: leader1_e,
            leader2: leader2_e,
        }
    }

    pub fn leaders(&self) -> (Entity, Entity) {
        (self.leader1, self.leader2)
    }

    pub fn calculate_control(
        &self,
        follower: &NonHolonomicDynamics,
        (leader1, leader2): (&NonHolonomicDynamics, &NonHolonomicDynamics),
    ) -> (MetresPerSecond, RadiansPerSecond) {
        let (l_13, psi_13, gamma_1) = leader1.calculate_control_parameters(follower);
        let (l_23, psi_23, gamma_2) = leader2.calculate_control_parameters(follower);
        let l_13_d = self.l_13_d;
        let l_23_d = self.l_23_d;
        let angular_velocity = (A1 * (l_13_d - l_13) * gamma_2.cos()
            + leader1.speed * psi_13.cos() * gamma_2.cos()
            - A2 * (l_23_d - l_23) * gamma_1.cos()
            - leader2.speed * psi_23.cos() * gamma_1.cos())
            / (D * (gamma_1 - gamma_2).sin());
        let speed = (A1 * (l_13_d - l_13) + leader1.speed * psi_13.cos()
            - D * angular_velocity * gamma_1.sin()) / gamma_1.cos();
        (speed, angular_velocity)
    }
}

#[derive(Debug, Component)]
struct PrescribedControl {
    path: MultiDubinsPath,
    path_length: Seconds,
}

// this is theoretically unsound (since DubinsPaths have Cells), but we know we won't be accessing this component from more than one thread at a time
unsafe impl Sync for PrescribedControl {}

impl PrescribedControl {
    pub fn new(path: MultiDubinsPath) -> Self {
        let path_length = path.length();
        PrescribedControl { path, path_length }
    }

    pub fn sample(&self, t: Seconds, resolution: Seconds) -> NonHolonomicDynamics {
        if t > self.path_length {
            let (position, heading) = self.path.endpoint();
            return NonHolonomicDynamics {
                position,
                heading,
                angular_velocity: 0.,
                speed: 0.,
            };
        }

        let (position, heading) = self.path.sample(t).expect("Invalid t parameter given");
        let (next_t, next_position, next_heading) = {
            let next_t = t + resolution;
            if next_t > self.path_length {
                let (pos, rot) = self.path.endpoint();
                (self.path_length, pos, rot)
            } else {
                let (pos, rot) = self
                    .path
                    .sample(next_t)
                    .expect("Invalid t parameter in next point sampling");
                (next_t, pos, rot)
            }
        };
        let delta_t = next_t - t;
        let speed = (next_position - position).length() / delta_t;
        let angular_velocity = (next_heading - heading) / delta_t;

        NonHolonomicDynamics {
            position,
            heading,
            speed,
            angular_velocity,
        }
    }
}

#[derive(Debug, Component)]
struct RobotId(String);

#[derive(Debug, Component)]
struct TrackedDynamicTrajectory {
    data: UniformDynamicTrajectory,
    resolution_ticks: usize,
}

impl TrackedDynamicTrajectory {
    pub fn feed(&mut self, time: &GlobalUniformTime, dynamics: NonHolonomicDynamics) {
        if time.sim_index() % self.resolution_ticks != 0 {
            return;
        }

        self.data.push((time.sim_time(), dynamics));
    }

    pub fn new(time: &GlobalUniformTime, resolution: Seconds) -> Self {
        let resolution_ticks = (resolution / time.sim_delta()).round() as usize;
        TrackedDynamicTrajectory {
            resolution_ticks,
            data: Vec::new(),
        }
    }

    pub fn into_data(self) -> UniformDynamicTrajectory {
        self.data
    }
}

struct TrackTrajectories;

impl<'a> System<'a> for TrackTrajectories {
    type SystemData = (
        Read<'a, GlobalUniformTime>,
        ReadStorage<'a, NonHolonomicDynamics>,
        WriteStorage<'a, TrackedDynamicTrajectory>,
    );

    fn run(&mut self, (time, dynamics, mut trajectories): Self::SystemData) {
        use specs::Join;

        for (dynamic, trajectory) in (&dynamics, &mut trajectories).join() {
            trajectory.feed(&*time, *dynamic);
        }
    }
}

struct ApplyNonHolonomicDynamics;

impl<'a> System<'a> for ApplyNonHolonomicDynamics {
    type SystemData = (
        WriteStorage<'a, NonHolonomicDynamics>,
        ReadStorage<'a, PrescribedControl>,
        Read<'a, GlobalUniformTime>,
    );

    fn run(&mut self, (mut dynamics, prescribed, time): Self::SystemData) {
        use specs::Join;

        // followers
        for (dynamic, ()) in (&mut dynamics, !&prescribed).join() {
            // todo maybe clamp speed
            dynamic.update(time.sim_delta());
        }

        // leaders
        for (dynamic, control) in (&mut dynamics, &prescribed).join() {
            let new_data = control.sample(time.sim_time(), time.sim_delta());
            *dynamic = new_data;
        }
    }
}

struct DynamicsChange(pub MetresPerSecond, pub RadiansPerSecond);

impl AddAssign<DynamicsChange> for DynamicsChange {
    fn add_assign(&mut self, rhs: DynamicsChange) {
        *self = rhs;
    }
}

impl DynamicsChange {
    pub fn new(data: (f64, f64)) -> Self {
        DynamicsChange(data.0, data.1)
    }
}

struct ApplyControl {
    new_dynamics: ChangeSet<DynamicsChange>,
}

impl ApplyControl {
    pub fn new() -> Self {
        ApplyControl {
            new_dynamics: ChangeSet::new(),
        }
    }
}

impl<'a> System<'a> for ApplyControl {
    type SystemData = (
        Entities<'a>,
        WriteStorage<'a, NonHolonomicDynamics>,
        ReadStorage<'a, LPsiControl>,
        ReadStorage<'a, LLControl>,
        Read<'a, GlobalUniformTime>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut dynamics, lpsi, ll, time) = data;
        use specs::Join;

        for (follower_entity, follower, control) in (&*entities, &dynamics, &lpsi).join() {
            let leader = dynamics.get(control.leader()).unwrap();
            let new_dynamics = DynamicsChange::new(control.calculate_control(follower, leader));
            self.new_dynamics.add(follower_entity, new_dynamics);
        }

        for (follower_entity, follower, control) in (&*entities, &dynamics, &ll).join() {
            let (le1, le2) = control.leaders();
            let leader1 = dynamics.get(le1).unwrap();
            let leader2 = dynamics.get(le2).unwrap();
            let new_dynamics =
                DynamicsChange::new(control.calculate_control(follower, (leader1, leader2)));
            self.new_dynamics.add(follower_entity, new_dynamics);
        }

        for (ref mut dynamic, DynamicsChange(new_speed, new_omega)) in
            (&mut dynamics, &self.new_dynamics).join()
        {
            dynamic.speed = *new_speed;
            dynamic.angular_velocity = *new_omega;
        }

        self.new_dynamics.clear();
    }
}

pub struct NonHolonomicRobotSpec {
    pub id: String,
    pub initial_configuration: OrientedPosition2D,
    pub control: DesaiControl,
}

pub enum DesaiControl {
    Prescribed { path: MultiDubinsPath },
    LPsi { leader: String },
    LL { leaders: (String, String) },
}

pub fn do_desai_simulation(
    robots: Vec<NonHolonomicRobotSpec>,
    sim_resolution: f64,
    track_resolution: f64,
) -> HashMap<String, UniformDynamicTrajectory> {
    let mut world = World::new();
    world.add_resource(GlobalUniformTime::new(sim_resolution));
    let sim_time = 10.;
    let num_robots = robots.len();

    let mut dispatcher = DispatcherBuilder::new()
        .with(ApplyNonHolonomicDynamics, "apply_dynamics", &[])
        .with(ApplyControl::new(), "apply_control", &["apply_dynamics"])
        .with(
            TrackTrajectories,
            "track_trajectories",
            &["apply_dynamics", "apply_control"],
        )
        .build();

    dispatcher.setup(&mut world.res);
    world.register::<RobotId>();

    // add robots

    let robot_entities: HashMap<String, Entity> = robots
        .iter()
        .map(|robot| {
            let dynamics = NonHolonomicDynamics {
                position: robot.initial_configuration.position,
                heading: robot.initial_configuration.rotation,
                speed: 0.,
                angular_velocity: 0.,
            };
            let tracking = {
                let time = world.read_resource::<GlobalUniformTime>();
                TrackedDynamicTrajectory::new(&*time, track_resolution)
            };
            let entity = world
                .create_entity()
                .with(dynamics)
                .with(tracking)
                .with(RobotId(robot.id.clone()))
                .build();
            (robot.id.clone(), entity)
        })
        .collect();

    // add control to the robots

    for spec in robots {
        let entity = *robot_entities.get(&spec.id).unwrap();
        match spec.control {
            DesaiControl::Prescribed { path } => {
                let control = PrescribedControl::new(path);
                world
                    .write_storage::<PrescribedControl>()
                    .insert(entity, control)
                    .expect("Prescribed control already present");
            }
            DesaiControl::LPsi { leader } => {
                // todo possibly make the whole function return a Result in case e.g. ids are incorrect
                let leader_entity = *robot_entities
                    .get(&leader)
                    .expect("L-Psi leader was not found");
                let dynamics = world.read_storage::<NonHolonomicDynamics>();
                let control = LPsiControl::from_positions(
                    dynamics.get(entity).unwrap(),
                    dynamics.get(leader_entity).unwrap(),
                    leader_entity,
                );
                world
                    .write_storage::<LPsiControl>()
                    .insert(entity, control)
                    .expect("L-Psi control already present");
            }
            DesaiControl::LL { leaders } => {
                let (lid1, lid2) = leaders;
                let (le1, le2) = (
                    *robot_entities.get(&lid1).expect("LL leader 1 not found"),
                    *robot_entities.get(&lid2).expect("LL leader 2 not found"),
                );
                let dynamics = world.read_storage::<NonHolonomicDynamics>();
                let ld = (dynamics.get(le1).unwrap(), dynamics.get(le2).unwrap());
                let control =
                    LLControl::from_positions(dynamics.get(entity).unwrap(), ld, (le1, le2));
                world
                    .write_storage::<LLControl>()
                    .insert(entity, control)
                    .expect("L-L control already present");
            }
        }
    }

    // run simulation

    loop {
        dispatcher.dispatch(&mut world.res);
        let mut time = world.write_resource::<GlobalUniformTime>();
        if time.sim_time() >= sim_time {
            break;
        }
        time.tick();
    }

    // extract trajectories

    let mut trajectory_map: HashMap<String, UniformDynamicTrajectory> =
        HashMap::with_capacity(num_robots);
    let mut ids = world.write_storage::<RobotId>();
    let mut trajectories = world.write_storage::<TrackedDynamicTrajectory>();

    use specs::Join;
    for (RobotId(id), trajectory) in (ids.drain(), trajectories.drain()).join() {
        trajectory_map.insert(id, trajectory.into_data());
    }

    trajectory_map
}

#[cfg(test)]
mod tests {
    use super::*;
    use base::*;
    use dubins::MultiDubinsPath;
    use rand::thread_rng;

    #[test]
    fn basic_dubins_sim() {
        let mut rng = thread_rng();
        let origin = OrientedPosition2D::new(0., 0., PI / 2.);
        let left = OrientedPosition2D::new(-2., -2., PI / 2.);
        let right = OrientedPosition2D::new(2., -2., PI / 2.);
        let back = OrientedPosition2D::new(0., -4., PI / 2.);
        let multi = MultiDubinsPath::generate(1., 2., 15., &mut rng, origin, 10.)
            .expect("could not generate");
        let specs = vec![
            NonHolonomicRobotSpec {
                id: "leader".to_string(),
                control: DesaiControl::Prescribed { path: multi },
                initial_configuration: origin,
            },
            NonHolonomicRobotSpec {
                id: "left".to_string(),
                control: DesaiControl::LPsi {
                    leader: "leader".to_string(),
                },
                initial_configuration: left,
            },
            NonHolonomicRobotSpec {
                id: "right".to_string(),
                control: DesaiControl::LPsi {
                    leader: "leader".to_string(),
                },
                initial_configuration: right,
            },
            NonHolonomicRobotSpec {
                id: "back".to_string(),
                control: DesaiControl::LL {
                    leaders: ("left".to_string(), "right".to_string()),
                },
                initial_configuration: back,
            },
        ];

        let results = do_desai_simulation(specs, 1. / 64., 1. / 8.);
        println!("Simulation results: {:?}", results);
    }
}
