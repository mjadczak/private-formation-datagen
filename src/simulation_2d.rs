use base::*;
use specs::prelude::*;
use std::f64::consts::PI;
use std::ops::AddAssign;
use std::ops::Index;

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

const D: Metres = 0.5;
const A1: f64 = 0.25;
const A2: f64 = 0.25;

type UniformDynamicTrajectory = Vec<(Seconds, NonHolonomicDynamics)>;

/// Contains the uniform resolution index, as well as the actual clock time, for assertions
#[derive(Default)]
struct GlobalUniformTime {}

impl GlobalUniformTime {
    pub fn sim_delta(&self) -> Seconds {
        unimplemented!();
    }

    pub fn sim_time(&self) -> Seconds {
        unimplemented!();
    }

    pub fn sim_index(&self) -> usize {
        unimplemented!();
    }
}

#[derive(Debug, Component, Copy, Clone)]
struct NonHolonomicDynamics {
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

    pub fn set_from(&mut self, other: &Self) {
        *self = other.clone()
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

#[derive(Debug, Component, Default)]
#[storage(NullStorage)]
struct Follower;

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
}

#[derive(Debug, Component)]
struct LLControl {
    leader1: Entity,
    leader2: Entity,
    l_13_d: Metres,
    l_23_d: Metres,
}

impl LLControl {
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
struct PrescribedControl(UniformDynamicTrajectory);

impl Index<usize> for PrescribedControl {
    type Output = (Seconds, NonHolonomicDynamics);

    fn index(&self, index: usize) -> &<Self as Index<usize>>::Output {
        &self.0[index]
    }
}

#[derive(Debug, Component)]
#[storage(VecStorage)]
struct RobotId(u32);

#[derive(Debug, Component)]
struct TrackedDynamicTrajectory(UniformDynamicTrajectory);

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
        let sim_index = time.sim_index();
        let sim_time = time.sim_time();

        for (dynamic, control) in (&mut dynamics, &prescribed).join() {
            let (t, new_data) = control[sim_index];
            debug_assert_eq!(t, sim_time);
            dynamic.set_from(&new_data);
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
