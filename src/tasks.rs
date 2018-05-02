use std::collections::HashMap;

#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioSpec {
    name: String,
    working_dir: String,
    robot: RobotSpec,
    resolution: f64,
    length: f64,
    dimensions: usize,
    formations: FormationSpec,
    reference_trajectories: ReferenceTrajectorySpec,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RobotSpec {
    max_speed: f64,
    controller: ControllerSpec,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
pub enum ControllerSpec {
    P {
        p_gain: f64
    },
    PID {
        p_gain: f64,
        i_gain: f64,
        d_gain: f64,
    },
}

#[derive(Debug, Clone, Deserialize)]
pub struct FormationSpec {
    generator: String,
    params: ParamsSpec,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case", tag = "type")]
pub enum ParamsSpec {
    Random { values: HashMap<String, RandomParamSpec> },
    Constant { values: HashMap<String, ConstantParamSpec> },
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case", tag = "dist")]
pub enum RandomParamSpec {
    Uniform {
        range: (f64, f64)
    },
    Normal {
        range: (f64, f64)
    },
}

#[derive(Debug, Clone, Deserialize)]
pub enum ConstantParamSpec {
    Float(f64)
}

#[derive(Debug, Clone, Deserialize)]
pub struct ReferenceTrajectorySpec {
    num_sets: usize,
    num_per_set: usize,
    generator: String,
    params: ParamsSpec,
}