extern crate serde_yaml;
use serde_yaml::Value;
use std::collections::HashMap;
extern crate nalgebra as na;
use na::{Vector3, Matrix, UnitQuaternion};

/* Helper structs for thruster_mapper.yaml
 * Mostly temporary to test code
 */
#[derive(Deserialize)]
struct ThrusterYaml {
     thruster_ports: Vec<Value>,
     thrusters: HashMap<String, Thruster>
}
#[derive(Deserialize)]
struct Thruster {
    node_id: u32,
    position: Vec<f32>,
    direction: Vec<f32>,
    thrust_bounds: Vec<f32>,
    calib: Value
}

#[derive(Deserialize)]
struct ThrusterBounds {
    forward: Vec<f32>,
    backward: Vec<f32>
}

// "System" is a nonlinear/linear function that describes motion. Namely, system = f(x,u) where x
// is some state vector, and u is our input vector. In this case, we assume u is mapped via a
// linear function, y = Bu, where B represents a control matrix.
// Thus, system(x,u) = f(x) + Bu
pub fn get_b_matrix() -> na::MatrixMN<f32, na::U6, na::U8> {
    let file = std::fs::File::open("thruster_layout.yaml").expect("file should open read only");
    let yaml: ThrusterYaml = serde_yaml::from_reader(file).expect("file should be proper JSON");

    let mut cols = Vec::new();
    let mut bounds = Vec::new();

    let thruster_order = vec!["FLH", "FLV", "FRH", "FRV", "BLH", "BLV", "BRH", "BRV"];

    for name in thruster_order.iter() {
        let value = yaml.thrusters.get::<str>(name).unwrap();

        let position = Vector3::new(value.position[0], value.position[1], value.position[2]);
        let direction = Vector3::new(value.direction[0], value.direction[1], value.direction[2]);
        
        let torque = position.cross(&direction);
        println!("Thruster: {}, position: {:#?}, direction: {:#?}, torque: {:#?}, bounds: {:#?}", name, position, direction, torque, value.thrust_bounds);
        let column =  na::Vector6::new(value.direction[0], value.direction[1], value.direction[2], torque[0], torque[1], torque[2]);
 
        cols.push(column);
        bounds.push(&value.thrust_bounds);
    }
    na::MatrixMN::<f32, na::U6, na::U8>::from_columns(&cols)
}

pub struct PID {
   pub error:na::Vector6<f32>
}
impl PID {
pub fn pid_loop(&mut self, current: (Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>, Vector3<f32>) , desired: (Vector3<f32>, Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>, Vector3<f32>, Vector3<f32>)) -> na::Vector6<f32> {
    let (c_position, c_velocity, c_orientation, c_angular_velocity) = current;
    let (d_position, d_velocity, d_acceleration, d_orientation, d_angular_velocity, d_angular_acceleration) = desired;
    let err_position = d_position - c_position;
    let err_orientation = (d_orientation * c_orientation.inverse()).scaled_axis();
    let err_position_orientation = na::Vector6::new(err_position[0], err_position[1], err_position[2], err_orientation[0], err_orientation[1], err_orientation[2]);
    self.error += err_position_orientation;

    return 0.001 * err_position_orientation + 0.00001 * self.error;
}
}
