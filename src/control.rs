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
    position: Vec<f64>,
    direction: Vec<f64>,
    thrust_bounds: Vec<f64>,
    calib: Value
}

#[derive(Deserialize)]
struct ThrusterBounds {
    forward: Vec<f64>,
    backward: Vec<f64>
}

// "System" is a nonlinear/linear function that describes motion. Namely, system = f(x,u) where x
// is some state vector, and u is our input vector. In this case, we assume u is mapped via a
// linear function, y = Bu, where B represents a control matrix.
// Thus, system(x,u) = f(x) + Bu
pub fn get_b_matrix() -> na::MatrixMN<f64, na::U6, na::U8> {
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
    na::MatrixMN::<f64, na::U6, na::U8>::from_columns(&cols)
}

pub fn pid(current: (Vector3<f64>, UnitQuaternion<f64>), desired: (Vector3<f64>, UnitQuaternion<f64>)) -> () {
    let (current_position, current_orientation) = current;
    let (desired_position, desired_orientation) = desired;

}
