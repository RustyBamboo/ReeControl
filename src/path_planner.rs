extern crate nalgebra as na;
use na::{Vector3};

pub fn get_winding_path() -> Vec<Vector3::<f32>> {
    let points = vec![Vector3::<f32>::zeros(), Vector3::new(5.0, 5.0, 0.0), Vector3::new(5.0, 5.0, 1.0), Vector3::new(5.0, 0.0, 10.0)];
    let mut d_points = Vec::new();
    for (i, p) in points.iter().enumerate() {
        if i < 1 {
            continue;
        }
        println!("{}", i);
        let last = points[i-1];
        println!("{} {}", last, p);
        let step_vec = p - last; 
        let mag = step_vec.magnitude();
        println!("{}", mag);
        for step in 1..mag as i32 {
            d_points.push(step_vec * step as f32/mag + last);
        }
    }

    let last = points.last().unwrap();
    let step_vec = points[0] - last; 
    let mag = step_vec.magnitude();

    for step in 1..mag as i32 {
            d_points.push(step_vec * step as f32/mag + last);
    }

    return d_points;
}
