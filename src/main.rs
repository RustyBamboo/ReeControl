#[macro_use]
extern crate serde_derive;

extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::window::{Window};
use kiss3d::camera::ArcBall;
use kiss3d::scene::SceneNode;
use na::{UnitQuaternion, Vector, Vector3, Vector6, Translation3, Point3};

type Vector8<N> = na::VectorN<N, na::U8>;

mod control;
mod path_planner;

fn draw_grid(window: &mut Window) {
    let size : f32 = 100.0;
    for x in -size as i32..size as i32 {
        window.draw_line(&Point3::new(x as f32, size, 0.0), &Point3::new(x as f32, -size, 0.0), &Point3::new(255.0,255.0,255.0));
        window.draw_line(&Point3::new(size, x as f32, 0.0), &Point3::new(-size, x as f32, 0.0), &Point3::new(255.0,255.0,255.0));
        //window.draw_line(&Point3::new(10.0, 0.0, x as f32), &Point3::new(-10.0, 0.0, x as f32), &Point3::new(255.0, 255.0, 255.0));
    }
}

struct Drag {
    velocity : f32,
    angular_velocity: f32
}

struct Sub {
    position: Translation3<f32>,
    velocity: Translation3<f32>,
    acceleration: Translation3<f32>,
    angular_position: UnitQuaternion<f32>,
    angular_velocity: Vector3<f32>,
    angular_acceleration: Vector3<f32>,

    c: SceneNode
}

impl Sub {
    // Update object by integrating linear and angular components
    fn update(&mut self, drag:&Drag, a:Vector6<f32>) -> () {
        // Apply linear acceleration
        let apply_acc = Translation3::new(a[0] as f32, a[1] as f32, a[2] as f32);
        self.velocity.vector += self.acceleration.vector + self.angular_position.to_rotation_matrix() * apply_acc.vector;
        self.position.vector += drag.velocity * (self.angular_position.to_rotation_matrix() *self.velocity.vector);

        // Apply angular 
        let axis_vec = Vector3::new(a[3] as f32, a[4] as f32, a[5] as f32); 
        self.angular_acceleration += axis_vec;

        // cross product is linear
        self.angular_velocity += self.angular_acceleration;
        // Apply angular drag
        self.angular_velocity *= drag.angular_velocity;

        // Don't attempt if small rotation
        if self.angular_velocity.magnitude().abs() > 0.0001 {
            let mut quat_vel = na::UnitQuaternion::from_axis_angle(&na::Unit::new_unchecked(self.angular_velocity.normalize()), self.angular_velocity.magnitude()); 
            self.angular_position *= quat_vel;
        }

        //self.c.set_local_rotation(self.angular_position);
        //self.c.append_translation(&self.position);
        let iso = na::Isometry3::from_parts(self.position, self.angular_position);
        self.c.set_local_transformation(iso);
    }

    fn get_current(&self) -> (Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>, Vector3<f32>) {
        (self.position.vector, self.velocity.vector, self.angular_position, self.angular_velocity)
    }

    fn update_camera(&mut self, camera: &mut ArcBall) {
        camera.set_at(Point3::new(self.position.vector.x as f32, self.position.vector.y as f32, self.position.vector.z as f32));
    }

    // Create new cube
    fn new(window:&mut Window) -> Self {
        let mut c = window.add_cube(1.0, 1.0, 1.0);
        c.set_color(1.0, 0.0, 0.0);
        Sub{position:Translation3::new(0.0, 0.0, 0.0), velocity:Translation3::new(0.0, 0.0, 0.0), acceleration:Translation3::new(0.0, 0.0, 0.0),
            angular_position:UnitQuaternion::identity(), angular_velocity:Vector3::zeros(), angular_acceleration:Vector3::zeros(), c: c} 
    }
}

fn main() {
    let mut window = Window::new("REESim");
    let font = kiss3d::text::Font::new(std::path::Path::new("/usr/share/fonts/TTF/DejaVuSansMono.ttf"));
    if !font.is_some() {
        println!("oof");
        return;
    }
    window.set_light(Light::StickToCamera);
    draw_grid(&mut window);

    let mut camera = ArcBall::new(Point3::new(0.0, 2.0, -5.0), Point3::new(0.0, 0.0, 0.0)); 

    let mut sub = Sub::new(&mut window);
    //sub.acceleration = Translation3::new(0.0001, 0.0, 0.0);
    
    let drag = Drag {velocity:0.00001, angular_velocity:0.00001};

    let b_matrix = control::get_b_matrix();
    println!("B Matrix: {}", b_matrix);
    let mut thrust :Vector8<f32> = Vector8::<f32>::repeat(0.0);
    thrust[1] = 0.1; 
    println!("{}", thrust);
    let out = b_matrix * thrust;
    println!("{}", out);

    let b_matrix_inv = b_matrix.pseudo_inverse(0.001);
    println!("B Matrix Inverse: {}", b_matrix_inv);

    let mut desired = (Vector3::identity() * 5.0,
                   Vector3::identity() * 0.01,
                   Vector3::identity() * 0.01,
                   UnitQuaternion::<f32>::from_axis_angle(&Vector3::x_axis(), 0.5),
                   Vector3::<f32>::identity(),
                   Vector3::<f32>::identity());
    
    let mut pid = control::PID {error: Vector6::<f32>::zeros(), last_error: Vector6::<f32>::zeros()};

    let path = path_planner::get_winding_path();
    let mut step = 0.0;

    let mut path_cube = window.add_cube(0.2, 0.2, 0.2);
    path_cube.set_color(0.0, 0.2, 1.0);

    while window.render_with_camera(&mut camera) {
        desired.0 = path[step as usize]; 
        println!("{}", desired.0);
        desired.3 = UnitQuaternion::rotation_between(&Vector3::identity(), &desired.0).unwrap();
        let current = sub.get_current(); 
        let wrench = pid.pid_loop(current, desired); 
        let thrust = b_matrix * (b_matrix_inv * wrench); 
        println!("Thrusters: {}", b_matrix_inv * wrench);
        println!("Required Acceleration: {}", thrust);
        sub.update(&drag, thrust);
        sub.update_camera(&mut camera);
        draw_grid(&mut window);

        path_cube.set_local_translation(Translation3::new(desired.0[0], desired.0[1], desired.0[2]));
        if (step as usize) < path.len() -1 {
            step +=0.01;
        }
        else {
            step = 0.0;
        }
        //window.draw_text(&text, &na::Point2::new(5.0, 5.0), 100.0, font.as_ref().unwrap(), &na::Point3::new(255.0, 0.0, 0.0));
    }
}
