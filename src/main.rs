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
    fn update(&mut self, drag:&Drag, a:Vector6<f64>) -> () {
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
    fn update_camera(&mut self, camera: &mut ArcBall) {
        camera.set_at(Point3::new(self.position.vector.x, self.position.vector.y, self.position.vector.z));
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
    window.set_light(Light::StickToCamera);
    draw_grid(&mut window);

    let mut camera = ArcBall::new(Point3::new(0.0, 2.0, -5.0), Point3::new(0.0, 0.0, 0.0)); 

    let mut sub = Sub::new(&mut window);
    sub.acceleration = Translation3::new(0.0001, 0.0, 0.0);
    
    let drag = Drag {velocity:0.001, angular_velocity:0.001};

    let b_matrix = control::get_b_matrix();
    println!("B Matrix: {}", b_matrix);
    let mut thrust :Vector8<f64> = Vector8::<f64>::repeat(0.0);
    thrust[1] = 0.1; 
    println!("{}", thrust);
    let out = b_matrix * thrust;
    println!("{}", out);

    while window.render_with_camera(&mut camera) {
         sub.update(&drag, out);
         sub.update_camera(&mut camera);
         draw_grid(&mut window);
    }
}
