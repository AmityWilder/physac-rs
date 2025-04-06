use raylib::prelude::*;
use physac::prelude::*;

fn main() {
    let (mut rl, thread) = raylib::init()
        .size(640, 480)
        .title("Hello, Physics")
        .build();

    let mut ph = init_physics::<24, 48>()
        .gravity_force(0.0, 3.0)
        .build();

    let ball = ph.borrow_mut()
        .create_physics_body_circle(Vector2::new(320.0, 240.0), 45.0, 10.0)
        .clone();

    ball.borrow_mut().restitution = 0.9;
    let ball_id = ball.borrow().id;

    ph.borrow_mut()
        .create_physics_body_rectangle(Vector2::new(320.0, 450.0), 620.0, 40.0, 10.0)
        .borrowed_mut(|floor| {
            floor.enabled = false;
            floor.restitution = 0.9;
        });

    while !rl.window_should_close() {
        if rl.is_key_pressed(KeyboardKey::KEY_SPACE) {
            ball.borrowed_mut(|ball| ball.velocity.y -= 1.0)
        }

        let mut d = rl.begin_drawing(&thread);

        d.clear_background(Color::RAYWHITE);

        for body in ph.borrow().physics_body_iter() {
            let color = if body.id == ball_id {
                Color::DODGERBLUE
            } else {
                Color::BLUEVIOLET
            };

            let vertices = body
                .vertices_iter_closed()
                .rev()
                .collect::<Vec<Vector2>>();

            d.draw_triangle_fan(&vertices, color);
        }
    }
}
