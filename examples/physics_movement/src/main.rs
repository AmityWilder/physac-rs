/*******************************************************************************************
*
*   Physac - Physics movement
*
*   NOTE 1: Physac requires multi-threading, when InitPhysics() a second thread
*           is created to manage physics calculations.
*   NOTE 2: Physac requires static C library linkage to avoid dependency
*           on MinGW DLL (-static -lpthread)
*
*   Compile this program using:
*       gcc -o $(NAME_PART).exe $(FILE_NAME) -s ..\icon\physac_icon -I. -I../src
*           -I../src/external/raylib/src -static -lraylib -lopengl32 -lgdi32 -pthread -std=c99
*
*   Copyright (c) 2016-2025 Victor Fisac (github: @victorfisac)
*
********************************************************************************************/

use raylib::prelude::*;
use physac::*;

const VELOCITY: f32 = 0.5;

fn main() {
    // Initialization
    //--------------------------------------------------------------------------------------
    let screen_width = 800;
    let screen_height = 450;

    let (mut rl, thread) = init()
        .size(screen_width, screen_height)
        .title("[physac] - Body controller demo")
        .msaa_4x()
        .build();

    // Physac logo drawing position
    let logo_x = screen_width - rl.measure_text("Physac", 30) - 10;
    let logo_y = 15;

    // Initialize physics and default physics bodies
    let mut ph = init_physics::<24, 24>().build();

    // Create floor and walls rectangle physics body
    let floor = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32), screen_width as f32, 100.0, 10.0);
    let platform_left = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32*0.25, screen_height as f32*0.6), screen_width as f32*0.25, 10.0, 10.0);
    let platform_right = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32*0.75, screen_height as f32*0.6), screen_width as f32*0.25, 10.0, 10.0);
    let wall_left = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(-5.0, screen_height as f32/2.0), 10.0, screen_height as f32, 10.0);
    let wall_right = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32 + 5.0, screen_height as f32/2.0), 10.0, screen_height as f32, 10.0);

    // Disable dynamics to floor and walls physics bodies
    floor.borrow_mut().enabled = false;
    platform_left.borrow_mut().enabled = false;
    platform_right.borrow_mut().enabled = false;
    wall_left.borrow_mut().enabled = false;
    wall_right.borrow_mut().enabled = false;

    // Create movement physics body
    let body = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32/2.0), 50.0, 50.0, 1.0));
    body.borrow_mut().freeze_orient = true;  // Constrain body rotation to avoid little collision torque amounts

    rl.set_target_fps(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while !rl.window_should_close() {    // Detect window close button or ESC key
        // Update
        //----------------------------------------------------------------------------------
        // Horizontal movement input
        if rl.is_key_down(KeyboardKey::KEY_RIGHT) {
            body.borrow_mut().velocity.x = VELOCITY;
        } else if rl.is_key_down(KeyboardKey::KEY_LEFT) {
            body.borrow_mut().velocity.x = -VELOCITY;
        }

        // Vertical movement input checking if player physics body is grounded
        if rl.is_key_down(KeyboardKey::KEY_UP) && body.borrow().is_grounded {
            body.borrow_mut().velocity.y = -VELOCITY*4.0;
        }

        #[cfg(not(feature = "phys_thread"))]
        ph.borrow_mut().run_physics_step();
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        {
            let mut d = rl.begin_drawing(&thread);

            d.clear_background(Color::BLACK);

            d.draw_fps(screen_width - 90, screen_height - 30);

            // Draw created physics bodies
            ph.borrowed(|ph| {
                for body in ph.physics_body_iter() {
                    let vertex_count = body.get_physics_shape_vertices_count();
                    for j in 0..vertex_count {
                        // Get physics bodies shape vertices to draw lines
                        // Note: get_physics_shape_vertex() already calculates rotation transformations
                        let vertex_a = body.get_physics_shape_vertex(j);

                        let jj = next_idx(j, vertex_count);   // Get next vertex or first to close the shape
                        let vertex_b = body.get_physics_shape_vertex(jj);

                        d.draw_line_v(vertex_a, vertex_b, Color::GREEN);     // Draw a line between two vertex positions
                    }
                }
            });

            d.draw_text("Use 'ARROWS' to move player", 10, 10, 10, Color::WHITE);

            d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
            d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);
        }
        //----------------------------------------------------------------------------------
    }
}
