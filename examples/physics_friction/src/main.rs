/*******************************************************************************************
*
*   Physac - Physics friction
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
use physac::{DEG2RAD, *};

fn main() {
    // Initialization
    //--------------------------------------------------------------------------------------
    let screen_width = 800;
    let screen_height = 450;

    let (mut rl, thread) = init()
        .size(screen_width, screen_height)
        .title("[physac] Friction demo")
        .msaa_4x()
        .build();

    // Physac logo drawing position
    let logo_x = screen_width - rl.measure_text("Physac", 30) - 10;
    let logo_y = 15;

    // Initialize physics and default physics bodies
    let mut ph = init_physics::<24>().build();

    // Create floor rectangle physics body
    let floor = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32), screen_width as f32, 100.0, 10.0).unwrap();
    floor.borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    let wall = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32*0.8), 10.0, 80.0, 10.0).unwrap();
    wall.borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)

    // Create left ramp physics body
    let rect_left = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(25.0, screen_height as f32 - 5.0), 250.0, 250.0, 10.0).unwrap();
    rect_left.borrowed_mut(|rect_left| {
        rect_left.enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
        rect_left.set_rotation(30.0*DEG2RAD);
    });

    // Create right ramp  physics body
    let rect_right = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32 - 25.0, screen_height as f32 - 5.0), 250.0, 250.0, 10.0).unwrap();
    rect_right.borrowed_mut(|rect_right| {
        rect_right.enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
        rect_right.set_rotation(330.0*DEG2RAD);
    });

    // Create dynamic physics bodies
    let body_a = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(35.0, screen_height as f32*0.6), 40.0, 40.0, 10.0).unwrap();
    body_a.borrowed_mut(|body_a| {
        body_a.static_friction = 0.1;
        body_a.dynamic_friction = 0.1;
        body_a.set_rotation(30.0*DEG2RAD);
    });

    let body_b = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32 - 35.0, screen_height as f32*0.6), 40.0, 40.0, 10.0).unwrap();
    body_b.borrowed_mut(|body_b| {
        body_b.static_friction = 1.0;
        body_b.dynamic_friction = 1.0;
        body_b.set_rotation(330.0*DEG2RAD);
    });

    rl.set_target_fps(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while !rl.window_should_close() {   // Detect window close button or ESC key
        // Update
        //----------------------------------------------------------------------------------
        // ...
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        {
            let mut d = rl.begin_drawing(&thread);

            d.clear_background(Color::BLACK);

            d.draw_fps(screen_width - 90, screen_height - 30);

            // Draw created physics bodies
            let bodies_count = ph.borrow().get_physics_bodies_count();
            for i in 0..bodies_count {
                if let Some(body) = ph.borrow().get_physics_body(i) {
                    let vertex_count = ph.borrow().get_physics_shape_vertices_count(i).unwrap();
                    for j in 0..vertex_count {
                        // Get physics bodies shape vertices to draw lines
                        // Note: ph.get_physics_shape_vertex() already calculates rotation transformations
                        let vertex_a = ph.borrow().get_physics_body_shape_vertex(&body, j).unwrap();

                        let jj = if (j + 1) < vertex_count { j + 1 } else { 0 };   // Get next vertex or first to close the shape
                        let vertex_b = ph.borrow().get_physics_body_shape_vertex(&body, jj).unwrap();

                        d.draw_line_v(vertex_a, vertex_b, Color::GREEN);     // Draw a line between two vertex positions
                    }
                }
            }

            d.draw_rectangle(0, screen_height - 49, screen_width, 49, Color::BLACK);

            d.draw_text("Friction amount", (screen_width - d.measure_text("Friction amount", 30))/2, 75, 30, Color::WHITE);
            {
                let body_a = body_a.borrow();
                let body_b = body_b.borrow();
                d.draw_text("0.1", body_a.position.x as i32 - d.measure_text("0.1", 20)/2, body_a.position.y as i32 - 7, 20, Color::WHITE);
                d.draw_text("1", body_b.position.x as i32 - d.measure_text("1", 20)/2, body_b.position.y as i32 - 7, 20, Color::WHITE);
            }

            d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
            d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);

        }
        //----------------------------------------------------------------------------------
    }
}
