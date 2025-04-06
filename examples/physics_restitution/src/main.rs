/*******************************************************************************************
*
*   Physac - Physics restitution
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
use physac::prelude::*;

fn main() {
    // Initialization
    //--------------------------------------------------------------------------------------
    let screen_width = 800;
    let screen_height = 450;

    let (mut rl, thread) = init()
        .size(screen_width, screen_height)
        .title("[physac] - Restitution demo")
        .build();

    // Physac logo drawing position
    let logo_x = screen_width - rl.measure_text("Physac", 30) - 10;
    let logo_y = 15;

    // Initialize physics and default physics bodies
    let mut ph = init_physics::<24, 24>().build();

    // Create floor rectangle physics body
    let floor = ph.borrow_mut().create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32), screen_width as f32, 100.0, 10.0);
    floor.borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    floor.borrow_mut().restitution = 0.9;

    // Create circles physics body
    let circle_a = ph.borrow_mut().create_physics_body_circle(Vector2::new(screen_width as f32*0.25, screen_height as f32/2.0), 30.0, 10.0);
    circle_a.borrow_mut().restitution = 0.0;
    let circle_b = ph.borrow_mut().create_physics_body_circle(Vector2::new(screen_width as f32*0.5, screen_height as f32/2.0), 30.0, 10.0);
    circle_b.borrow_mut().restitution = 0.5;
    let circle_c = ph.borrow_mut().create_physics_body_circle(Vector2::new(screen_width as f32*0.75, screen_height as f32/2.0), 30.0, 10.0);
    circle_c.borrow_mut().restitution = 0.9;

    rl.set_target_fps(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while !rl.window_should_close() {    // Detect window close button or ESC key
        // Update
        //----------------------------------------------------------------------------------
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

            d.draw_text("Restitution amount", (screen_width - d.measure_text("Restitution amount", 30))/2, 75, 30, Color::WHITE);
            d.draw_text("0%", circle_a.borrow().position.x as i32 - d.measure_text("0%", 20)/2, circle_a.borrow().position.y as i32 - 7, 20, Color::WHITE);
            d.draw_text("50%", circle_b.borrow().position.x as i32 - d.measure_text("50%", 20)/2, circle_b.borrow().position.y as i32 - 7, 20, Color::WHITE);
            d.draw_text("90%", circle_c.borrow().position.x as i32 - d.measure_text("90%", 20)/2, circle_c.borrow().position.y as i32 - 7, 20, Color::WHITE);

            d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
            d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);

        }
        //----------------------------------------------------------------------------------
    }
}
