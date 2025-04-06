/*******************************************************************************************
*
*   Physac - Physics demo
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
        .title("[physac] Basic demo")
        .msaa_4x()
        .build();

    // Physac logo drawing position
    let logo_x = screen_width - rl.measure_text("Physac", 30) - 10;
    let logo_y = 15;

    // Initialize physics and default physics bodies
    let mut ph = init_physics::<24, 24>().build();

    // Create floor rectangle physics body
    ph.borrow_mut()
        .create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32), 500.0, 100.0, 10.0)
        .borrowed_mut(|floor| floor.enabled = false); // Disable body state to convert it to static (no dynamics, but collisions)

    // Create obstacle circle physics body
    ph.borrow_mut()
        .create_physics_body_circle(Vector2::new(screen_width as f32/2.0, screen_height as f32/2.0), 45.0, 10.0)
        .borrowed_mut(|circle| circle.enabled = false); // Disable body state to convert it to static (no dynamics, but collisions)

    rl.set_target_fps(60);

    //--------------------------------------------------------------------------------------

    // Main game loop
    while !rl.window_should_close() {    // Detect window close button or ESC key
        // Update
        //----------------------------------------------------------------------------------
        // Physics body creation inputs
        if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            ph.borrow_mut().create_physics_body_polygon(rl.get_mouse_position(), rl.get_random_value::<i32>(20..80) as f32, rl.get_random_value::<i32>(3..8) as usize, 10.0);
        } else if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_RIGHT) {
            ph.borrow_mut().create_physics_body_circle(rl.get_mouse_position(), rl.get_random_value::<i32>(10..45) as f32, 10.0);
        }

        // Destroy falling physics bodies
        ph.borrow_mut().destroy_physics_bodies(|body| body.position.y > (screen_height*2) as f32);

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

            d.draw_text("Left mouse button to create a polygon", 10, 10, 10, Color::WHITE);
            d.draw_text("Right mouse button to create a circle", 10, 25, 10, Color::WHITE);

            d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
            d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);
        }
        //----------------------------------------------------------------------------------
    }
}
