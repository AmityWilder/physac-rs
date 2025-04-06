/*******************************************************************************************
*
*   Physac - Body shatter
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
*   Copyright (c) 2016-2020 Victor Fisac (github: @victorfisac)
*
********************************************************************************************/

use raylib::prelude::*;
use physac::prelude::*;

const SHATTER_FORCE: f32 = 200.0;

fn main() {
    // Initialization
    //--------------------------------------------------------------------------------------
    let screen_width = 800;
    let screen_height = 450;

    let (mut rl, thread) = init()
        .size(screen_width, screen_height)
        .title("[physac] - Shatter demo")
        .msaa_4x()
        .build();

    // Physac logo drawing position
    let logo_x = screen_width - rl.measure_text("Physac", 30) - 10;
    let logo_y = 15;

    // Initialize physics and default physics bodies
    let mut ph = init_physics::<24, 24>().build();
    ph.borrow_mut().set_physics_gravity(0.0, 0.0);

    // Create random polygon physics body to shatter
    let _shatter_body = ph.borrow_mut().create_physics_body_polygon(Vector2::new(screen_width as f32/2.0, screen_height as f32/2.0), rl.get_random_value::<i32>(80..200) as f32, rl.get_random_value::<i32>(3..8) as usize, 10.0);

    rl.set_target_fps(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while !rl.window_should_close()    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        if rl.is_key_pressed(KeyboardKey::KEY_R) {
            drop(ph);

            ph = init_physics::<24, 24>().build();
            ph.borrow_mut().set_physics_gravity(0.0, 0.0);

            ph.borrow_mut().create_physics_body_polygon(Vector2::new(screen_width as f32/2.0, screen_height as f32/2.0), rl.get_random_value::<i32>(80..200) as f32, rl.get_random_value::<i32>(3..8) as usize, 10.0);
        } else if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            // Note: some values need to be stored in variables due to asynchronous changes during main thread
            let count = ph.borrow().get_physics_bodies_count();

            for i in (0..count).rev() {
                let current_body = ph.borrow().get_physics_body(i).downgrade();

                ph.borrow_mut().physics_shatter(current_body, rl.get_mouse_position(), SHATTER_FORCE);
            }
        }

        #[cfg(not(feature = "phys_thread"))]
        ph.borrow_mut().run_physics_step();
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        {
            let mut d = rl.begin_drawing(&thread);

            d.clear_background(Color::BLACK);

            ph.borrowed(|ph| {
                // Draw created physics bodies
                for current_body in ph.physics_body_iter() {
                    let vertex_count = current_body.get_physics_shape_vertices_count();

                    for j in 0..vertex_count {
                        // Get physics bodies shape vertices to draw lines
                        let vertex_a = current_body.get_physics_shape_vertex(j);

                        let jj = next_idx(j, vertex_count);   // Get next vertex or first to close the shape
                        let vertex_b = current_body.get_physics_shape_vertex(jj);

                        d.draw_line_v(vertex_a, vertex_b, Color::GREEN);     // Draw a line between two vertex positions
                    }
                }
            });

            d.draw_text("Left mouse button in polygon area to shatter body", 10, 10, 10, Color::WHITE);
            d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
            d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);

        }
        //----------------------------------------------------------------------------------
    }
}
