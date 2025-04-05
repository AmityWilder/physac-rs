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
    // // Initialization
    // //--------------------------------------------------------------------------------------
    // let mut screen_width = 800;
    // let mut screen_height = 450;

    // let (mut rl, thread) = init()
    //     .size(screen_width, screen_height)
    //     .title("[physac] Friction demo")
    //     .msaa_4x()
    //     .build();

    // // Physac logo drawing position
    // let mut logo_x = screen_width - rl.measure_text("Physac", 30) - 10;
    // let mut logo_y = 15;

    // // Initialize physics and default physics bodies
    // let mut ph = init_physics::<
    //     DEFAULT_MAX_BODIES,
    //     DEFAULT_MAX_MANIFOLDS,
    //     DEFAULT_MAX_VERTICES,
    //     DEFAULT_COLLISION_ITERATIONS,
    // >();

    // // Create floor rectangle physics body
    // let mut floor = ph.create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32), screen_width as f32, 100.0, 10.0);
    // floor.upgrade().unwrap().borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    // let mut wall = ph.create_physics_body_rectangle(Vector2::new(screen_width as f32/2.0, screen_height as f32*0.8), 10.0, 80.0, 10.0);
    // wall.upgrade().unwrap().borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)

    // // Create left ramp physics body
    // let mut rect_left = ph.create_physics_body_rectangle(Vector2::new(25.0, screen_height as f32 - 5.0), 250.0, 250.0, 10.0);
    // rect_left.upgrade().unwrap().borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    // rect_left.set_rotation(30.0*DEG2RAD);

    // // Create right ramp  physics body
    // let mut rect_right = ph.create_physics_body_rectangle(Vector2::new(screen_width as f32 - 25.0, screen_height as f32 - 5.0), 250.0, 250.0, 10.0);
    // rect_right.upgrade().unwrap().borrow_mut().enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    // rect_right.set_rotation(330.0*DEG2RAD);

    // // Create dynamic physics bodies
    // let mut body_a = ph.create_physics_body_rectangle(Vector2::new(35.0, screen_height as f32*0.6), 40.0, 40.0, 10.0).upgrade().unwrap();
    // body_a.borrow_mut().static_friction = 0.1;
    // body_a.borrow_mut().dynamic_friction = 0.1;
    // body_a.set_rotation(30.0*DEG2RAD);

    // let mut body_b = ph.create_physics_body_rectangle(Vector2::new(screen_width as f32 - 35.0, screen_height as f32*0.6), 40.0, 40.0, 10.0).upgrade().unwrap();
    // body_b.borrow_mut().static_friction = 1.0;
    // body_b.borrow_mut().dynamic_friction = 1.0;
    // body_b.set_rotation(330.0*DEG2RAD);

    // rl.set_target_fps(60);
    // //--------------------------------------------------------------------------------------

    // // Main game loop
    // while !rl.window_should_close() {   // Detect window close button or ESC key
    //     // Update
    //     //----------------------------------------------------------------------------------
    //     // ...
    //     //----------------------------------------------------------------------------------

    //     // Draw
    //     //----------------------------------------------------------------------------------
    //     {
    //         let mut d = rl.begin_drawing(&thread);

    //         d.clear_background(Color::BLACK);

    //         d.draw_fps(screen_width - 90, screen_height - 30);

    //         // Draw created physics bodies
    //         let mut bodies_count = ph.get_physics_bodies_count();
    //         for i in 0..bodies_count {
    //             if let Some(body) = ph.get_physics_body(i) {
    //                 let mut vertex_count = ph.get_physics_shape_vertices_count(i).unwrap();
    //                 for j in 0..vertex_count {
    //                     // Get physics bodies shape vertices to draw lines
    //                     // Note: ph.get_physics_shape_vertex() already calculates rotation transformations
    //                     let mut vertex_a = body.get_physics_shape_vertex(j).unwrap();

    //                     let mut jj = if (j + 1) < vertex_count { j + 1 } else { 0 };   // Get next vertex or first to close the shape
    //                     let mut vertex_b = body.get_physics_shape_vertex(jj).unwrap();

    //                     d.draw_line_v(vertex_a, vertex_b, Color::GREEN);     // Draw a line between two vertex positions
    //                 }
    //             }
    //         }

    //         d.draw_rectangle(0, screen_height - 49, screen_width, 49, Color::BLACK);

    //         d.draw_text("Friction amount", (screen_width - d.measure_text("Friction amount", 30))/2, 75, 30, Color::WHITE);
    //         d.draw_text("0.1", body_a.position.x - d.measure_text("0.1", 20)/2, body_a.position.y - 7, 20, Color::WHITE);
    //         d.draw_text("1", body_b.position.x - d.measure_text("1", 20)/2, body_b.position.y - 7, 20, Color::WHITE);

    //         d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
    //         d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);

    //     }
    //     //----------------------------------------------------------------------------------
    // }
}
