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
    let mut screenWidth = 800;
    let mut screenHeight = 450;

    let (mut rl, thread) = init()
        .size(screenWidth, screenHeight)
        .title("[physac] - Body controller demo")
        .msaa_4x()
        .build();

    // Physac logo drawing position
    let mut logo_x = screenWidth - rl.measure_text("Physac", 30) - 10;
    let mut logo_y = 15;

    // Initialize physics and default physics bodies
    let mut ph = init_physics();

    // Create floor and walls rectangle physics body
    let mut floor = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(screenWidth as f32/2.0, screenHeight as f32), screenWidth as f32, 100.0, 10.0)).unwrap();
    let mut platformLeft = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(screenWidth as f32*0.25, screenHeight as f32*0.6), screenWidth as f32*0.25, 10.0, 10.0)).unwrap();
    let mut platformRight = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(screenWidth as f32*0.75, screenHeight as f32*0.6), screenWidth as f32*0.25, 10.0, 10.0)).unwrap();
    let mut wallLeft = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(-5.0, screenHeight as f32/2.0), 10.0, screenHeight as f32, 10.0)).unwrap();
    let mut wallRight = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(screenWidth as f32 + 5.0, screenHeight as f32/2.0), 10.0, screenHeight as f32, 10.0)).unwrap();

    // Disable dynamics to floor and walls physics bodies
    floor.borrow_mut().enabled = false;
    platformLeft.borrow_mut().enabled = false;
    platformRight.borrow_mut().enabled = false;
    wallLeft.borrow_mut().enabled = false;
    wallRight.borrow_mut().enabled = false;

    // Create movement physics body
    let mut body = ph.borrowed_mut(|ph| ph.create_physics_body_rectangle(Vector2::new(screenWidth as f32/2.0, screenHeight as f32/2.0), 50.0, 50.0, 1.0)).unwrap();
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
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        {
            let mut d = rl.begin_drawing(&thread);

            d.clear_background(Color::BLACK);

            d.draw_fps(screenWidth - 90, screenHeight - 30);

            // Draw created physics bodies
            let mut bodies_count = ph.borrow().get_physics_bodies_count();
            for i in 0..bodies_count {
                let mut body = ph.borrow().get_physics_body(i).unwrap();

                let mut vertex_count = ph.borrow().get_physics_shape_vertices_count(i).unwrap();
                for j in 0..vertex_count {
                    // Get physics bodies shape vertices to draw lines
                    // Note: body.get_physics_shape_vertex() already calculates rotation transformations
                    let mut vertex_a = body.borrowed(|body| body.get_physics_shape_vertex(j).unwrap()).unwrap();

                    let mut jj = next_idx(j, vertex_count);   // Get next vertex or first to close the shape
                    let mut vertex_b = body.borrowed(|body| body.get_physics_shape_vertex(jj).unwrap()).unwrap();

                    d.draw_line_v(vertex_a, vertex_b, Color::GREEN);     // Draw a line between two vertex positions
                }
            }

            d.draw_text("Use 'ARROWS' to move player", 10, 10, 10, Color::WHITE);

            d.draw_text("Physac", logo_x, logo_y, 30, Color::WHITE);
            d.draw_text("Powered by", logo_x + 50, logo_y - 7, 10, Color::WHITE);
        }
        //----------------------------------------------------------------------------------
    }
}
