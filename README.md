<table style="border:0">
<tr>
<td>

# physac-rs

physac-rs is a Rust-native translation of [Physac][] with built-in support for [raylib-rs][]. Raylib is not *required* however, and can be disabled by removing the `raylib` feature flag in your `Cargo.toml` file.

See the examples directory for usage examples converted 1:1 from the original C implementation of Physac.

While this library tries to mirror the C API, some changes have been made in order to improve soundness, and to shorten the names of certain methods where they are implied by the name of the type they are implemented for.

</td>
</tr>
</table>

- Resources are automatically cleaned up when they go out of scope (or when [`std::mem::drop`][drop] is called). This means that `ClosePhysics` is not exposed and not necessary.

  `destroy_physics_body()` is still required for unloading physics bodies though, because the `create` methods store "strong" references (either [`std::sync::Arc`][Arc] if the `sync` feature flag is enabled or [`std::rc::Rc`][Rc] if not) inside of `Physac` and only return a reference *to those* references.

- Most of the Physac API is exposed through `Physac`, which is used for storing all fields which the original C implementation has as static globals. This ensures thread safety, and can even allow multiple independent instances of Physac to safely run in the same program simultaneously.

- If the `sync` feature flag is enabled (which is the default), `Physac` is borrowed through the `PhysacHandle` to prevent race conditions between threads. If the `sync` feature flag is not enabled, `PhysacHandle` will implement `std::ops::DerefMut`, which allows `Physac` to be accessed directly by reference without borrowing (the borrow functions will still be available in case you would like to write your code in a way that works both with *and* without the `sync` feature flag).

- A `PhysacHandle` can be obtained through the `init_physics()` function, which will allow you to `build` the physics environment with some of the settings normally provided to Physac through `#define`s.

  Most of these values can be provided by chaining methods. However, while `MAX_VERTICES` and `CIRCLE_VERTICES` *have* default values, explicit values for both constants are unfortunately required at the moment to be provided to the `init_physics()` function through "turbofish" (`::<>`) syntax (ex: `init_physics::<24, 24>()`).

- The fixed-capacity arrays that store `PhysicsBody`s and `PhysicsManifold`s have been replaced with growable `Vec`s, whose initial capacities are set in `init_physics` with the `max_bodies()` and `max_manifolds()` chain methods respectively.

- Manually closing physics is not necessary, because it will automatically close when `PhysacHandle` goes out of scope or otherwise drops (such as with [`std::mem::drop`][drop] or during unwinding). If the `phys_thread` feature flag is enabled, the physics thread will also finish & join when this happens.

  The physics thread can also finish if any unrecoverable errors occur on that thread, such as running out of IDs or a resource being poisoned (i.e. another thread panicks while mutably borrowing a physics body or `Physac`).

- See the section on [Thread Safety](#thread-safety) for important information about one of the more influencial differences between the C and Rust implementations of Physac.

# Build Dependencies

Requires the Rust standard library. Optionally requires [raylib-rs][] by default.

1. Add the dependency to your `Cargo.toml`

```toml
[dependencies]
raylib = { version = "*" } # optional
physac = { version = "0.1" }
```

2. Start coding

```rs
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
```
See the [hello_physics](./examples/hello_physics/) example to run the code above.

# Features

| Feature Flag  | Description                                                                                                                                                                                                                               | Default                               |
|:-------------:|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:-------------------------------------:|
| `raylib`      | Use raylib-rs in the library. If disabled, data types defined on raylib are defined internally in the library and input management and drawing functions must be provided by the user (check library implementation for further details). | Enabled                               |
| `sync`        | The library will use `std::sync` instead of `std::rc` and `std::cell`.                                                                                                                                                                    | Enabled transitively by `phys_thread` |
| `phys_thread` | If disabled, the library won't use `std::thread` and user must create a secondary thread to call `run_physics_step()`, or call it in the main thread if `sync` is disabled. <br/> Requres and automatically enables `sync`.               | Enabled                               |
| `debug`       | Traces log messages when creating and destroying physics bodies and detects errors in physics calculations and reference exceptions; it is useful for debug purposes.                                                                     | Disabled                              |

# Thread Safety

Physac is inherently multithreading-compatible. In the C implementation, this is accomplished with statics and raw pointers. In the Rust version, some additional steps are needed to ensure safeness. This is accomplished with [`std::sync::Arc`][Arc], [`std::sync::Weak`][syncWeak], and [`std::sync::RwLock`][RwLock] (which have been abstracted into `Strong` and `Weak` to enable compatibility between the multithreaded (`sync`) and single-threaded (`rc`) implementations).

---

If you want to "remember" a particular `PhysicsBody` across multiple frames (like if it the body *of* some object, or is part of a physics constraint), use a `Strong<PhysicsBody>` or `Weak<PhysicsBody>`.

`Strong` and `Weak` references can be `clone()`'d to obtain duplicate references to the same physics body. The body they refer to can be temporarily accessed with `borrow()`, `borrow_mut()`, `borrowed()`, `borrowed_mut()`, `read()`, or `write()`.

- `borrow()` and `borrow_mut()` return temporary `Read/WriteGuard`s, borrowing the body until the guard goes out of scope or gets dropped with [`std::mem::drop()`][drop].
- `borrowed()` and `borrowed_mut()` borrow the body for the duration of the closure you pass into them, allowing you to access the body by `&`-reference instead of through a guard.
- `read()` and `write()` are identical to `borrow()` and `borrow_mut()` respectively--except instead of panicking in the case of poisoning, they return the poison error itself. \
  **Note:** `read()` and `write()` *do* still panic if Rust's "no-mutable-aliases" rule is broken. They only make *poison* errors recoverable.

If another thread is currently borrowing the body, these functions will block (wait) until the other thread is finished borrowing it. If you try to borrow a body while it is *already* borrowed by the same thread, the program may panic.
```rs
let body_a = body.borrow_mut();
let body_b = body.borrow_mut(); // `body` is already borrowed in `body_a`
```
See [`std::rc::Rc`][Rc] and [`std::sync::Arc`][Arc] for more information about this behavior.

- `Strong`: Use a strong reference if the body *should not* end during the lifetime of the reference. Note that this does not *enforce* such behavior: holding onto a `Strong` reference after calling `destroy_physics_body` on the body it refers to does not *keep it* in `Physac`, it only makes it so that you don't need to call `upgrade()` on it, and allows the body's information to stay alive after it has been removed from the simulation. \
  Call `downgrade()` on a `Strong` reference to get a `Weak` reference to the same object.

- `Weak`: Use a weak reference if it is *possible* for the body to be destroyed while the reference exists, and you want your code to be conditional on whether that has happened. \
  Call `upgrade()` on a `Weak` reference to get a `Strong` reference to the same object, then let the `Strong` reference go out of scope when you are done accessing it. \
  See [`std::rc::Weak`][rcWeak] and [`std::sync::Weak`][syncWeak] for more information about this behavior.

---

**Important:** Remember not to **borrow** physics bodies, nor `Physac`, across multiple frames. Storing `Strong` and `Weak` references across frames is fine, just not `Read/WriteGuard`s.

When the physics thread needs to borrow a physics body, it will block (wait its turn) until no thread is borrowing it exclusively. The physics thread will mutably borrow *every* physics body being simulated at multiple points during each physics step.

If you are borrowing a physics body for longer than a frame, there's a high likelihood that the physics thread will borrow another body that you need for rendering the frame, and hold onto it until you give up the one you're borrowing. This can lead to a *deadlock*, where the main thread can't borrow a particular physics body until physics thread is finished borrowing it and the physics thread can't finish borrowing it until you finish borrowing yours. This can cause the program to freeze and stop responding, because the main thread needs to finish rendering in order for the program to stay responsive.

It is perfectly reasonable to borrow `Physac` and/or one or multiple `PhysicBody`s for the full duration of a function--as long as the borrow(s) are given up during the same frame. \
**You *should not* store `Read/WriteGuard`s in variables that will persist across multiple frames.**

## Debugging Tips

If your program is encountering physics-related bugs, try disabling `sync` & `phys_thread` and call `run_physics_step()` on your main thread. That way, you can put a breakpoint on the main thread and step into the physics function to see where the problem is occurring.

If this seems to magically solve the problem, the bug might have something to do with how the physics thread isn't synchronized with your mutations on physics bodies.
If you have a lot of individual borrows of `Physac` and/or `PhysicsBodyData`s, try borrowing them for the entire duration of whatever operation you are performing and see if that fixes it.

### Example

Before
```rs
ph.borrow_mut().do_thing();
// because Physac is only borrowed long enough to do_thing(), the borrow will drop when it's finished
// and the physics thread will be free to update simultaneously with the following lines.
if body1.borrow().some_condition() {
    // the physics thread *may have* modified body1 in the split-second since some_condition was tested
    body1.borrow_mut().modification1();
    // the physics thread *may have* modified body1 in the split-second since modification1() occurred
    body1.borrow_mut().modification2(); 
}
```
After
```rs
ph.borrowed_mut(|ph| {
    // borrowed_mut locks ph, blocking the physics thread, until this closure finishes.
    ph.do_thing();
    body1.borrow_mut(|body1| {
        // even though the physics thread can't do anything while Physac is borrowed, other threads with access
        // to your physics bodies may still try to borrow them. by borrowing them for the entire time you need
        // uninterrupted access to them, you can ensure they won't be modified elsewhere.
        if body1.some_condition() {
            body1.modification1();
            body1.modification2();
        }
    }); // borrow of body1 drops, so other threads are free to use it again.
}); // borrow of ph drops, so the physics thread is free to update again.
```
After (using `borrow_mut()` instead of `borrowed_mut()`, so that you can still `return`--but be careful not to let borrows overstay their welcome!)
```rs
{
    let mut ph = ph.borrow_mut();
    // note that this does not necessarily guarantee ph is still borrowed after this line like borrowed_mut() would
    ph.do_thing();
    {
        let mut body1 = body1.borrow_mut();
        if body1.some_condition() {
            body1.modification1();
            body1.modification2();
        }
    }
}
```

[Physac]: https://github.com/victorfisac/Physac
[raylib-rs]: https://github.com/raylib-rs/raylib-rs
[drop]: https://doc.rust-lang.org/std/mem/fn.drop.html
[Arc]: https://doc.rust-lang.org/std/sync/struct.Arc.html
[Rc]: https://doc.rust-lang.org/std/rc/struct.Rc.html
[RwLock]: https://doc.rust-lang.org/std/sync/struct.RwLock.html
[rcWeak]: https://doc.rust-lang.org/std/rc/struct.Weak.html
[syncWeak]: https://doc.rust-lang.org/std/sync/struct.Weak.html
