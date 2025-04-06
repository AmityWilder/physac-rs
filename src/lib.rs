//! # physac-rs
//!
//! Based on Physac v1.1 - 2D Physics library for videogames
//!
//! # Description
//!
//! Physac is a small 2D physics library written in pure C. The engine uses a fixed time-step thread loop to simluate physics. A physics step contains the following phases: get collision information, apply dynamics, collision solving and position correction. It uses a very simple struct for physic bodies with a position vector to be used in any 3D rendering API.
//!
//! # Features
//!
//! | Feature | Description |
//! | ------- | ----------- |
//! | `sync` | The generated implementation will use [`std::sync`] instead of [`std::rc`] and [`std::cell`] (required by `phys_thread`) |
//! | `phys_thread` | If disabled, the generated implementation won't include [`std::thread`] and user must create a secondary thread to call [`physics_thread()`]. It is so important that the thread where [`physics_thread()`] is called must not have v-sync or any other CPU limitation. |
//! | `standalone` | Avoid raylib-rs inclusion in this file. Data types defined on raylib are defined internally in the library and input management and drawing functions must be provided by the user (check library implementation for further details). |
//! | `debug` | Traces log messages when creating and destroying physics bodies and detects errors in physics calculations and reference exceptions; it is useful for debug purposes |
//!
//!
//! NOTE 1: Physac requires multi-threading, when init_physics() a second thread is created to manage physics calculations.

/*
VERY THANKS TO:
    - raysan5: helped with library design
    - ficoos: added support for Linux
    - R8D8: added support for Linux
    - jubalh: fixed implementation of time calculations
    - a3f: fixed implementation of time calculations
    - define-private-public: added support for OSX
    - pamarcos: fixed implementation of physics steps
    - noshbar: fixed some memory leaks

LICENSE: zlib/libpng

Copyright (c) 2016-2025 Victor Fisac (github: @victorfisac)

This software is provided "as-is", without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.

Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.

    2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.

    3. This notice may not be removed or altered from any source distribution.
*/

#![warn(clippy::missing_panics_doc, clippy::missing_safety_doc, clippy::missing_errors_doc)]

pub mod prelude {
    pub use crate::*;
}

#[cfg(feature = "raylib")]
use raylib::prelude::Vector2;

#[cfg(not(feature = "raylib"))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

use std::{num::NonZeroUsize, time::Instant};

#[cfg(feature = "phys_thread")]
use std::{
    time::Duration,
    sync::atomic::{AtomicBool, Ordering::{Relaxed, Release}},
    thread,
};


#[cfg(feature = "sync")]
use std::sync::{self, Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

#[cfg(not(feature = "sync"))]
use std::{rc::{self, Rc}, cell::{RefCell, Ref, RefMut}};

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
use std::f64::consts::PI;
const DEG2RAD: f64 = PI/180.0;

macro_rules! debug_print {
    ($($msg:tt)*) => {
        #[cfg(feature = "debug")] {
            println!($($msg)*);
        }
    };
}

#[inline(always)]
pub const fn next_idx(index: usize, len: usize) -> usize {
    if (index + 1) < len { index + 1 } else { 0 }
}

//----------------------------------------------------------------------------------
// Types and Structures Definition
//----------------------------------------------------------------------------------
// Mat2 type (used for polygon shape rotation matrix)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Mat2 {
    pub m00: f32,
    pub m01: f32,
    pub m10: f32,
    pub m11: f32,
}
impl Mat2 {
    #[inline]
    pub const fn identity() -> Self {
        Self {
            m00: 1.0, m01: 0.0,
            m10: 0.0, m11: 1.0,
        }
    }

    /// Creates a matrix 2x2 from a given radians value
    pub fn radians(radians: f32) -> Mat2 {
        let (s, c) = radians.sin_cos();

        Mat2 {
            m00: c, m01: -s,
            m10: s, m11: c,
        }
    }

    /// Set values from radians to a created matrix 2x2
    pub fn set(&mut self, radians: f32) {
        let (sin, cos) = radians.sin_cos();

        self.m00 = cos; self.m01 = -sin;
        self.m10 = sin; self.m11 = cos;
    }

    // Returns the transpose of a given matrix 2x2
    #[inline(always)]
    pub fn transpose(&self) -> Mat2 {
        Mat2 {
            m00: self.m00,
            m01: self.m10,
            m10: self.m01,
            m11: self.m11,
        }
    }

    // Multiplies a vector by a matrix 2x2
    #[inline(always)]
    pub fn multiply_vector2(&self, vector: Vector2) -> Vector2 {
        Vector2 {
            x: self.m00*vector.x + self.m01*vector.y,
            y: self.m10*vector.x + self.m11*vector.y,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolygonData<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> {
    /// Current used vertex and normals count
    pub vertex_count: usize,
    /// Polygon vertex positions vectors
    pub positions: [Vector2; MAX_VERTICES],
    /// Polygon vertex normals vectors
    pub normals: [Vector2; MAX_VERTICES],
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PolygonData<MAX_VERTICES, CIRCLE_VERTICES> {
    pub const fn new() -> Self {
        Self {
            vertex_count: 0,
            positions: [const { Vector2::zero() }; MAX_VERTICES],
            normals: [const { Vector2::zero() }; MAX_VERTICES],
        }
    }
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> Default for PolygonData<MAX_VERTICES, CIRCLE_VERTICES> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PhysicsShape<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> {
    Circle {
        /// Circle shape radius
        radius: f32,
    },
    Polygon {
        /// Polygon shape vertices position and normals data (just used for polygon shapes)
        vertex_data: PolygonData<MAX_VERTICES, CIRCLE_VERTICES>,
        /// Vertices transform matrix 2x2
        transform: Mat2,
    },
}
pub use PhysicsShape::{Circle as PHYSICS_CIRCLE, Polygon as PHYSICS_POLYGON};
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PhysicsShape<MAX_VERTICES, CIRCLE_VERTICES> {
    pub const fn new() -> Self {
        PHYSICS_CIRCLE { radius: 0.0 }
    }
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> Default for PhysicsShape<MAX_VERTICES, CIRCLE_VERTICES> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct PhysicsBodyData<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> {
    /// Reference unique identifier
    pub id: u32,
    /// Enabled dynamics state (collisions are calculated anyway)
    pub enabled: bool,
    /// Physics body shape pivot
    pub position: Vector2,
    /// Current linear velocity applied to position
    pub velocity: Vector2,
    /// Current linear force (reset to 0 every step)
    pub force: Vector2,
    /// Current angular velocity applied to orient
    pub angular_velocity: f32,
    /// Current angular force (reset to 0 every step)
    pub torque: f32,
    /// Rotation in radians
    pub orient: f32,
    /// Moment of inertia
    pub inertia: f32,
    /// Inverse value of inertia
    pub inverse_inertia: f32,
    /// Physics body mass
    pub mass: f32,
    /// Inverse value of mass
    pub inverse_mass: f32,
    /// Friction when the body has not movement (0 to 1)
    pub static_friction: f32,
    /// Friction when the body has movement (0 to 1)
    pub dynamic_friction: f32,
    /// Restitution coefficient of the body (0 to 1)
    pub restitution: f32,
    /// Apply gravity force to dynamics
    pub use_gravity: bool,
    /// Physics grounded on other body state
    pub is_grounded: bool,
    /// Physics rotation constraint
    pub freeze_orient: bool,
    /// Physics body shape information (type, radius, vertices, normals)
    pub shape: PhysicsShape<MAX_VERTICES, CIRCLE_VERTICES>,
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES> {
    const fn new() -> Self {
        Self {
            id: 0,
            enabled: false,
            position: Vector2::zero(),
            velocity: Vector2::zero(),
            force: Vector2::zero(),
            angular_velocity: 0.0,
            torque: 0.0,
            orient: 0.0,
            inertia: 0.0,
            inverse_inertia: 0.0,
            mass: 0.0,
            inverse_mass: 0.0,
            static_friction: 0.0,
            dynamic_friction: 0.0,
            restitution: 0.0,
            use_gravity: false,
            is_grounded: false,
            freeze_orient: false,
            shape: PhysicsShape::new(),
        }
    }
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> Default for PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
struct PhysicsManifoldData<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> {
    /// Reference unique identifier
    pub id: u32,
    /// Manifold first physics body reference
    pub body_a: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>,
    /// Manifold second physics body reference
    pub body_b: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>,
    /// Depth of penetration from collision
    pub penetration: f32,
    /// Normal direction vector from 'a' to 'b'
    pub normal: Vector2,
    /// Points of contact during collision
    pub contacts: [Vector2; 2],
    /// Current collision number of contacts
    pub contacts_count: u32,
    /// Mixed restitution during collision
    pub restitution: f32,
    /// Mixed dynamic friction during collision
    pub dynamic_friction: f32,
    /// Mixed static friction during collision
    pub static_friction: f32,
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES> {
    const fn new(body_a: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>, body_b: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>) -> Self {
        Self {
            id: 0,
            body_a,
            body_b,
            penetration: 0.0,
            normal: Vector2::zero(),
            contacts: [Vector2::zero(); 2],
            contacts_count: 0,
            restitution: 0.0,
            dynamic_friction: 0.0,
            static_friction: 0.0,
        }
    }
}

pub mod phys_rc {
    use super::*;

    #[cfg(feature = "sync")]
    pub type PhysacHandleReadGuard<'a, T> = RwLockReadGuard<'a, T>;
    #[cfg(not(feature = "sync"))]
    pub type PhysacHandleReadGuard<'a, T> = &'a T;

    #[cfg(feature = "sync")]
    pub type PhysacHandleWriteGuard<'a, T> = RwLockWriteGuard<'a, T>;
    #[cfg(not(feature = "sync"))]
    pub type PhysacHandleWriteGuard<'a, T> = &'a mut T;

    #[cfg(feature = "sync")]
    pub type PhysacReadGuard<'a, T> = RwLockReadGuard<'a, T>;
    #[cfg(not(feature = "sync"))]
    pub type PhysacReadGuard<'a, T> = Ref<'a, T>;

    #[cfg(feature = "sync")]
    pub type PhysacWriteGuard<'a, T> = RwLockWriteGuard<'a, T>;
    #[cfg(not(feature = "sync"))]
    pub type PhysacWriteGuard<'a, T> = RefMut<'a, T>;

    #[derive(Debug)]
    pub struct Strong<T> {
        #[cfg(feature = "sync")]
        inner: Arc<RwLock<T>>,
        #[cfg(not(feature = "sync"))]
        inner: Rc<RefCell<T>>,
    }
    impl<T> Clone for Strong<T> {
        fn clone(&self) -> Self {
            Self { inner: self.inner.clone() }
        }
    }
    impl<T> Strong<T> {
        pub(super) fn new(data: T) -> Self {
            Self {
                #[cfg(feature = "sync")]
                inner: Arc::new(RwLock::new(data)),
                #[cfg(not(feature = "sync"))]
                inner: Rc::new(RefCell::new(data)),
            }
        }

        /// Get a weak body from a strong one
        pub fn downgrade(&self) -> Weak<T> {
            let inner = &self.inner;
            Weak {
                #[cfg(feature = "sync")]
                inner: Arc::downgrade(inner),
                #[cfg(not(feature = "sync"))]
                inner: Rc::downgrade(inner),
            }
        }

        /// Get a temporary reference to the body
        ///
        /// **Note:** Physics cannot tick while any body is being borrowed, so try not to borrow across ticks
        #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing this object")]
        pub fn borrow(&self) -> PhysacReadGuard<'_, T> {
            #[cfg(feature = "sync")] {
                self.inner.read().expect("thread poison recovery is not supported")
            } #[cfg(not(feature = "sync"))] {
                self.inner.borrow()
            }
        }

        /// Get a temporary mutable reference to the body
        ///
        /// **Note:** Physics cannot tick while any body is being borrowed, so try not to borrow across ticks
        #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing this object")]
        pub fn borrow_mut(&self) -> PhysacWriteGuard<'_, T> {
            #[cfg(feature = "sync")] {
                self.inner.write().expect("thread poison recovery is not supported")
            } #[cfg(not(feature = "sync"))] {
                self.inner.borrow_mut()
            }
        }

        /// Get a temporary reference to the body
        #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing this object")]
        pub fn borrowed<U, F>(&self, f: F) -> U
        where
            F: FnOnce(&T) -> U
        {
            #[cfg(feature = "sync")] {
                f(&*self.inner.read().expect("thread poison recovery is not supported"))
            } #[cfg(not(feature = "sync"))] {
                f(&*self.inner.borrow())
            }
        }

        /// Get a temporary mutable reference to the body
        #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing this object")]
        pub fn borrowed_mut<U, F>(&self, f: F) -> U
        where
            F: FnOnce(&mut T) -> U
        {
            #[cfg(feature = "sync")] {
                f(&mut *self.inner.write().expect("thread poison recovery is not supported"))
            } #[cfg(not(feature = "sync"))] {
                f(&mut *self.inner.borrow_mut())
            }
        }
    }

    #[derive(Debug, Clone)]
    pub struct Weak<T> {
        #[cfg(feature = "sync")]
        inner: sync::Weak<RwLock<T>>,
        #[cfg(not(feature = "sync"))]
        inner: rc::Weak<RefCell<T>>,
    }
    impl<T> Default for Weak<T> {
        fn default() -> Self {
            Self::new()
        }
    }
    impl<T> Weak<T> {
        /// Constructs a weak body with no reference
        pub fn new() -> Self {
            Self {
                #[cfg(feature = "sync")]
                inner: sync::Weak::new(),
                #[cfg(not(feature = "sync"))]
                inner: rc::Weak::new()
            }
        }

        pub(super) fn strong_count(&self) -> usize {
            #[cfg(feature = "sync")]
            let n = sync::Weak::strong_count(&self.inner);
            #[cfg(not(feature = "sync"))]
            let n = rc::Weak::strong_count(&self.inner);
            n
        }

        /// Try to get a strong body from a weak one
        ///
        /// Returns [`None`] if the body has been destroyed
        ///
        /// **Note:** Holding onto a `StrongPhysicsBody` extends the physics body's lifetime, but it will stop
        /// receiving physics ticks once it has been destroyed--even if you are still holding onto one of these
        pub fn upgrade(&self) -> Option<Strong<T>> {
            self.inner.upgrade().map(|inner| Strong { inner })
        }

        /// Upgrades and borrows the inner value, applying the closure to that
        ///
        /// Returns [`None`] if the body has been destroyed
        #[must_use]
        pub fn borrowed<U, F>(&self, f: F) -> Option<U>
        where
            F: FnOnce(&T) -> U
        {
            self.upgrade().map(|x| x.borrowed(f))
        }

        /// Upgrades and borrows the inner value, applying the closure to that
        ///
        /// Returns [`None`] if the body has been destroyed
        #[must_use]
        pub fn borrowed_mut<U, F>(&self, f: F) -> Option<U>
        where
            F: FnOnce(&mut T) -> U
        {
            self.upgrade().map(|x| x.borrowed_mut(f))
        }
    }
}
pub use self::phys_rc::*;

/***********************************************************************************
*
*   PHYSAC IMPLEMENTATION
*
************************************************************************************/

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
pub const PHYSAC_K: f32 = 1.0/3.0;

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
pub struct Physac<const MAX_VERTICES: usize = 24, const CIRCLE_VERTICES: usize = MAX_VERTICES> {
    #[cfg(feature = "phys_thread")]
    fixed_time: f64,
    collision_iterations: usize,
    penetration_allowance: f32,
    penetration_correction: f32,

    #[cfg(feature = "phys_thread")]
    /// Physics thread
    physics_thread: Option<thread::JoinHandle<()>>,

    /// Offset time for MONOTONIC clock
    base_time: Instant,
    /// Start time in milliseconds
    start_time: f64,
    /// Delta time used for physics steps, in milliseconds
    delta_time: f64,
    /// Current time in milliseconds
    current_time: f64,

    /// Physics time step delta time accumulator
    accumulator: f64,
    /// Total physics steps processed
    steps_count: u32,
    /// Physics world gravity force
    gravity_force: Vector2,

    /// Physics bodies pointers array
    bodies: Vec<Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>>,
    /// Physics bodies pointers array
    contacts: Vec<PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES>>,
}
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> Drop for Physac<MAX_VERTICES, CIRCLE_VERTICES> {
    fn drop(&mut self) {
        #[cfg(feature = "phys_thread")]
        self.physics_thread
            .take().expect("[PHYSAC] thread should exist if physics has been initialized")
            .join().expect("[PHYSAC] physics thread failed to close");

        debug_print!("[PHYSAC] physics module closed successfully");
    }
}

pub struct PhysacHandle<T> {
    #[cfg(feature = "sync")]
    phys: Arc<RwLock<T>>,
    #[cfg(not(feature = "sync"))]
    phys: T,

    /// Physics thread enabled state
    #[cfg(feature = "phys_thread")]
    is_physics_thread_enabled: Arc<AtomicBool>,
}
impl<T> PhysacHandle<T> {
    fn new(phys: T) -> Self {
        #[cfg(feature = "sync")]
        let phys = Arc::new(RwLock::new(phys));
        Self {
            phys,
            #[cfg(feature = "phys_thread")]
            is_physics_thread_enabled: Arc::new(AtomicBool::new(false)),
        }
    }
}
impl<T> Drop for PhysacHandle<T> {
    /// Unitializes physics pointers and exits physics loop thread
    fn drop(&mut self) {
        #[cfg(feature = "phys_thread")]
        // Exit physics loop thread
        self.is_physics_thread_enabled.store(false, Release);
    }
}
#[cfg(not(feature = "sync"))]
impl<T> std::ops::Deref for PhysacHandle<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.phys
    }
}
#[cfg(not(feature = "sync"))]
impl<T> std::ops::DerefMut for PhysacHandle<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.phys
    }
}

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
pub struct PhysacBuilder<const MAX_VERTICES: usize = 24, const CIRCLE_VERTICES: usize = MAX_VERTICES> {
    circle_vertices: usize,
    fixed_time: f64,
    collision_iterations: usize,
    penetration_allowance: f32,
    penetration_correction: f32,
    gravity_force: Vector2,
    max_bodies: usize,
    max_manifolds: usize,
    is_max_manifolds_overridden: bool,
}

/// Initializes physics values, pointers and creates physics loop thread
pub fn init_physics<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize>() -> PhysacBuilder<MAX_VERTICES, CIRCLE_VERTICES> {
    PhysacBuilder {
        circle_vertices: MAX_VERTICES,
        fixed_time: 1.0/60.0,
        collision_iterations: 20,
        penetration_allowance: 0.05,
        penetration_correction: 0.4,
        gravity_force: Vector2 { x: 0.0, y: 9.81 },
        max_bodies: 64,
        max_manifolds: 4096,
        is_max_manifolds_overridden: false,
    }
}

impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PhysacBuilder<MAX_VERTICES, CIRCLE_VERTICES> {
    pub fn circle_vertices(&mut self, n: NonZeroUsize) -> &mut Self {
        self.circle_vertices = n.get();
        self
    }
    pub fn fixed_time(&mut self, value: f64) -> &mut Self {
        self.fixed_time = value;
        self
    }
    pub fn collision_iterations(&mut self, n: usize) -> &mut Self {
        self.collision_iterations = n;
        self
    }
    pub fn penetration_allowance(&mut self, amount: f32) -> &mut Self {
        self.penetration_allowance = amount;
        self
    }
    pub fn penetration_correction(&mut self, amount: f32) -> &mut Self {
        self.penetration_correction = amount;
        self
    }
    pub fn gravity_force(&mut self, x: f32, y: f32) -> &mut Self {
        self.gravity_force.x = x;
        self.gravity_force.y = y;
        self
    }
    pub fn gravity_force_v(&mut self, v: Vector2) -> &mut Self {
        self.gravity_force = v;
        self
    }
    pub fn gravity_force_x(&mut self, x: f32) -> &mut Self {
        self.gravity_force.x = x;
        self
    }
    pub fn gravity_force_y(&mut self, y: f32) -> &mut Self {
        self.gravity_force.y = y;
        self
    }
    pub fn max_bodies(&mut self, n: usize) -> &mut Self {
        self.max_bodies = n;
        if !self.is_max_manifolds_overridden {
            self.max_manifolds = n*n;
        }
        self
    }
    pub fn max_manifolds(&mut self, n: usize) -> &mut Self {
        self.max_manifolds = n;
        self.is_max_manifolds_overridden = true;
        self
    }
    pub fn build(&mut self) -> PhysacHandle<Physac<MAX_VERTICES, CIRCLE_VERTICES>> {
        let mut phys = Physac {
            #[cfg(feature = "phys_thread")]
            fixed_time: self.fixed_time,
            collision_iterations: self.collision_iterations,
            penetration_allowance: self.penetration_allowance,
            penetration_correction: self.penetration_correction,
            #[cfg(feature = "phys_thread")]
            physics_thread: None,
            base_time: Instant::now(),
            start_time: 0.0,
            delta_time: self.fixed_time/10.0 * 1000.0,
            current_time: 0.0,
            accumulator: 0.0,
            steps_count: 0,
            gravity_force: self.gravity_force,
            bodies: Vec::with_capacity(self.max_bodies),
            contacts: Vec::with_capacity(self.max_manifolds),
        };

        // Initialize high resolution timer
        phys.init_timer();

        #[allow(unused_mut, reason = "used mutably if phys_thread is active")]
        let mut ph = PhysacHandle::new(phys);

        #[cfg(feature = "phys_thread")] {
            // NOTE: if defined, user will need to create a thread for PhysicsThread function manually
            // Create physics thread using POSIXS thread libraries
            let phys_clone = ph.phys.clone();
            let is_physics_thread_enabled = ph.is_physics_thread_enabled.clone();
            ph.borrowed_mut(|ph| ph.physics_thread = Some(thread::spawn(move || physics_loop(phys_clone, is_physics_thread_enabled))));
        }

        debug_print!("[PHYSAC] physics module initialized successfully");

        ph
    }
}

impl<T> PhysacHandle<T> {
    #[cfg(feature = "phys_thread")]
    /// Returns true if physics thread is currently enabled
    pub fn is_physics_enabled(&self) -> bool {
        self.is_physics_thread_enabled.load(Relaxed)
    }

    /// Borrow Physac from any other threads for the duration of the closure
    #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing Physac")]
    pub fn borrowed<U, F>(&self, f: F) -> U
    where
        F: FnOnce(&T) -> U
    {
        let phys = &self.phys;
        #[cfg(feature = "sync")]
        let phys = phys.read().expect("thread poison recovery is not supported");
        f(&*phys)
    }

    /// Borrow Physac mutably from any other threads for the duration of the closure
    #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing Physac")]
    pub fn borrowed_mut<U, F>(&mut self, f: F) -> U
    where
        F: FnOnce(&mut T) -> U
    {
        let phys = &mut self.phys;
        #[cfg(feature = "sync")]
        let mut phys = phys.write().expect("thread poison recovery is not supported");
        f(&mut *phys)
    }

    /// Borrow Physac from any other threads until the guard goes out of scope
    #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing Physac")]
    pub fn borrow(&self) -> PhysacHandleReadGuard<'_, T> {
        let phys = &self.phys;
        #[cfg(feature = "sync")]
        let phys = phys.read().expect("thread poison recovery is not supported");
        phys
    }

    /// Borrow Physac mutably from any other threads until the guard goes out of scope
    #[cfg_attr(feature = "sync", doc = "\n # Panics\n\n This method may panic if another thread panicked while mutably borrowing Physac")]
    pub fn borrow_mut(&mut self) -> PhysacHandleWriteGuard<'_, T> {
        let phys = &mut self.phys;
        #[cfg(feature = "sync")]
        let phys = phys.write().expect("thread poison recovery is not supported");
        phys
    }
}

impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> Physac<MAX_VERTICES, CIRCLE_VERTICES> {
    /// Sets physics global gravity force
    pub fn set_physics_gravity(&mut self, x: f32, y: f32) {
        self.gravity_force.x = x;
        self.gravity_force.y = y;
    }

    /// Creates a new circle physics body with generic parameters
    ///
    /// Returns [`None`] if there are no available IDs
    pub fn try_create_physics_body_circle(&mut self, pos: Vector2, radius: f32, density: f32) -> Option<Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>> {
        if let Some(new_id) = self.find_available_body_index() {
            let new_body = Strong::new(PhysicsBodyData::default());

            let new_weak_body = new_body.clone();
            let mut new_body_data = new_body.borrow_mut();

            // Initialize new body with generic values
            new_body_data.id = new_id;
            new_body_data.enabled = true;
            new_body_data.position = pos;
            new_body_data.velocity = Vector2::zero();
            new_body_data.force = Vector2::zero();
            new_body_data.angular_velocity = 0.0;
            new_body_data.torque = 0.0;
            new_body_data.orient = 0.0;
            new_body_data.shape = PHYSICS_CIRCLE { radius };

            new_body_data.mass = (PI*radius as f64*radius as f64*density as f64) as f32;
            new_body_data.inverse_mass = if new_body_data.mass != 0.0 { 1.0/new_body_data.mass } else { 0.0 };
            new_body_data.inertia = new_body_data.mass*radius*radius;
            new_body_data.inverse_inertia = if new_body_data.inertia != 0.0 { 1.0/new_body_data.inertia } else { 0.0 };
            new_body_data.static_friction = 0.4;
            new_body_data.dynamic_friction = 0.2;
            new_body_data.restitution = 0.0;
            new_body_data.use_gravity = true;
            new_body_data.is_grounded = false;
            new_body_data.freeze_orient = false;

            drop(new_body_data);

            // Add new body to bodies pointers array and update bodies count
            self.bodies.push(new_body);

            debug_print!("[PHYSAC] created polygon physics body id {new_id}");
            Some(new_weak_body)
        } else {
            debug_print!("[PHYSAC] new physics body creation failed because there is any available id to use");
            None
        }
    }

    /// Creates a new circle physics body with generic parameters
    ///
    /// # Panics
    ///
    /// This method may panic if there are no available IDs
    #[inline]
    pub fn create_physics_body_circle(&mut self, pos: Vector2, radius: f32, density: f32) -> Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>> {
        self.try_create_physics_body_circle(pos, radius, density).unwrap()
    }

    /// Creates a new rectangle physics body with generic parameters
    ///
    /// Returns [`None`] if there are no available IDs
    pub fn try_create_physics_body_rectangle(&mut self, pos: Vector2, width: f32, height: f32, density: f32) -> Option<Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>> {
        if let Some(new_id) = self.find_available_body_index() {
            let new_body = Strong::new(PhysicsBodyData::default());

            let new_weak_body = new_body.clone();
            let mut new_body_data = new_body.borrow_mut();

            // Initialize new body with generic values
            new_body_data.id = new_id;
            new_body_data.enabled = true;
            new_body_data.position = pos;
            new_body_data.velocity = Vector2::zero();
            new_body_data.force = Vector2::zero();
            new_body_data.angular_velocity = 0.0;
            new_body_data.torque = 0.0;
            new_body_data.orient = 0.0;

            let mut vertex_data = PolygonData::create_rectangle_polygon(pos, Vector2 { x: width, y: height });

            // Calculate centroid and moment of inertia
            let mut center = Vector2 { x: 0.0, y: 0.0 };
            let mut area = 0.0;
            let mut inertia = 0.0;

            for i in 0..vertex_data.vertex_count {
                // Triangle vertices, third vertex implied as (0, 0)
                let p1 = vertex_data.positions[i];
                let next_index = next_idx(i, vertex_data.vertex_count);
                let p2 = vertex_data.positions[next_index];

                #[allow(non_snake_case)]
                let D = math_cross_vector2(p1, p2);
                let triangle_area = D/2.0;

                area += triangle_area;

                // Use area to weight the centroid average, not just vertex position
                center.x += triangle_area*PHYSAC_K*(p1.x + p2.x);
                center.y += triangle_area*PHYSAC_K*(p1.y + p2.y);

                let intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
                let inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
                inertia += (0.25*PHYSAC_K*D)*(intx2 + inty2);
            }

            center.x *= 1.0/area;
            center.y *= 1.0/area;

            // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
            // Note: this is not really necessary
            for i in 0..vertex_data.vertex_count {
                vertex_data.positions[i].x -= center.x;
                vertex_data.positions[i].y -= center.y;
            }

            new_body_data.shape = PHYSICS_POLYGON {
                vertex_data,
                transform: Mat2::radians(0.0),
            };

            new_body_data.mass = density*area;
            new_body_data.inverse_mass = if new_body_data.mass != 0.0 { 1.0/new_body_data.mass } else { 0.0 };
            new_body_data.inertia = density*inertia;
            new_body_data.inverse_inertia = if new_body_data.inertia != 0.0 { 1.0/new_body_data.inertia } else { 0.0 };
            new_body_data.static_friction = 0.4;
            new_body_data.dynamic_friction = 0.2;
            new_body_data.restitution = 0.0;
            new_body_data.use_gravity = true;
            new_body_data.is_grounded = false;
            new_body_data.freeze_orient = false;

            drop(new_body_data);

            // Add new body to bodies pointers array and update bodies count
            self.bodies.push(new_body);

            debug_print!("[PHYSAC] created polygon physics body id {new_id}");
            Some(new_weak_body)
        } else {
            debug_print!("[PHYSAC] new physics body creation failed because there is any available id to use");
            None
        }
    }

    /// Creates a new rectangle physics body with generic parameters
    ///
    /// # Panics
    ///
    /// This method may panic if there are no available IDs
    #[inline]
    pub fn create_physics_body_rectangle(&mut self, pos: Vector2, width: f32, height: f32, density: f32) -> Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>> {
        self.try_create_physics_body_rectangle(pos, width, height, density).unwrap()
    }

    /// Creates a new polygon physics body with generic parameters
    ///
    /// Returns [`None`] if there are no available IDs
    pub fn try_create_physics_body_polygon(&mut self, pos: Vector2, radius: f32, sides: usize, density: f32) -> Option<Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>> {
        if let Some(new_id) = self.find_available_body_index() {
            let new_body = Strong::new(PhysicsBodyData::default());

            let new_weak_body = new_body.clone();
            let mut new_body_data = new_body.borrow_mut();

            // Initialize new body with generic values
            new_body_data.id = new_id;
            new_body_data.enabled = true;
            new_body_data.position = pos;
            new_body_data.velocity = Vector2::zero();
            new_body_data.force = Vector2::zero();
            new_body_data.angular_velocity = 0.0;
            new_body_data.torque = 0.0;
            new_body_data.orient = 0.0;
            let mut vertex_data = PolygonData::create_random_polygon(radius, sides);

            // Calculate centroid and moment of inertia
            let mut center = Vector2 { x: 0.0, y: 0.0 };
            let mut area = 0.0;
            let mut inertia = 0.0;

            for i in 0..vertex_data.vertex_count {
                // Triangle vertices, third vertex implied as (0, 0)
                let position1 = vertex_data.positions[i];
                let next_index = next_idx(i, vertex_data.vertex_count);
                let position2 = vertex_data.positions[next_index];

                let cross = math_cross_vector2(position1, position2);
                let triangle_area = cross/2.0;

                area += triangle_area;

                // Use area to weight the centroid average, not just vertex position
                center.x += triangle_area*PHYSAC_K*(position1.x + position2.x);
                center.y += triangle_area*PHYSAC_K*(position1.y + position2.y);

                let intx2 = position1.x*position1.x + position2.x*position1.x + position2.x*position2.x;
                let inty2 = position1.y*position1.y + position2.y*position1.y + position2.y*position2.y;
                inertia += (0.25*PHYSAC_K*cross)*(intx2 + inty2);
            }

            center.x *= 1.0/area;
            center.y *= 1.0/area;

            // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
            // Note: this is not really necessary
            for i in 0..vertex_data.vertex_count {
                vertex_data.positions[i].x -= center.x;
                vertex_data.positions[i].y -= center.y;
            }

            new_body_data.shape = PHYSICS_POLYGON {
                vertex_data,
                transform: Mat2::radians(0.0),
            };
            new_body_data.mass = density*area;
            new_body_data.inverse_mass = if new_body_data.mass != 0.0 { 1.0/new_body_data.mass } else { 0.0 };
            new_body_data.inertia = density*inertia;
            new_body_data.inverse_inertia = if new_body_data.inertia != 0.0 { 1.0/new_body_data.inertia } else { 0.0 };
            new_body_data.static_friction = 0.4;
            new_body_data.dynamic_friction = 0.2;
            new_body_data.restitution = 0.0;
            new_body_data.use_gravity = true;
            new_body_data.is_grounded = false;
            new_body_data.freeze_orient = false;

            drop(new_body_data);

            // Add new body to bodies pointers array and update bodies count
            self.bodies.push(new_body);

            debug_print!("[PHYSAC] created polygon physics body id {new_id}");
            Some(new_weak_body)
        } else {
            debug_print!("[PHYSAC] new physics body creation failed because there is any available id to use");
            None
        }
    }

    /// Creates a new polygon physics body with generic parameters
    ///
    /// # Panics
    ///
    /// This method may panic if there are no available IDs
    #[inline]
    pub fn create_physics_body_polygon(&mut self, pos: Vector2, radius: f32, sides: usize, density: f32) -> Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>> {
        self.try_create_physics_body_polygon(pos, radius, sides, density).unwrap()
    }

    /// Shatters a polygon shape physics body to little physics bodies with explosion force
    ///
    /// # Panics
    ///
    /// This method may panic if there are not enough available IDs to create fragments
    pub fn physics_shatter(&mut self, body: Weak<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>, position: Vector2, force: f32) {
        if let Some(phys_body) = body.upgrade() {
            let body = phys_body.borrow_mut();
            if let PHYSICS_POLYGON { vertex_data, transform } = body.shape {
                let mut collision = false;

                for i in 0..vertex_data.vertex_count {
                    let position_a = body.position;
                    let position_b = transform.multiply_vector2(body.position + vertex_data.positions[i]);
                    let next_index = next_idx(i, vertex_data.vertex_count);
                    let position_c = transform.multiply_vector2(body.position + vertex_data.positions[next_index]);

                    // Check collision between each triangle
                    let alpha = ((position_b.y - position_c.y)*(position.x - position_c.x) + (position_c.x - position_b.x)*(position.y - position_c.y))/
                                ((position_b.y - position_c.y)*(position_a.x - position_c.x) + (position_c.x - position_b.x)*(position_a.y - position_c.y));

                    let beta = ((position_c.y - position_a.y)*(position.x - position_c.x) + (position_a.x - position_c.x)*(position.y - position_c.y))/
                                ((position_b.y - position_c.y)*(position_a.x - position_c.x) + (position_c.x - position_b.x)*(position_a.y - position_c.y));

                    let gamma = 1.0 - alpha - beta;

                    if (alpha > 0.0) && (beta > 0.0) && (gamma > 0.0) {
                        collision = true;
                        break;
                    }
                }

                if collision {
                    let count = vertex_data.vertex_count;
                    let body_pos = body.position;
                    let mut vertices = vec![Vector2::zero(); count];
                    let trans = transform;

                    vertices[..count].copy_from_slice(&vertex_data.positions[..count]);

                    // Destroy shattered physics body
                    drop(body);
                    self.destroy_physics_body(phys_body);

                    for i in 0..count {
                        let next_index = next_idx(i, count);
                        let mut center = triangle_barycenter(vertices[i], vertices[next_index], Vector2::zero());
                        center = body_pos + center;
                        let offset = center - body_pos;

                        // Create polygon physics body with relevant values
                        let new_body = self.try_create_physics_body_polygon(center, 10.0, 3, 10.0).expect("failed to create physics body"); // The original line just creates a pointer without checking if it's valid
                        let mut new_body_data = new_body.borrow_mut();

                        let mut new_data = PolygonData {
                            vertex_count: 3,
                            ..Default::default()
                        };

                        new_data.positions[0] = vertices[i] - offset;
                        new_data.positions[1] = vertices[next_index] - offset;
                        new_data.positions[2] = position - center;

                        // Separate vertices to avoid unnecessary physics collisions
                        new_data.positions[0].x *= 0.95;
                        new_data.positions[0].y *= 0.95;
                        new_data.positions[1].x *= 0.95;
                        new_data.positions[1].y *= 0.95;
                        new_data.positions[2].x *= 0.95;
                        new_data.positions[2].y *= 0.95;

                        // Calculate polygon faces normals
                        for j in 0..new_data.vertex_count {
                            let next_vertex = next_idx(j, new_data.vertex_count);
                            let face = new_data.positions[next_vertex] - new_data.positions[j];

                            new_data.normals[j] = Vector2 { x: face.y, y: -face.x };
                            math_normalize(&mut new_data.normals[j]);
                        }

                        // Calculate centroid and moment of inertia
                        center = Vector2::zero();
                        let mut area = 0.0;
                        let mut inertia = 0.0;

                        for j in 0..new_data.vertex_count {
                            // Triangle vertices, third vertex implied as (0, 0)
                            let p1 = new_data.positions[j];
                            let next_vertex = next_idx(j, new_data.vertex_count);
                            let p2 = new_data.positions[next_vertex];

                            #[allow(non_snake_case)]
                            let D = math_cross_vector2(p1, p2);
                            let triangle_area = D/2.0;

                            area += triangle_area;

                            // Use area to weight the centroid average, not just vertex position
                            center.x += triangle_area*PHYSAC_K*(p1.x + p2.x);
                            center.y += triangle_area*PHYSAC_K*(p1.y + p2.y);

                            let intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
                            let inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
                            inertia += (0.25*PHYSAC_K*D)*(intx2 + inty2);
                        }

                        // Apply computed vertex data to new physics body shape
                        new_body_data.shape = PHYSICS_POLYGON { vertex_data: new_data, transform: trans };

                        center.x *= 1.0/area;
                        center.y *= 1.0/area;

                        new_body_data.mass = area;
                        new_body_data.inverse_mass = if new_body_data.mass != 0.0 { 1.0/new_body_data.mass } else { 0.0 };
                        new_body_data.inertia = inertia;
                        new_body_data.inverse_inertia = if new_body_data.inertia != 0.0 { 1.0/new_body_data.inertia } else { 0.0 };

                        // Calculate explosion force direction
                        let point_a = new_body_data.position;
                        let mut point_b = new_data.positions[1] - new_data.positions[0];
                        point_b.x /= 2.0;
                        point_b.y /= 2.0;
                        let mut force_direction = (point_a + (new_data.positions[0] + point_b)) - new_body_data.position;
                        math_normalize(&mut force_direction);
                        force_direction.x *= force;
                        force_direction.y *= force;

                        drop(new_body_data);

                        // Apply force to new physics body
                        new_body.borrow_mut().add_force(force_direction);
                    }

                    drop(vertices);
                }
            }
        } else {
            debug_print!("[PHYSAC] error when trying to shatter a null reference physics body");
        }
    }

    /// Returns the current amount of created physics bodies
    pub fn get_physics_bodies_count(&self) -> usize {
        self.bodies.len()
    }

    /// Returns a physics body of the bodies pool at a specific index
    ///
    /// Returns [`None`] if `index` is out of bounds
    pub fn try_get_physics_body(&self, index: usize) -> Option<&Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>> {
        if index >= self.bodies.len() {
            debug_print!("[PHYSAC] physics body index is out of bounds");
        }

        self.bodies.get(index)
    }

    /// Returns a physics body of the bodies pool at a specific index
    ///
    /// # Panics
    ///
    /// This method may panic if `index` is out of bounds
    pub fn get_physics_body(&self, index: usize) -> &Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>> {
        self.try_get_physics_body(index).unwrap()
    }

    /// Returns a physics body of the bodies pool at a specific index
    ///
    /// # Panics
    ///
    /// This method may panic if `index` is out of bounds
    pub fn borrow_physics_body(&self, index: usize) -> PhysacReadGuard<'_, PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>> {
        self.get_physics_body(index).borrow()
    }

    /// Returns a physics body of the bodies pool at a specific index
    ///
    /// # Panics
    ///
    /// This method may panic if `index` is out of bounds
    pub fn borrow_physics_body_mut(&self, index: usize) -> PhysacWriteGuard<'_, PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>> {
        self.get_physics_body(index).borrow_mut()
    }

    pub fn physics_body_iter(&self) -> impl DoubleEndedIterator<Item = PhysacReadGuard<'_, PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>> + ExactSizeIterator {
        self.bodies.iter().map(|body| body.borrow())
    }

    pub fn physics_body_iter_mut(&self) -> impl DoubleEndedIterator<Item = PhysacWriteGuard<'_, PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>> + ExactSizeIterator {
        self.bodies.iter().map(|body| body.borrow_mut())
    }

    /// Unitializes and destroys a physics body
    pub fn destroy_physics_body(&mut self, body: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>) {
        let id = body.borrow().id;

        let index = self.bodies.iter().position(|body| body.borrow().id == id);

        if let Some(index) = index {
            #[cfg(debug_assertions)]
            let weak = body.downgrade();
            // Free body allocated memory
            drop(body);
            self.bodies.remove(index);

            debug_print!("[PHYSAC] destroyed physics body id {id}");
            #[cfg(debug_assertions)] {
                let strong_count = weak.strong_count();
                if strong_count > 0 {
                    debug_print!("[PHYSAC] WARNING: physics body id {id} is destroyed, but still has {strong_count} strong references");
                }
            }
        } else {
            debug_print!("[PHYSAC] Not possible to find body id {id} in pointers array");
        }
    }

    /// Unitializes and destroys a physics body
    pub fn try_destroy_physics_body(&mut self, body: Weak<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>) {
        if let Some(body) = body.upgrade() {
            self.destroy_physics_body(body);
        } else {
            debug_print!("[PHYSAC] error trying to destroy a null referenced body");
        }
    }
}

impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES> {
    /// Adds a force to a physics body
    pub fn add_force(&mut self, force: Vector2) {
        self.force += force;
    }

    /// Adds an angular force to a physics body
    pub fn add_torque(&mut self, amount: f32) {
        self.torque += amount;
    }

    /// Sets physics body shape transform based on radians parameter
    pub fn set_rotation(&mut self, radians: f32) {
        self.orient = radians;

        if let PHYSICS_POLYGON { transform, .. } = &mut self.shape {
            *transform = Mat2::radians(radians);
        }
    }

    /// Returns the number of vertices of a physics body shape
    pub fn get_physics_shape_vertices_count(&self) -> usize {
        match self.shape {
            PHYSICS_CIRCLE { .. } => CIRCLE_VERTICES,
            PHYSICS_POLYGON { vertex_data, .. } => vertex_data.vertex_count,
        }
    }

    /// Returns transformed position of a body shape (body position + vertex transformed position)
    ///
    /// Returns [`None`] if `vertex` index is out of bounds
    pub fn try_get_physics_body_shape_vertex(&self, vertex: usize) -> Option<Vector2> {
        match self.shape {
            PHYSICS_CIRCLE { radius } => {
                Some(Vector2 {
                    x: self.position.x + (360.0/CIRCLE_VERTICES as f32*vertex as f32*DEG2RAD as f32).cos()*radius,
                    y: self.position.y + (360.0/CIRCLE_VERTICES as f32*vertex as f32*DEG2RAD as f32).sin()*radius,
                })
            }
            PHYSICS_POLYGON { vertex_data, transform } => {
                if let Some(&p) = vertex_data.positions.get(vertex) {
                    Some(self.position + transform.multiply_vector2(p))
                } else {
                    debug_print!("[PHYSAC] physics shape vertex index is out of bounds");
                    None
                }
            }
        }
    }

    /// Returns transformed position of a body shape (body position + vertex transformed position)
    ///
    /// # Panics
    ///
    /// This method may panic if `vertex` index is out of bounds
    pub fn get_physics_shape_vertex(&self, vertex: usize) -> Vector2 {
        self.try_get_physics_body_shape_vertex(vertex).unwrap()
    }

    /// Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
    pub fn get_physics_shape_type(&self) -> std::mem::Discriminant<PhysicsShape<MAX_VERTICES, CIRCLE_VERTICES>> {
        std::mem::discriminant(&self.shape)
    }
}

//----------------------------------------------------------------------------------
// Module Internal Functions Definition
//----------------------------------------------------------------------------------
impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PolygonData<MAX_VERTICES, CIRCLE_VERTICES> {
    /// Creates a random polygon shape with max vertex distance from polygon pivot
    fn create_random_polygon(radius: f32, sides: usize) -> PolygonData<MAX_VERTICES, CIRCLE_VERTICES> {
        let mut data = PolygonData {
            vertex_count: sides,
            ..Default::default()
        };

        // Calculate polygon vertices positions
        for i in 0..data.vertex_count {
            data.positions[i].x = (360.0/sides as f32*i as f32*DEG2RAD as f32).cos()*radius;
            data.positions[i].y = (360.0/sides as f32*i as f32*DEG2RAD as f32).sin()*radius;
        }

        // Calculate polygon faces normals
        for i in 0..data.vertex_count {
            let next_index = next_idx(i, sides);
            let face = data.positions[next_index] - data.positions[i];

            data.normals[i] = Vector2 { x: face.y, y: -face.x };
            math_normalize(&mut data.normals[i]);
        }

        data
    }

    /// Creates a rectangle polygon shape based on a min and max positions
    fn create_rectangle_polygon(pos: Vector2, size: Vector2) -> PolygonData<MAX_VERTICES, CIRCLE_VERTICES> {
        let mut data = PolygonData {
            vertex_count: 4,
            ..Default::default()
        };

        // Calculate polygon vertices positions
        data.positions[0] = Vector2 { x: pos.x + size.x/2.0, y: pos.y - size.y/2.0 };
        data.positions[1] = Vector2 { x: pos.x + size.x/2.0, y: pos.y + size.y/2.0 };
        data.positions[2] = Vector2 { x: pos.x - size.x/2.0, y: pos.y + size.y/2.0 };
        data.positions[3] = Vector2 { x: pos.x - size.x/2.0, y: pos.y - size.y/2.0 };

        // Calculate polygon faces normals
        for i in 0..data.vertex_count {
            let next_index = next_idx(i, data.vertex_count);
            let face = data.positions[next_index] - data.positions[i];

            data.normals[i] = Vector2 { x: face.y, y: -face.x };
            math_normalize(&mut data.normals[i]);
        }

        data
    }
}

/// Physics loop thread function
#[cfg(feature = "phys_thread")]
fn physics_loop<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize>(phys: Arc<RwLock<Physac<MAX_VERTICES, CIRCLE_VERTICES>>>, is_physics_thread_enabled: Arc<AtomicBool>) {
    debug_print!("[PHYSAC] physics thread created successfully");

    // Initialize physics loop thread values
    is_physics_thread_enabled.store(true, Relaxed);

    // Physics update loop
    while is_physics_thread_enabled.load(Relaxed) {
        phys.write().expect("thread poison recovery is not supported").run_physics_step();

        let req = Duration::from_secs_f64(phys.read().expect("thread poison recovery is not supported").fixed_time);

        std::thread::sleep(req);
    }
}

impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> Physac<MAX_VERTICES, CIRCLE_VERTICES> {
    /// Physics steps calculations (dynamics, collisions and position corrections)
    fn physics_step(&mut self) {
        // Update current steps count
        self.steps_count += 1;

        // Clear previous generated collisions information
        self.contacts.clear();

        // Reset physics bodies grounded state
        for body in &self.bodies {
            body.borrow_mut().is_grounded = false;
        }

        // Generate new collision information
        for i in 0..self.bodies.len() {
            for j in (i + 1)..self.bodies.len() {
                let body_a = self.bodies[i].clone();
                let body_b = self.bodies[j].clone();
                if (self.bodies[i].borrow().inverse_mass == 0.0) && (body_b.borrow().inverse_mass == 0.0) {
                    continue;
                }

                let manifold = self.create_physics_manifold(body_a.clone(), body_b.clone()).unwrap();
                manifold.solve(&mut *body_a.borrow_mut(), &mut *body_b.borrow_mut());

                if manifold.contacts_count > 0 {
                    let manifold = manifold.clone();
                    // Create a new manifold with same information as previously solved manifold and add it to the manifolds pool last slot
                    let new_manifold = self.create_physics_manifold(body_a, body_b).unwrap();
                    new_manifold.penetration = manifold.penetration;
                    new_manifold.normal = manifold.normal;
                    new_manifold.contacts[0] = manifold.contacts[0];
                    new_manifold.contacts[1] = manifold.contacts[1];
                    new_manifold.contacts_count = manifold.contacts_count;
                    new_manifold.restitution = manifold.restitution;
                    new_manifold.dynamic_friction = manifold.dynamic_friction;
                    new_manifold.static_friction = manifold.static_friction;
                }
            }
        }

        // Integrate forces to physics bodies
        for body in &self.bodies {
            Self::integrate_physics_forces(&mut *body.borrow_mut(), self.delta_time, self.gravity_force);
        }

        // Initialize physics manifolds to solve collisions
        for manifold in &mut self.contacts {
            Self::initialize_physics_manifolds(manifold, self.delta_time, self.gravity_force);
        }

        // Integrate physics collisions impulses to solve collisions
        for _ in 0..self.collision_iterations {
            for manifold in &mut self.contacts {
                Self::integrate_physics_impulses(manifold);
            }
        }

        // Integrate velocity to physics bodies
        for body in &self.bodies {
            Self::integrate_physics_velocity(&mut *body.borrow_mut(), self.delta_time, self.gravity_force);
        }

        // Correct physics bodies positions based on manifolds collision information
        for manifold in &mut self.contacts {
            Self::correct_physics_positions(manifold, self.penetration_allowance, self.penetration_correction);
        }

        // Clear physics bodies forces
        for body in &self.bodies {
            let mut body = body.borrow_mut();
            body.force = Vector2::zero();
            body.torque = 0.0;
        }
    }

    /// Wrapper to ensure PhysicsStep is run with at a fixed time step
    pub fn run_physics_step(&mut self) {
        // Calculate current time
        self.current_time = self.get_curr_time();

        // Calculate current delta time
        let delta: f64 = self.current_time - self.start_time;

        // Store the time elapsed since the last frame began
        self.accumulator += delta;

        // Fixed time stepping loop
        while self.accumulator >= self.delta_time {
            self.physics_step();
            self.accumulator -= self.delta_time;
        }

        // Record the starting of this frame
        self.start_time = self.current_time;
    }

    pub fn set_physics_time_step(&mut self, delta: f64) {
        self.delta_time = delta;
    }

    /// Finds a valid index for a new physics body initialization
    fn find_available_body_index(&self) -> Option<u32> {
        let mut index = None;
        for i in 0..self.bodies.capacity() as u32 {
            let mut current_id = i;

            // Check if current id already exist in other physics body
            for body in &self.bodies {
                if body.borrow().id == current_id {
                    current_id += 1;
                    break;
                }
            }

            // If it is not used, use it as new physics body id
            if current_id == i {
                index = Some(i);
                break;
            }
        }

        index
    }

    /// Finds a valid index for a new manifold initialization
    fn find_available_manifold_index(&self) -> Option<u32> {
        (self.contacts.len() as u32).checked_add(1)
            .filter(|id| *id < self.contacts.capacity() as u32)
    }

    /// Creates a new physics manifold to solve collision
    fn create_physics_manifold(&mut self, a: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>, b: Strong<PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>>) -> Option<&mut PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES>> {
        if let Some(new_id) = self.find_available_manifold_index() {
            let mut new_manifold = PhysicsManifoldData::new(a, b);

            // Initialize new manifold with generic values
            new_manifold.id = new_id;
            new_manifold.penetration = 0.0;
            new_manifold.normal = Vector2::zero();
            new_manifold.contacts[0] = Vector2::zero();
            new_manifold.contacts[1] = Vector2::zero();
            new_manifold.contacts_count = 0;
            new_manifold.restitution = 0.0;
            new_manifold.dynamic_friction = 0.0;
            new_manifold.static_friction = 0.0;

            // Add new body to bodies pointers array and update bodies count
            self.contacts.push(new_manifold);
            Some(&mut *self.contacts.last_mut().expect("should have at least one element after push"))
        } else {
            debug_print!("[PHYSAC] new physics manifold creation failed because there is any available id to use");
            None
        }
    }

    /// Integrates physics forces into velocity
    fn integrate_physics_forces(body: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, delta_time: f64, gravity_force: Vector2) {
        if (body.inverse_mass == 0.0) || !body.enabled {
            return;
        }

        body.velocity.x += ((body.force.x*body.inverse_mass) as f64*(delta_time/2.0)) as f32;
        body.velocity.y += ((body.force.y*body.inverse_mass) as f64*(delta_time/2.0)) as f32;

        if body.use_gravity {
            body.velocity.x += (gravity_force.x as f64*(delta_time/1000.0/2.0)) as f32;
            body.velocity.y += (gravity_force.y as f64*(delta_time/1000.0/2.0)) as f32;
        }

        if !body.freeze_orient {
            body.angular_velocity += (body.torque as f64*body.inverse_inertia as f64*(delta_time/2.0)) as f32;
        }
    }

    /// Initializes physics manifolds to solve collisions
    fn initialize_physics_manifolds(manifold: &mut PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES>, delta_time: f64, gravity_force: Vector2) {
        let body_a = manifold.body_a.borrow();
        let body_b = manifold.body_b.borrow();

        // Calculate average restitution, static and dynamic friction
        manifold.restitution = (body_a.restitution*body_b.restitution).sqrt();
        manifold.static_friction = (body_a.static_friction*body_b.static_friction).sqrt();
        manifold.dynamic_friction = (body_a.dynamic_friction*body_b.dynamic_friction).sqrt();

        for i in 0..manifold.contacts_count {
            // Caculate radius from center of mass to contact
            let radius_a = manifold.contacts[i as usize] - body_a.position;
            let radius_b = manifold.contacts[i as usize] - body_b.position;

            let cross_a = math_cross(body_a.angular_velocity, radius_a);
            let cross_b = math_cross(body_b.angular_velocity, radius_b);

            let mut radius_v = Vector2 { x: 0.0, y: 0.0 };
            radius_v.x = body_b.velocity.x + cross_b.x - body_a.velocity.x - cross_a.x;
            radius_v.y = body_b.velocity.y + cross_b.y - body_a.velocity.y - cross_a.y;

            // Determine if we should perform a resting collision or not;
            // The idea is if the only thing moving this object is gravity, then the collision should be performed without any restitution
            if radius_v.length_sqr() < ((Vector2 {
                x: gravity_force.x*delta_time as f32/1000.0,
                y: gravity_force.y*delta_time as f32/1000.0,
            }).length_sqr() + f32::EPSILON) {
                manifold.restitution = 0.0;
            }
        }
    }

    /// Integrates physics collisions impulses to solve collisions
    fn integrate_physics_impulses(manifold: &mut PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES>) {
        let mut body_a = manifold.body_a.borrow_mut();
        let mut body_b = manifold.body_b.borrow_mut();

        // Early out and positional correct if both objects have infinite mass
        if (body_a.inverse_mass + body_b.inverse_mass).abs() <= f32::EPSILON {
            body_a.velocity = Vector2::zero();
            body_b.velocity = Vector2::zero();
            return;
        }

        for i in 0..manifold.contacts_count {
            // Calculate radius from center of mass to contact
            let radius_a = manifold.contacts[i as usize] - body_a.position;
            let radius_b = manifold.contacts[i as usize] - body_b.position;

            // Calculate relative velocity
            let mut radius_v = Vector2::zero();
            radius_v.x = body_b.velocity.x + math_cross(body_b.angular_velocity, radius_b).x - body_a.velocity.x - math_cross(body_a.angular_velocity, radius_a).x;
            radius_v.y = body_b.velocity.y + math_cross(body_b.angular_velocity, radius_b).y - body_a.velocity.y - math_cross(body_a.angular_velocity, radius_a).y;

            // Relative velocity along the normal
            let contact_velocity = radius_v.dot(manifold.normal);

            // Do not resolve if velocities are separating
            if contact_velocity > 0.0 {
                return;
            }

            let ra_cross_n = math_cross_vector2(radius_a, manifold.normal);
            let rb_cross_n = math_cross_vector2(radius_b, manifold.normal);

            let inverse_mass_sum = body_a.inverse_mass + body_b.inverse_mass + (ra_cross_n*ra_cross_n)*body_a.inverse_inertia + (rb_cross_n*rb_cross_n)*body_b.inverse_inertia;

            // Calculate impulse scalar value
            let mut impulse = -(1.0 + manifold.restitution)*contact_velocity;
            impulse /= inverse_mass_sum;
            impulse /= manifold.contacts_count as f32;

            // Apply impulse to each physics body
            let impulse_v = manifold.normal*impulse;

            if body_a.enabled {
                body_a.velocity.x += body_a.inverse_mass*(-impulse_v.x);
                body_a.velocity.y += body_a.inverse_mass*(-impulse_v.y);

                if !body_a.freeze_orient {
                    body_a.angular_velocity += body_a.inverse_inertia*math_cross_vector2(radius_a, -impulse_v);
                }
            }

            if body_b.enabled {
                body_b.velocity.x += body_b.inverse_mass*(impulse_v.x);
                body_b.velocity.y += body_b.inverse_mass*(impulse_v.y);

                if !body_b.freeze_orient {
                    body_b.angular_velocity += body_b.inverse_inertia*math_cross_vector2(radius_b, impulse_v);
                }
            }

            // Apply friction impulse to each physics body
            radius_v.x = body_b.velocity.x + math_cross(body_b.angular_velocity, radius_b).x - body_a.velocity.x - math_cross(body_a.angular_velocity, radius_a).x;
            radius_v.y = body_b.velocity.y + math_cross(body_b.angular_velocity, radius_b).y - body_a.velocity.y - math_cross(body_a.angular_velocity, radius_a).y;

            let mut tangent = Vector2 {
                x: radius_v.x - (manifold.normal.x*radius_v.dot(manifold.normal)),
                y: radius_v.y - (manifold.normal.y*radius_v.dot(manifold.normal)),
            };
            math_normalize(&mut tangent);

            // Calculate impulse tangent magnitude
            let mut impulse_tangent = -radius_v.dot(tangent);
            impulse_tangent /= inverse_mass_sum;
            impulse_tangent /= manifold.contacts_count as f32;

            let abs_impulse_tangent = impulse_tangent.abs();

            // Don't apply tiny friction impulses
            if abs_impulse_tangent <= f32::EPSILON {
                return;
            }

            // Apply coulumb's law
            let tangent_impulse = if abs_impulse_tangent < impulse*manifold.static_friction {
                Vector2 { x: tangent.x*impulse_tangent, y: tangent.y*impulse_tangent }
            } else {
                Vector2 { x: tangent.x*-impulse*manifold.dynamic_friction, y: tangent.y*-impulse*manifold.dynamic_friction }
            };

            // Apply friction impulse
            if body_a.enabled {
                body_a.velocity.x += body_a.inverse_mass*(-tangent_impulse.x);
                body_a.velocity.y += body_a.inverse_mass*(-tangent_impulse.y);

                if !body_a.freeze_orient {
                    body_a.angular_velocity += body_a.inverse_inertia*math_cross_vector2(radius_a, -tangent_impulse);
                }
            }

            if body_b.enabled {
                body_b.velocity.x += body_b.inverse_mass*(tangent_impulse.x);
                body_b.velocity.y += body_b.inverse_mass*(tangent_impulse.y);

                if !body_b.freeze_orient {
                    body_b.angular_velocity += body_b.inverse_inertia*math_cross_vector2(radius_b, tangent_impulse);
                }
            }
        }
    }

    /// Integrates physics velocity into position and forces
    fn integrate_physics_velocity(body: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, delta_time: f64, gravity_force: Vector2) {
        if !body.enabled {
            return;
        }

        body.position.x += (body.velocity.x as f64*delta_time) as f32;
        body.position.y += (body.velocity.y as f64*delta_time) as f32;

        if !body.freeze_orient {
            body.orient += (body.angular_velocity as f64*delta_time) as f32;
        }

        let orient = body.orient;
        if let PHYSICS_POLYGON { transform, .. } = &mut body.shape {
            transform.set(orient);
        }

        Self::integrate_physics_forces(body, delta_time, gravity_force);
    }

    /// Corrects physics bodies positions based on manifolds collision information
    fn correct_physics_positions(manifold: &mut PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES>, penetration_allowance: f32, penetration_correction: f32) {
        let mut body_a = manifold.body_a.borrow_mut();
        let mut body_b = manifold.body_b.borrow_mut();

        let correction = Vector2 {
            x: ((manifold.penetration - penetration_allowance).max(0.0)/(body_a.inverse_mass + body_b.inverse_mass))*manifold.normal.x*penetration_correction,
            y: ((manifold.penetration - penetration_allowance).max(0.0)/(body_a.inverse_mass + body_b.inverse_mass))*manifold.normal.y*penetration_correction,
        };

        if body_a.enabled {
            body_a.position.x -= correction.x*body_a.inverse_mass;
            body_a.position.y -= correction.y*body_a.inverse_mass;
        }

        if body_b.enabled {
            body_b.position.x += correction.x*body_b.inverse_mass;
            body_b.position.y += correction.y*body_b.inverse_mass;
        }
    }

    /// Initializes hi-resolution MONOTONIC timer
    fn init_timer(&mut self) {
        self.base_time = Instant::now(); // Get MONOTONIC clock time offset
        self.start_time = self.get_curr_time(); // Get current time
    }

    /// Get current time in milliseconds
    fn get_curr_time(&self) -> f64 {
        let duration = self.base_time.elapsed();
        duration.as_secs_f64() * 1_000.0
    }
}

impl<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize> PhysicsManifoldData<MAX_VERTICES, CIRCLE_VERTICES> {
    /// Solves a created physics manifold between two physics bodies
    fn solve(&mut self, body_a: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, body_b: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>) {
        match body_a.shape {
            PHYSICS_CIRCLE { .. } => match body_b.shape {
                PHYSICS_CIRCLE { .. } => self.solve_circle_to_circle(body_a, body_b),
                PHYSICS_POLYGON { .. } => self.solve_circle_to_polygon(body_a, body_b),
            }
            PHYSICS_POLYGON { .. } => match body_b.shape {
                PHYSICS_CIRCLE { .. } => self.solve_polygon_to_circle(body_a, body_b),
                PHYSICS_POLYGON { .. } => self.solve_polygon_to_polygon(body_a, body_b),
            }
        }

        // Update physics body grounded state if normal direction is down and grounded state is not set yet in previous manifolds
        if !body_b.is_grounded {
            body_b.is_grounded = self.normal.y < 0.0;
        }
    }

    // Solves collision between two circle shape physics bodies
    fn solve_circle_to_circle(&mut self, body_a: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, body_b: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>) {
        let PHYSICS_CIRCLE { radius: radus_a } = body_a.shape else { panic!("only circle bodies should be passed to solve_circle_to_circle") };
        let PHYSICS_CIRCLE { radius: radus_b } = body_b.shape else { panic!("only circle bodies should be passed to solve_circle_to_circle") };

        // Calculate translational vector, which is normal
        let normal = body_b.position - body_a.position;

        let dist_sqr = normal.length_sqr();
        let radius = radus_a + radus_b;

        // Check if circles are not in contact
        if dist_sqr >= radius*radius {
            self.contacts_count = 0;
            return;
        }

        let distance = dist_sqr.sqrt();
        self.contacts_count = 1;

        if distance == 0.0 {
            self.penetration = radus_a;
            self.normal = Vector2 { x: 1.0, y: 0.0 };
            self.contacts[0] = body_a.position;
        } else {
            self.penetration = radius - distance;
            self.normal = Vector2 { x: normal.x/distance, y: normal.y/distance }; // Faster than using MathNormalize() due to sqrt is already performed
            self.contacts[0] = Vector2 { x: self.normal.x*radus_a + body_a.position.x, y: self.normal.y*radus_a + body_a.position.y };
        }

        // Update physics body grounded state if normal direction is down
        if !body_a.is_grounded {
            body_a.is_grounded = self.normal.y < 0.0;
        }
    }

    // Solves collision between a circle to a polygon shape physics bodies
    fn solve_circle_to_polygon(&mut self, body_a: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, body_b: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>) {
        self.solve_different_shapes(body_a, body_b);
    }

    // Solves collision between a circle to a polygon shape physics bodies
    fn solve_polygon_to_circle(&mut self, body_a: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, body_b: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>) {
        self.solve_different_shapes(body_b, body_a);

        self.normal.x *= -1.0;
        self.normal.y *= -1.0;
    }

    // Solve collision between two different types of shapes
    fn solve_different_shapes(&mut self, body_a: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, body_b: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>) {
        let PHYSICS_CIRCLE { radius: radius_a } = body_a.shape else { panic!("only circle bodies should be passed to solve_different_shapes body_a") };
        let PHYSICS_POLYGON { vertex_data: vertex_data_b, transform: transform_b } = &body_b.shape else { panic!("only circle bodies should be passed to solve_different_shapes body_b") };

        self.contacts_count = 0;

        // Transform circle center to polygon transform space
        let mut center = body_a.position;
        center = transform_b.transpose().multiply_vector2(center - body_b.position);

        // Find edge with minimum penetration
        // It is the same concept as using support points in SolvePolygonToPolygon
        let mut separation = f32::MIN;
        let mut face_normal = 0;
        let vertex_data = vertex_data_b;

        for i in 0..vertex_data.vertex_count {
            let current_separation = vertex_data.normals[i].dot(center - vertex_data.positions[i]);

            if current_separation > radius_a {
                return;
            }

            if current_separation > separation {
                separation = current_separation;
                face_normal = i;
            }
        }

        // Grab face's vertices
        let mut v1 = vertex_data.positions[face_normal];
        let next_index = next_idx(face_normal, vertex_data.vertex_count);
        let mut v2 = vertex_data.positions[next_index];

        // Check to see if center is within polygon
        if separation < f32::EPSILON {
            self.contacts_count = 1;
            let normal = transform_b.multiply_vector2(vertex_data.normals[face_normal]);
            self.normal = -normal;
            self.contacts[0] = Vector2 { x: self.normal.x*radius_a + body_a.position.x, y: self.normal.y*radius_a + body_a.position.y };
            self.penetration = radius_a;
            return;
        }

        // Determine which voronoi region of the edge center of circle lies within
        let dot1 = (center - v1).dot(v2 - v1);
        let dot2 = (center - v2).dot(v1 - v2);
        self.penetration = radius_a - separation;

        if dot1 <= 0.0 { // Closest to v1
            if dist_sqr(center, v1) > radius_a*radius_a {
                return;
            }

            self.contacts_count = 1;
            let mut normal = v1 - center;
            normal = transform_b.multiply_vector2(normal);
            math_normalize(&mut normal);
            self.normal = normal;
            v1 = transform_b.multiply_vector2(v1);
            v1 += body_b.position;
            self.contacts[0] = v1;
        } else if dot2 <= 0.0 { // Closest to v2
            if dist_sqr(center, v2) > radius_a*radius_a {
                return;
            }

            self.contacts_count = 1;
            let mut normal = v2 - center;
            v2 = transform_b.multiply_vector2(v2);
            v2 += body_b.position;
            self.contacts[0] = v2;
            normal = transform_b.multiply_vector2(normal);
            math_normalize(&mut normal);
            self.normal = normal;
        } else { // Closest to face
            let mut normal = vertex_data.normals[face_normal];

            if (center - v1).dot(normal) > radius_a {
                return;
            }

            normal = transform_b.multiply_vector2(normal);
            self.normal = Vector2 { x: -normal.x, y: -normal.y };
            self.contacts[0] = Vector2 { x: self.normal.x*radius_a + body_a.position.x, y: self.normal.y*radius_a + body_a.position.y };
            self.contacts_count = 1;
        }
    }

    // Solves collision between two polygons shape physics bodies
    fn solve_polygon_to_polygon(&mut self, body_a: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, body_b: &mut PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>) {
        self.contacts_count = 0;

        // Check for separating axis with A shape's face planes
        let mut face_a = 0;
        let penetration_a = find_axis_least_penetration(&mut face_a, body_a, body_b);

        if penetration_a >= 0.0 {
            return;
        }

        // Check for separating axis with B shape's face planes
        let mut face_b = 0;
        let penetration_b = find_axis_least_penetration(&mut face_b, body_b, body_a);

        if penetration_b >= 0.0 {
            return;
        }

        let mut reference_index;
        let mut flip = false;  // Always point from A shape to B shape

        let ref_body; // Reference
        let inc_body; // Incident

        // Determine which shape contains reference face
        if bias_greater_than(penetration_a, penetration_b) {
            ref_body = body_a;
            inc_body = body_b;
            reference_index = face_a;
        } else {
            ref_body = body_b;
            inc_body = body_a;
            reference_index = face_b;
            flip = true;
        }
        let PHYSICS_POLYGON { vertex_data: ref_data, transform: ref_transform } = &ref_body.shape else { panic!("only polygon bodies should be passed to solve_polygon_to_polygon") };
        // let PHYSICS_POLYGON { vertex_data: inc_data, transform: inc_transform } = &inc_body.shape else { panic!("only polygon bodies should be passed to solve_polygon_to_polygon") };

        // World space incident face
        let mut incident_face = find_incident_face(ref_body, inc_body, reference_index);

        // Setup reference face vertices
        let mut v1 = ref_data.positions[reference_index];
        reference_index = next_idx(reference_index, ref_data.vertex_count);
        let mut v2 = ref_data.positions[reference_index];

        // Transform vertices to world space
        v1 = ref_transform.multiply_vector2(v1);
        v1 += ref_body.position;
        v2 = ref_transform.multiply_vector2(v2);
        v2 += ref_body.position;

        // Calculate reference face side normal in world space
        let mut side_plane_normal = v2 - v1;
        math_normalize(&mut side_plane_normal);

        // Orthogonalize
        let ref_face_normal = Vector2 { x: side_plane_normal.y, y: -side_plane_normal.x };
        let ref_c = ref_face_normal.dot(v1);
        let neg_side = side_plane_normal.dot(v1)*-1.0;
        let pos_side = side_plane_normal.dot(v2);

        // Clip incident face to reference face side planes (due to floating point error, possible to not have required points
        let [face_a, face_b] = &mut incident_face;
        if clip(-side_plane_normal, neg_side, face_a, face_b) < 2 {
            return;
        }

        if clip(side_plane_normal, pos_side, face_a, face_b) < 2 {
            return;
        }

        // Flip normal if required
        self.normal = if flip { -ref_face_normal } else { ref_face_normal };

        // Keep points behind reference face
        let mut current_point: u32 = 0; // Clipped points behind reference face
        let mut separation = ref_face_normal.dot(incident_face[0]) - ref_c;

        if separation <= 0.0 {
            self.contacts[current_point as usize] = incident_face[0];
            self.penetration = -separation;
            current_point += 1;
        } else {
            self.penetration = 0.0;
        }

        separation = ref_face_normal.dot(incident_face[1]) - ref_c;

        if separation <= 0.0 {
            self.contacts[current_point as usize] = incident_face[1];
            self.penetration += -separation;
            current_point += 1;

            // Calculate total penetration average
            self.penetration /= current_point as f32;
        }

        self.contacts_count = current_point;
    }
}

/// Returns the extreme point along a direction within a polygon
fn get_support<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize>(vertex_data: &PolygonData<MAX_VERTICES, CIRCLE_VERTICES>, dir: Vector2) -> Vector2 {
    let mut best_projection = -f32::MIN_POSITIVE;
    let mut best_vertex = Vector2 { x: 0.0, y: 0.0 };

    for i in 0..vertex_data.vertex_count {
        let vertex = vertex_data.positions[i];
        let projection = vertex.dot(dir);

        if projection > best_projection {
            best_vertex = vertex;
            best_projection = projection;
        }
    }

    best_vertex
}

/// Finds polygon shapes axis least penetration
fn find_axis_least_penetration<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize>(
    face_index: &mut usize,
    body_a: &PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>,
    body_b: &PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>,
) -> f32 {
    let shape_a = &body_a.shape;
    let shape_b = &body_b.shape;

    let PHYSICS_POLYGON { vertex_data: data_a, transform: transform_a } = &shape_a else { panic!("only polygons should be passed to find_axis_least_penetration") };
    let PHYSICS_POLYGON { vertex_data: data_b, transform: transform_b } = &shape_b else { panic!("only polygons should be passed to find_axis_least_penetration") };

    let mut best_distance = f32::MIN;
    let mut best_index = 0;

    for i in 0..data_a.vertex_count {
        // Retrieve a face normal from A shape
        let mut normal = data_a.normals[i];
        let trans_normal = transform_a.multiply_vector2(normal);

        // Transform face normal into B shape's model space
        let bu_t = transform_b.transpose();
        normal = bu_t.multiply_vector2(trans_normal);

        // Retrieve support point from B shape along -n
        let support = get_support(data_b, Vector2 { x: -normal.x, y: -normal.y });

        // Retrieve vertex on face from A shape, transform into B shape's model space
        let mut vertex = data_a.positions[i];
        vertex = transform_a.multiply_vector2(vertex);
        vertex += body_a.position;
        vertex -= body_b.position;
        vertex = bu_t.multiply_vector2(vertex);

        // Compute penetration distance in B shape's model space
        let distance = normal.dot(support - vertex);

        // Store greatest distance
        if distance > best_distance {
            best_distance = distance;
            best_index = i;
        }
    }

    *face_index = best_index;
    best_distance
}

/// Finds two polygon shapes incident face
fn find_incident_face<const MAX_VERTICES: usize, const CIRCLE_VERTICES: usize>(ref_body: &PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, inc_body: &PhysicsBodyData<MAX_VERTICES, CIRCLE_VERTICES>, index: usize) -> [Vector2; 2] {
    let ref_shape = &ref_body.shape;
    let inc_shape = &inc_body.shape;

    let PHYSICS_POLYGON { vertex_data: ref_data, transform: ref_transform } = &ref_shape else { panic!("only polygons should be passed to find_incident_face") };
    let PHYSICS_POLYGON { vertex_data: inc_data, transform: inc_transform } = &inc_shape else { panic!("only polygons should be passed to find_incident_face") };

    let mut reference_normal = ref_data.normals[index];

    // Calculate normal in incident's frame of reference
    reference_normal = ref_transform.multiply_vector2(reference_normal); // To world space
    reference_normal = inc_transform.transpose().multiply_vector2(reference_normal); // To incident's model space

    // Find most anti-normal face on polygon
    let mut incident_face = 0;
    let mut min_dot = f32::MAX;

    for i in 0..inc_data.vertex_count {
        let dot = reference_normal.dot(inc_data.normals[i]);

        if dot < min_dot {
            min_dot = dot;
            incident_face = i;
        }
    }

    // Assign face vertices for incident face
    let v0 = inc_transform.multiply_vector2(inc_data.positions[incident_face]) + inc_body.position;
    incident_face = next_idx(incident_face, inc_data.vertex_count);
    let v1 = inc_transform.multiply_vector2(inc_data.positions[incident_face]) + inc_body.position;
    [v0, v1]
}

/// Calculates clipping based on a normal and two faces
fn clip(normal: Vector2, clip: f32, face_a: &mut Vector2, face_b: &mut Vector2) -> usize {
    let mut sp = 0;
    let mut out = [*face_a, *face_b];

    // Retrieve distances from each endpoint to the line
    let distance_a = normal.dot(*face_a) - clip;
    let distance_b = normal.dot(*face_b) - clip;

    // If negative (behind plane)
    if distance_a <= 0.0 {
        out[sp] = *face_a;
        sp += 1;
    }

    if distance_b <= 0.0 {
        out[sp] = *face_b;
        sp += 1;
    }

    // If the points are on different sides of the plane
    if (distance_a*distance_b) < 0.0 {
        // Push intersection point
        let alpha = distance_a/(distance_a - distance_b);
        out[sp] = *face_a;
        let mut delta = *face_b - *face_a;
        delta.x *= alpha;
        delta.y *= alpha;
        out[sp] += delta;
        sp += 1;
    }

    // Assign the new converted values
    *face_a = out[0];
    *face_b = out[1];

    sp
}

/// Check if values are between bias range
fn bias_greater_than(value_a: f32, value_b: f32) -> bool {
    value_a >= (value_b*0.95 + value_a*0.01)
}

/// Returns the barycenter of a triangle given by 3 points
fn triangle_barycenter(v1: Vector2, v2: Vector2, v3: Vector2) -> Vector2 {
    Vector2 {
        x: (v1.x + v2.x + v3.x)/3.0,
        y: (v1.y + v2.y + v3.y)/3.0,
    }
}

// Returns the cross product of a vector and a value
#[inline(always)]
fn math_cross(value: f32, vector: Vector2) -> Vector2 {
    Vector2 { x: -value*vector.y, y: value*vector.x }
}

// Returns the cross product of two vectors
#[inline(always)]
fn math_cross_vector2(v1: Vector2, v2: Vector2) -> f32 {
    v1.x*v2.y - v1.y*v2.x
}

#[cfg(not(feature = "raylib"))]
impl Vector2 {
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::Neg for Vector2 {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.x,
        }
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::Add for Vector2 {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.x + rhs.y,
        }
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::Sub for Vector2 {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.x - rhs.y,
        }
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::Mul for Vector2 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.x * rhs.y,
        }
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::Mul<f32> for Vector2 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.x * rhs,
        }
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::AddAssign for Vector2 {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::SubAssign for Vector2 {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::MulAssign for Vector2 {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs
    }
}
#[cfg(not(feature = "raylib"))]
impl std::ops::MulAssign<f32> for Vector2 {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: f32) {
        *self = *self * rhs
    }
}
#[cfg(not(feature = "raylib"))]
impl Vector2 {
    #[inline(always)]
    pub const fn dot(self, other: Vector2) -> f32 {
        self.x*other.x + self.y*other.y
    }

    #[inline(always)]
    pub const fn length_sqr(self) -> f32 {
        self.dot(self)
    }
}

// Returns the square root of distance between two vectors
#[inline(always)]
fn dist_sqr(v1: Vector2, v2: Vector2) -> f32 {
    let dir = v1 - v2;
    dir.dot(dir)
}

/// Returns the normalized values of a vector
fn math_normalize(vector: &mut Vector2) {
    let (mut length, ilength): (f32, f32);

    let aux = *vector;
    length = (aux.x*aux.x + aux.y*aux.y).sqrt();

    if length == 0.0 {
        length = 1.0;
    }

    ilength = 1.0/length;

    vector.x *= ilength;
    vector.y *= ilength;
}
