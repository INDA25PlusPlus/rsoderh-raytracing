use winit::event_loop::EventLoop;

use crate::{app::App, camera::KeyboardLayout, scene::Scene};

pub mod app;
pub(crate) mod asset;
pub(crate) mod bvh;
pub(crate) mod camera;
mod cli;
pub(crate) mod environments;
pub(crate) mod hdr;
pub(crate) mod mesh;
pub(crate) mod scene;
pub mod state;
pub(crate) mod texture;

pub use cli::cli;

pub fn run(layout: KeyboardLayout, scene: Scene) -> anyhow::Result<()> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        env_logger::init();
    }
    #[cfg(target_arch = "wasm32")]
    {
        console_log::init_with_level(log::Level::Info).unwrap_throw();
    }

    let event_loop = EventLoop::with_user_event().build()?;
    let mut app = App::new(
        layout,
        scene,
        #[cfg(target_arch = "wasm32")]
        &event_loop,
    );
    event_loop.run_app(&mut app)?;

    Ok(())
}
