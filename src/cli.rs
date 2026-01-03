use std::process::exit;

use clap::Parser as ClapParser;

use crate::camera::KeyboardLayout;

#[derive(ClapParser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Keys used to move camera as a string of (case-insensitive) character symbols.
    #[arg(long, default_value = "wasd")]
    movement_keys: String,
    /// Key used to move toggle mouse capture as a (case-insensitive) character symbol.
    #[arg(long, default_value = "c")]
    capture_mouse_key: String,
}

pub fn cli() -> anyhow::Result<()> {
    let args = Args::parse();

    let layout = KeyboardLayout::parse_config(&args.movement_keys, &args.capture_mouse_key)
        .unwrap_or_else(|error| {
            eprintln!("Invalid keyboard config: {}", error);
            exit(2)
        });

    crate::run(layout)
}
