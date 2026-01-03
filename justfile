run log_level="":
    RUST_LOG="{{log_level}}" cargo run

# Run with the runtime-asset-src feature enabled, useful when developing.
run-dev log_level="":
    RUST_LOG="{{log_level}}" cargo run --features runtime-asset-src

# Run with a workman friendly layout and the the runtime-asset-src feature enabled.
run-dev-workman log_level="":
    RUST_LOG="{{log_level}}" cargo run --features runtime-asset-src -- \
        --movement-keys="dash"

