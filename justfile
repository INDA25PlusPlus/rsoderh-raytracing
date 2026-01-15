run log_level="" *args:
    RUST_LOG="{{log_level}}" cargo run -- \
        --scene="assets/scenes/default.toml" {{args}}

# Run with the runtime-asset-src feature enabled, useful when developing.
run-dev log_level="" *args:
    RUST_LOG="{{log_level}}" cargo run --features runtime-asset-src -- \
        --scene="assets/scenes/default.toml" {{args}}
        

# Run with a workman friendly layout and the the runtime-asset-src feature enabled.
run-dev-workman log_level="" *args:
    RUST_LOG="{{log_level}}" cargo run --features runtime-asset-src -- \
        --scene="assets/scenes/default.toml" --movement-keys="dashqr" {{args}}

