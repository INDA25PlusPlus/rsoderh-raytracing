# rsoderh Path Tracer

The rays do be tracin' 🫣

This is my path tracer, which was a homework assignment in the Inda++ group.

<img src="./docs/render-house.png" width="80%" alt="Example render of the path tracer.">

## Running

There is a Justfile with convenient commands for running the application:
```shell
just run
```

Otherwise you can just manually run the Cargo project. Altough you need to specify the scene file in that case.
```shell
cargo run -- --scene="assets/scenes/default.toml"
```
You can also pass `--scene` to the Just command. There is another scene with a small house btw :)
```shell
just run "info" --scene="assets/scenes/house.toml"
```
(**Note:** you also need to specify the log level when passing `--scene` to the Just command.)
