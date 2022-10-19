#!/bin/bash

# Build the rust library, which produces `wave.a` and `wave.h`
cd wave
cargo build --release --target=thumbv7em-none-eabihf
cbindgen --config cbindgen.toml --crate wave --output wave.h

