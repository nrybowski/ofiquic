# oFIQUIC Ordering Module

This repository is part of the [oFIQUIC project](https://github.com/nrybowski/ofiquic-artefacts).
It contains the ordering module written in `Rust`, as well as its `C` FFI.

The library is built by running:

```console
$ cargo build --release
OR
$ make lib
```

You can launch the built-in tests with:

```console
$ cargo test
OR
$ make test
```

The FFI header file is not expected to be built outside of the BIRD repository as it requires some specific header files form BIRD.
