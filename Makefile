cbindgen:
	cbindgen --config cbindgen.toml --crate ofibrs -o ofib_ffi.h

target/release/libofibrs.a: cbindgen
	cargo build --release

lib: target/release/libofibrs.a

all: cbindgen
	gcc main.c -o main  -I../proto/ospf/ -I../sysdeps -I../obj -I.. -Ltarget/release -lofibrs

clean:
	if [[ -f target/release/libofibrs.a ]]; then rm target/release/libofibrs.a; fi
test:
	cargo test --release
