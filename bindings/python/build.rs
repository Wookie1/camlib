//! Link the Python extension the way CPython loads it on every platform.
//!
//! A CPython extension module must not link libpython: the interpreter that
//! imports it provides the Python symbols. Linux and Windows linkers accept
//! that by default, but Apple's linker refuses the undefined symbols unless the
//! module is linked with `-undefined dynamic_lookup`. PyO3 provides this build
//! script helper for its `extension-module` feature; it adds exactly that flag
//! on macOS and nothing on Linux or Windows:
//! <https://docs.rs/pyo3-build-config/0.27.2/pyo3_build_config/fn.add_extension_module_link_args.html>.
fn main() {
    pyo3_build_config::add_extension_module_link_args();
}
