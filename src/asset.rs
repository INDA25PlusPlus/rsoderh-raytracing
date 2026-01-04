use std::borrow::Cow;

#[cfg(feature = "runtime-asset-src")]
#[doc(hidden)]
pub fn _load_path_filesystem_str(file_path: &str, source_file_path: &str) -> Cow<'static, str> {
    use std::{fs, path};

    let mut path = path::PathBuf::from(source_file_path);
    let _ = path.pop();
    path.push(file_path);

    let source =
        fs::read_to_string(path).expect("File exists, contains utf-8 text, and can be read");
    source.into()
}

#[cfg(feature = "runtime-asset-src")]
#[doc(hidden)]
pub fn _load_path_filesystem_bytes(file_path: &str, source_file_path: &str) -> Cow<'static, [u8]> {
    use std::{fs, path};

    let mut path = path::PathBuf::from(source_file_path);
    let _ = path.pop();
    path.push(file_path);

    let source = fs::read(path).expect("File exists, and can be read");
    source.into()
}

#[doc(hidden)]
pub fn _assemble_shader_module_descriptor(
    file_path: &'static str,
    source: Cow<'static, str>,
) -> wgpu::ShaderModuleDescriptor<'static> {
    wgpu::ShaderModuleDescriptor {
        label: Some(file_path),
        source: wgpu::ShaderSource::Wgsl(source),
    }
}

macro_rules! include_str {
    ($path:expr $(,)?) => {{
        #[cfg(feature = "runtime-asset-src")]
        let source = crate::asset::_load_path_filesystem_str($path, file!());

        #[cfg(not(feature = "runtime-asset-src"))]
        let source = std::borrow::Cow::Borrowed(core::include_str!($path));

        source
    }};
}

macro_rules! include_bytes {
    ($path:expr $(,)?) => {{
        #[cfg(feature = "runtime-asset-src")]
        let source = crate::asset::_load_path_filesystem_bytes($path, file!());

        #[cfg(not(feature = "runtime-asset-src"))]
        let source = std::borrow::Cow::Borrowed(core::include_bytes!($path).as_slice());

        source
    }};
}

/// Load WGSL source code from a file at compile time or run time, depending on if the
/// `runtime-asset-src` feature is enabled.
///
/// The loaded path is relative to the path of the file containing the macro call, in the same way
/// as [`include_str!`] operates.
macro_rules! include_wgsl {
    ($path:expr $(,)?) => {
        crate::asset::_assemble_shader_module_descriptor($path, crate::asset::include_str!($path))
    };
}

pub(crate) use include_bytes;
pub(crate) use include_str;
pub(crate) use include_wgsl;
