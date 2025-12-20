use std::borrow::Cow;

#[cfg(feature = "runtime-shader-src")]
#[doc(hidden)]
pub fn _load_path_filesystem(shader_file_path: &str, source_file_path: &str) -> Cow<'static, str> {
    use std::{fs, path};

    let mut path = path::PathBuf::from(source_file_path);
    let _ = path.pop();
    path.push(shader_file_path);

    let source =
        fs::read_to_string(path).expect("File exists, contains utf-8 text, and can be read");
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

/// Load WGSL source code from a file at compile time or run time, depending on if the
/// `runtime-shader-src` feature is enabled.
///
/// The loaded path is relative to the path of the file containing the macro call, in the same way
/// as [`include_str!`] operates.
macro_rules! include_wgsl {
    ($path:expr $(,)?) => {{
        use crate::shader::*;

        #[cfg(feature = "runtime-shader-src")]
        let source = _load_path_filesystem($path, file!());

        #[cfg(not(feature = "runtime-shader-src"))]
        let source = std::borrow::Cow::Borrowed(include_str!($path));

        _assemble_shader_module_descriptor($path, source)
    }};
}

pub(crate) use include_wgsl;
