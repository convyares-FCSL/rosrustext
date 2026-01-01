use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let mut search_paths = Vec::new();

    if let Some(prefixes) = std::env::var_os("AMENT_PREFIX_PATH") {
        let candidates = std::env::split_paths(&prefixes)
            .map(|prefix| prefix.join("share").join("lifecycle_msgs"))
            .filter(|path| path.is_dir());
        search_paths.extend(candidates);
    }

    let fallback = PathBuf::from("/opt/ros/jazzy/share/lifecycle_msgs");
    if fallback.is_dir() && !search_paths.contains(&fallback) {
        search_paths.push(fallback);
    }

    if search_paths.is_empty() {
        return Err("lifecycle_msgs not found in AMENT_PREFIX_PATH or /opt/ros/jazzy".into());
    }

    let (source, dependent_paths) =
        roslibrust::codegen::find_and_generate_ros_messages_without_ros_package_path(search_paths)?;

    let out_dir = PathBuf::from(std::env::var("OUT_DIR")?);
    let dest_path = out_dir.join("messages.rs");
    std::fs::write(dest_path, source.to_string())?;

    for path in dependent_paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    Ok(())
}
