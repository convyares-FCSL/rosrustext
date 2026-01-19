use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let bond_enabled = std::env::var_os("CARGO_FEATURE_BOND").is_some();
    let mut packages = vec!["lifecycle_msgs", "std_msgs"];
    if bond_enabled {
        packages.push("bond");
    }
    let mut search_paths = Vec::new();

    if let Some(prefixes) = std::env::var_os("AMENT_PREFIX_PATH") {
        for prefix in std::env::split_paths(&prefixes) {
            for pkg in &packages {
                let candidate = prefix.join("share").join(pkg);
                if candidate.is_dir() && !search_paths.contains(&candidate) {
                    search_paths.push(candidate);
                }
            }
        }
    }

    for pkg in &packages {
        let fallback = PathBuf::from(format!("/opt/ros/jazzy/share/{pkg}"));
        if fallback.is_dir() && !search_paths.contains(&fallback) {
            search_paths.push(fallback);
        }
    }

    if bond_enabled {
        normalize_bond_constants(&mut search_paths)?;
    }

    let mut found = BTreeSet::new();
    for pkg in &packages {
        let mut present = false;
        for path in &search_paths {
            if path.join("package.xml").is_file() || path.join(pkg).join("package.xml").is_file() {
                present = true;
                break;
            }
        }
        if present {
            found.insert(pkg);
        }
    }

    if found.len() != packages.len() {
        let missing: Vec<_> = packages.iter().copied().filter(|pkg| !found.contains(pkg)).collect();
        return Err(
            format!("missing ROS packages in AMENT_PREFIX_PATH or /opt/ros/jazzy: {}", missing.join(", ")).into()
        );
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

fn normalize_bond_constants(search_paths: &mut Vec<PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let source_dir = search_paths
        .iter()
        .find(|path| path.file_name().is_some_and(|name| name == "bond") && path.join("package.xml").is_file())
        .cloned();
    let Some(source_dir) = source_dir else {
        return Ok(());
    };

    let out_dir = PathBuf::from(std::env::var("OUT_DIR")?);
    let patched_dir = out_dir.join("rosrustext_bond");
    if patched_dir.exists() {
        std::fs::remove_dir_all(&patched_dir)?;
    }

    copy_dir_all(&source_dir, &patched_dir)?;

    let constants_path = patched_dir.join("msg").join("Constants.msg");
    if constants_path.is_file() {
        let content = std::fs::read_to_string(&constants_path)?;
        let mut updated = Vec::new();
        for line in content.lines() {
            updated.push(normalize_string_constant(line));
        }
        std::fs::write(&constants_path, updated.join("\n"))?;
    }

    search_paths
        .retain(|path| !(path.file_name().is_some_and(|name| name == "bond") && path.join("package.xml").is_file()));
    search_paths.insert(0, patched_dir);
    Ok(())
}

fn normalize_string_constant(line: &str) -> String {
    let Some((lhs, rhs)) = line.split_once('=') else {
        return line.to_string();
    };
    if !lhs.trim_start().starts_with("string ") {
        return line.to_string();
    }
    let rhs = rhs.trim().trim_end_matches(';').trim();
    if rhs.len() >= 2 {
        let first = rhs.chars().next().unwrap();
        let last = rhs.chars().last().unwrap();
        if first == last && (first == '"' || first == '\'') {
            return format!("{}={}", lhs.trim_end(), rhs);
        }
    }
    format!("{}=\"{}\"", lhs.trim_end(), rhs)
}

fn copy_dir_all(src: &Path, dst: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dst)?;
    for entry in std::fs::read_dir(src)? {
        let entry = entry?;
        let file_type = entry.file_type()?;
        let dest_path = dst.join(entry.file_name());
        if file_type.is_dir() {
            copy_dir_all(&entry.path(), &dest_path)?;
        } else {
            std::fs::copy(entry.path(), dest_path)?;
        }
    }
    Ok(())
}
