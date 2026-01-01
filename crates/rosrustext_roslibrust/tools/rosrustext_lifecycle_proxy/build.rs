use std::collections::BTreeSet;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let packages = ["lifecycle_msgs", "bond", "std_msgs"];
    let mut search_paths = Vec::new();

    let local_override = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ros_msgs");
    let use_local_bond = local_override.join("bond").join("package.xml").is_file();
    if local_override.is_dir() {
        search_paths.push(local_override);
    }

    if let Some(prefixes) = std::env::var_os("AMENT_PREFIX_PATH") {
        for prefix in std::env::split_paths(&prefixes) {
            for pkg in packages {
                if pkg == "bond" && use_local_bond {
                    continue;
                }
                let candidate = prefix.join("share").join(pkg);
                if candidate.is_dir() && !search_paths.contains(&candidate) {
                    search_paths.push(candidate);
                }
            }
        }
    }

    for pkg in packages {
        if pkg == "bond" && use_local_bond {
            continue;
        }
        let fallback = PathBuf::from(format!("/opt/ros/jazzy/share/{pkg}"));
        if fallback.is_dir() && !search_paths.contains(&fallback) {
            search_paths.push(fallback);
        }
    }

    let mut found = BTreeSet::new();
    for pkg in packages {
        let mut present = false;
        for path in &search_paths {
            if path.join("package.xml").is_file() || path.join(pkg).join("package.xml").is_file()
            {
                present = true;
                break;
            }
        }
        if !present && pkg == "bond" && use_local_bond {
            present = true;
        }
        if present {
            found.insert(pkg);
        }
    }

    if found.len() != packages.len() {
        let missing: Vec<_> = packages
            .iter()
            .copied()
            .filter(|pkg| !found.contains(pkg))
            .collect();
        return Err(format!(
            "missing ROS packages in AMENT_PREFIX_PATH or /opt/ros/jazzy: {}",
            missing.join(", ")
        )
        .into());
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
