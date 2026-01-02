use std::collections::BTreeSet;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let packages = [
        "lifecycle_msgs",
        "bond",
        "std_msgs",
        "rosrustext_interfaces",
    ];
    let mut search_paths = Vec::new();

    let local_override = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("interfaces");
    let local_bond = local_override.join("bond");
    let use_local_bond = local_bond.join("package.xml").is_file();
    if local_override.is_dir() {
        search_paths.push(local_override);
    }
    if use_local_bond {
        search_paths.push(local_bond);
    }

    if let Ok(path) = std::env::var("ROSRUSTEXT_INTERFACES_PATH") {
        let candidate = PathBuf::from(path);
        if candidate.join("package.xml").is_file() && !search_paths.contains(&candidate) {
            search_paths.push(candidate.clone());
        }
        let candidate_bond = candidate.join("bond");
        if candidate_bond.join("package.xml").is_file() && !search_paths.contains(&candidate_bond) {
            search_paths.push(candidate_bond);
        }
    } else {
        let repo_interfaces = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("interfaces");
        if repo_interfaces.join("package.xml").is_file() && !search_paths.contains(&repo_interfaces)
        {
            search_paths.push(repo_interfaces.clone());
        }
        let repo_bond = repo_interfaces.join("bond");
        if repo_bond.join("package.xml").is_file() && !search_paths.contains(&repo_bond) {
            search_paths.push(repo_bond);
        }
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
            if path.join("package.xml").is_file() || path.join(pkg).join("package.xml").is_file() {
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
