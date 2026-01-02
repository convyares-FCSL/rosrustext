use std::env;

pub const DEFAULT_TARGET_NODE: &str = "hyfleet_ring_roslibrust";
pub const DEFAULT_BRIDGE_URL: &str = "ws://localhost:9090";

pub struct Config {
    pub node_name: String,
    pub target_node: String,
    pub bridge_url: String,
    pub bond_enabled: bool,
}

impl Config {
    pub fn from_args() -> Self {
        Self::from_args_iter(env::args())
    }

    pub fn from_args_iter<I, S>(iter: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        let mut node_name: Option<String> = None;
        let mut target_node =
            env::var("ROSRUSTEXT_TARGET_NODE").unwrap_or_else(|_| DEFAULT_TARGET_NODE.to_string());
        let mut bridge_url =
            env::var("ROSRUSTEXT_BRIDGE_URL").unwrap_or_else(|_| DEFAULT_BRIDGE_URL.to_string());
        let mut bond_enabled = env::var("ROSRUSTEXT_BOND")
            .ok()
            .and_then(parse_bool)
            .unwrap_or(true);

        let mut args = iter.into_iter();
        let _ = args.next();
        while let Some(arg) = args.next() {
            let arg = arg.as_ref();
            match arg {
                "-h" | "--help" => {
                    print_usage();
                    std::process::exit(0);
                }
                "--node-name" => {
                    if let Some(value) = args.next() {
                        node_name = Some(value.as_ref().to_string());
                    }
                }
                "--target-node" => {
                    if let Some(value) = args.next() {
                        target_node = value.as_ref().to_string();
                    }
                }
                "--bridge-url" => {
                    if let Some(value) = args.next() {
                        bridge_url = value.as_ref().to_string();
                    }
                }
                "--no-bond" => {
                    bond_enabled = false;
                }
                _ if arg.starts_with("--node-name=") => {
                    node_name = Some(arg["--node-name=".len()..].to_string());
                }
                _ if arg.starts_with("--target-node=") => {
                    target_node = arg["--target-node=".len()..].to_string();
                }
                _ if arg.starts_with("--bridge-url=") => {
                    bridge_url = arg["--bridge-url=".len()..].to_string();
                }
                _ => {}
            }
        }

        let node_name = node_name.unwrap_or_else(|| target_node.clone());

        Self {
            node_name,
            target_node,
            bridge_url,
            bond_enabled,
        }
    }
}

fn print_usage() {
    println!(
        "rosrustext_lifecycle_proxy --target-node <name> [--node-name <name>] [--bridge-url ws://host:port] [--no-bond]"
    );
}

fn parse_bool(value: String) -> Option<bool> {
    match value.trim().to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => Some(true),
        "0" | "false" | "no" | "off" => Some(false),
        _ => None,
    }
}
