use std::env;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use roslibrust::rosbridge::ClientHandle;

use rosrustext_lifecycle_proxy::std_msgs::String as RosString;
use rosrustext_roslibrust::lifecycle::{
    CallbackResult, LifecycleCallbacks, LifecycleNode, ManagedInterval, ManagedPublisher, RosbridgePublisher,
};
use rosrustext_roslibrust::transport::roslibrust::register_lifecycle_backend_rosbridge;
use rosrustext_roslibrust::transport::roslibrust::lifecycle::LifecycleService;

const DEFAULT_NODE_NAME: &str = "rosrustext_lifecycle_demo";
const DEFAULT_PUBLISH_TOPIC: &str = "demo_tick";

struct Config {
    bridge_url: String,
    node_name: String,
    publish_topic: String,
    publish_interval_ms: u64,
}

struct DemoCallbacks;

impl LifecycleCallbacks for DemoCallbacks {
    fn on_configure(&mut self) -> CallbackResult {
        println!("[callbacks] on_configure");
        CallbackResult::Success
    }

    fn on_activate(&mut self) -> CallbackResult {
        println!("[callbacks] on_activate");
        CallbackResult::Success
    }

    fn on_deactivate(&mut self) -> CallbackResult {
        println!("[callbacks] on_deactivate");
        CallbackResult::Success
    }

    fn on_cleanup(&mut self) -> CallbackResult {
        println!("[callbacks] on_cleanup");
        CallbackResult::Success
    }

    fn on_shutdown(&mut self) -> CallbackResult {
        println!("[callbacks] on_shutdown");
        CallbackResult::Success
    }

    fn on_error(&mut self) -> CallbackResult {
        println!("[callbacks] on_error");
        CallbackResult::Success
    }
}

fn usage() -> String {
    let cmd = env::args().next().unwrap_or_else(|| "rosrustext_lifecycle_demo".to_string());
    format!(
        "{cmd} [--bridge-url ws://host:port] [--node-name <name>] [--publish-topic <name>] [--publish-ms <ms>]\n\n"
    )
}

fn parse_args() -> Result<Config, String> {
    let mut bridge_url = env::var("BRIDGE_URL").unwrap_or_else(|_| "ws://localhost:9090".to_string());
    let mut node_name = DEFAULT_NODE_NAME.to_string();
    let mut publish_topic = DEFAULT_PUBLISH_TOPIC.to_string();
    let mut publish_interval_ms = 500u64;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--bridge-url" => {
                bridge_url = args.next().ok_or_else(|| "--bridge-url requires a value".to_string())?;
            }
            "--node-name" => {
                node_name = args.next().ok_or_else(|| "--node-name requires a value".to_string())?;
            }
            "--publish-topic" => {
                publish_topic = args.next().ok_or_else(|| "--publish-topic requires a value".to_string())?;
            }
            "--publish-ms" => {
                publish_interval_ms = args
                    .next()
                    .ok_or_else(|| "--publish-ms requires a value".to_string())?
                    .parse::<u64>()
                    .map_err(|_| "--publish-ms must be a number".to_string())?;
            }
            "-h" | "--help" => {
                return Err(usage());
            }
            other => {
                return Err(format!("unknown arg: {other}\n\n{}", usage()));
            }
        }
    }

    Ok(Config { bridge_url, node_name, publish_topic, publish_interval_ms })
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = match parse_args() {
        Ok(config) => config,
        Err(help) => {
            eprintln!("{help}");
            return Ok(());
        }
    };

    let ros = ClientHandle::new(&config.bridge_url).await?;

    let lifecycle = Arc::new(Mutex::new(LifecycleNode::new(
        config.node_name.clone(),
        Box::new(DemoCallbacks),
    )?));
    let service = LifecycleService::new(Arc::clone(&lifecycle));
    register_lifecycle_backend_rosbridge(&ros, &config.node_name, Arc::clone(&lifecycle)).await?;

    let gate = {
        let guard = lifecycle.lock().expect("lifecycle node poisoned");
        guard.activation_gate()
    };

    let publish_topic = format!("/{}/{}", config.node_name, config.publish_topic);
    let publish_topic_for_log = publish_topic.clone();
    let pub_handle = Arc::new(ros.advertise::<RosString>(&publish_topic).await?);
    let managed_pub = Arc::new(ManagedPublisher::new(
        gate.clone(),
        Arc::new(RosbridgePublisher(pub_handle)),
    ));

    let counter = Arc::new(AtomicUsize::new(0));
    let interval = ManagedInterval::new(gate, Duration::from_millis(config.publish_interval_ms));
    tokio::spawn({
        let managed_pub = Arc::clone(&managed_pub);
        let counter = Arc::clone(&counter);
        async move {
            interval
                .run(move || {
                    let managed_pub = Arc::clone(&managed_pub);
                    let counter = Arc::clone(&counter);
                    let publish_topic = publish_topic_for_log.clone();
                    async move {
                        let count = counter.fetch_add(1, Ordering::Relaxed) + 1;
                        let msg = RosString { data: format!("tick {count}") };
                        match managed_pub.publish(&msg).await {
                            Ok(true) => println!("[publish] {count} -> {publish_topic}"),
                            Ok(false) => println!("[publish] suppressed (inactive)"),
                            Err(err) => eprintln!("[publish] error: {err}"),
                        }
                    }
                })
                .await;
        }
    });

    println!("rosrustext lifecycle demo node running");
    println!("  node: {}", config.node_name);
    println!("  bridge: {}", config.bridge_url);
    println!("  backend namespace: /{}/_rosrustext/", config.node_name);
    println!("  publish topic: {}", publish_topic);

    tokio::signal::ctrl_c().await?;
    let (ok, message) = service.shutdown_best_effort();
    println!("shutdown: {} ({})", ok, message);
    Ok(())
}
