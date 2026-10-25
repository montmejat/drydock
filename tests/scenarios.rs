//! The example scenarios read, against the robots they're for.

use drydock::robot::Robot;
use drydock::scenario::Scenario;

#[test]
fn the_example_scenarios_read() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/examples");
    let mut read = 0;
    for robot in ["arm", "openarm"] {
        let path = format!("{dir}/{robot}");
        let joints: Vec<String> =
            Robot::load(format!("{path}/robot.yml")).unwrap().motors().map(|(_, m)| m.joint.clone()).collect();
        for entry in std::fs::read_dir(&path).unwrap() {
            let file = entry.unwrap().path();
            if file.to_string_lossy().ends_with(".scenario.yml") {
                Scenario::load(&file, &joints).unwrap_or_else(|e| panic!("{e}"));
                read += 1;
            }
        }
    }
    assert!(read >= 2, "found {read}");
}
