use super::*;
use delta::Encoder;
use dsg_convert::compare::compare;

fn object(id: u64, label: u64, x: f64) -> Value {
    json!({"id": (b'O' as u64) << 56 | id, "layer": 2, "partition": 0,
           "attributes": {"position": [x, 0.0, 0.0], "semantic_label": label}})
}

fn place(id: u64, x: f64) -> Value {
    json!({"id": (b'p' as u64) << 56 | id, "layer": 3, "partition": 0,
           "attributes": {"position": [x, 0.0, 0.0]}})
}

fn check(station: &Station) {
    let (want, merges) = station.merged_graph();
    assert_eq!(station.merges, merges);
    let d = compare(&want, &station.export(), 0.0);
    assert!(d.is_equal(), "{d:?}");
    assert_eq!(station.topology.num_edges(), station.num_edges());
}

#[test]
fn merges_the_same_object_seen_by_two_robots() {
    let p = |i: u64| (b'p' as u64) << 56 | i;
    let o = |i: u64| (b'O' as u64) << 56 | i;
    // robot 0: chair at x=1 under place 1; robot 1 sees it at x=1.2 in its own frame,
    // which is shifted by +5 in x, plus a table only it sees.
    let a = json!({"nodes": [object(1, 7, 1.0), place(1, 1.0)],
                   "edges": [{"source": p(1), "target": o(1), "info": {}}]});
    let b = json!({"nodes": [object(1, 7, -3.8), object(2, 9, -3.0), place(1, -3.8)],
                   "edges": [{"source": p(1), "target": o(1), "info": {}},
                             {"source": p(1), "target": o(2), "info": {}}]});
    let mut station = Station::new(Config::objects_only());
    station.add_robot(0, Transform::default());
    station.add_robot(
        1,
        Transform {
            yaw: 0.0,
            translation: [5.0, 0.0, 0.0],
        },
    );
    let (mut ea, mut eb) = (Encoder::default(), Encoder::default());
    station.receive(0, &ea.encode(&a)).unwrap();
    station.receive(1, &eb.encode(&b)).unwrap();
    check(&station);

    // robot 1's chair merged into robot 0's; its place now links to robot 0's chair
    let chair0 = global_id(0, o(1));
    assert_eq!(
        station.merges,
        BTreeMap::from([(global_id(1, o(1)), chair0)])
    );
    let g = station.export();
    assert_eq!(g["nodes"].as_array().unwrap().len(), 4);
    assert!(station.topology.has_edge(global_id(1, p(1)), chair0));
    assert_eq!(station.num_edges(), 3);

    // robot 0 loses its chair: robot 1's comes back, with its own edge
    let a2 = json!({"nodes": [place(1, 1.0)], "edges": []});
    station.receive(0, &ea.encode(&a2)).unwrap();
    check(&station);
    assert!(station.merges.is_empty());
    assert!(
        station
            .topology
            .has_edge(global_id(1, p(1)), global_id(1, o(1)))
    );

    // drift correction keeps the incremental graph equal to the reference
    station.correct_drift(&[OBJECTS, PLACES, ROOMS]);
    check(&station);

    // robot 1 drifts away: the chair seen again by robot 0 stays separate
    station.receive(0, &ea.encode(&a)).unwrap();
    station.set_transform(
        1,
        Transform {
            yaw: 0.0,
            translation: [9.0, 0.0, 0.0],
        },
    );
    check(&station);
    assert!(station.merges.is_empty());
}
