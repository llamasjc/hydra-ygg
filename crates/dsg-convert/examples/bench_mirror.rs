use std::time::Instant;
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let text = std::fs::read_to_string(path).unwrap();
    let t = Instant::now();
    let v: serde_json::Value = serde_json::from_str(&text).unwrap();
    println!("parse {:?} ({} nodes)", t.elapsed(), v["nodes"].as_array().unwrap().len());
    let mut m = dsg_convert::incremental::Mirror::default();
    let t = Instant::now(); m.apply(&v); println!("first apply {:?}", t.elapsed());
    let t = Instant::now(); m.apply(&v); println!("unchanged apply {:?}", t.elapsed());
    let t = Instant::now(); let e = m.export(); println!("export {:?}", t.elapsed());
    let t = Instant::now(); let s = e.to_string(); println!("to_string {:?} {}B", t.elapsed(), s.len());
}
