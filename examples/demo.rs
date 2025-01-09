use std::time::Instant;
use terrain::Mesh;

fn main() {
    let file_path = "./data/paul_ricard_colidable.obj";
    let y_up = true;

    // let file_path = "./data/le_mans_collidable.obj";
    // let y_up = false;

    let start_time = Instant::now();

    let mesh = Mesh::new(file_path, y_up);
    let duration = start_time.elapsed();
    println!("Loaded mesh in: {:?} [s]", duration);

    let start_time = Instant::now();

    let mut match_count = 0;
    let mut test_count = 0;

    let eps = 1.0e-9;

    // let upvec = if y_up {
    //     Vertex::new(0.0, 1.0, 0.0)
    // } else {
    //     Vertex::new(0.0, 0.0, 1.0)
    // };

    // Evaluate the x/y coordinate of each triangle centroid
    // and compare the resulting z coordinate
    mesh.get_triangles().iter().for_each(|tri| {
        let p = tri.centroid();
        // let z = mesh.traverse(p);
        let z = mesh.traverse_clamped(p, p.z + 0.5);

        test_count += 1;
        if f64::abs(p.z - z) < eps {
            match_count += 1;
        } else {
            // println!("#{:?} {:?} {:?}", t, p, z);
        }
    });

    let duration = start_time.elapsed();
    println!(
        "Completed mesh eval: {:?} [s] - {:?} [1/ms] - {:?} [%]",
        duration,
        (test_count as f64) / duration.as_secs_f64() / 1000.0,
        (match_count as f64) / (test_count as f64) * 100.0
    );

    println!("Finished!");
}
