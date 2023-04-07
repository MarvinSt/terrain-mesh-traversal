use std::fs::File;
use std::io::{BufRead, BufReader};
use std::time::Instant;

#[derive(Debug, PartialEq, Clone, Copy)]
enum Axis {
    X,
    Y,
}

#[derive(Debug)]
struct Vertex {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Clone, Copy)]
struct Triangle {
    x1: f64,
    y1: f64,
    z1: f64,
    x2: f64,
    y2: f64,
    z2: f64,
    x3: f64,
    y3: f64,
    z3: f64,
}

struct Mesh {
    nodes: Vec<Node>,
    triangles: Vec<Triangle>,
}

#[derive(Debug, Clone)]
struct Node {
    axis: Axis,
    min: f64,
    max: f64,
    child_index: usize,
    start_index: usize,
    end_index: usize,
}

impl Node {
    fn point_in(&self, point: &Vertex) -> bool {
        if self.axis == Axis::X {
            return !(point.x < self.min || point.x > self.max);
        } else {
            return !(point.y < self.min || point.y > self.max);
        };
    }
}

impl Triangle {
    fn centroid(&self) -> Vertex {
        Vertex {
            x: self.centroid_x(),
            y: self.centroid_y(),
            z: self.centroid_z(),
        }
    }

    fn vertex(&self, index: u8) -> Vertex {
        match index {
            0 => Vertex {
                x: self.x1,
                y: self.y1,
                z: self.z1,
            },
            1 => Vertex {
                x: self.x2,
                y: self.y2,
                z: self.z2,
            },
            2 => Vertex {
                x: self.x3,
                y: self.y3,
                z: self.z3,
            },
            _ => Vertex {
                x: self.x3,
                y: self.y3,
                z: self.z3,
            },
        }
    }

    fn centroid_x(&self) -> f64 {
        (self.x1 + self.x2 + self.x3) / 3.0
    }

    fn centroid_y(&self) -> f64 {
        (self.y1 + self.y2 + self.y3) / 3.0
    }

    fn centroid_z(&self) -> f64 {
        (self.z1 + self.z2 + self.z3) / 3.0
    }

    fn centroid_sum_x(&self) -> f64 {
        self.x1 + self.x2 + self.x3
    }

    fn centroid_sum_y(&self) -> f64 {
        self.y1 + self.y2 + self.y3
    }

    fn min_x(&self) -> f64 {
        self.x1.min(self.x2).min(self.x3)
    }

    fn min_y(&self) -> f64 {
        self.y1.min(self.y2).min(self.y3)
    }

    fn max_x(&self) -> f64 {
        self.x1.max(self.x2).max(self.x3)
    }

    fn max_y(&self) -> f64 {
        self.y1.max(self.y2).max(self.y3)
    }

    fn get_z(&self, point: &Vertex) -> f64 {
        let eps = 1.0e-9;
        let (px, py) = (point.x, point.y);
        let [(ax, ay, az), (bx, by, bz), (cx, cy, cz)] = [
            (self.x1, self.y1, self.z1),
            (self.x2, self.y2, self.z2),
            (self.x3, self.y3, self.z3),
        ];

        let d1 = (px - ax) * (by - ay) - (bx - ax) * (py - ay);
        let d2 = (px - bx) * (cy - by) - (cx - bx) * (py - by);
        let d3 = (px - cx) * (ay - cy) - (ax - cx) * (py - cy);

        let hit = d1 < eps && d2 < eps && d3 < eps || d1 > eps && d2 > eps && d3 > eps;

        if !hit {
            return f64::NEG_INFINITY;
        }

        let abx = bx - ax;
        let aby = by - ay;
        let abz = bz - az;
        let acx = cx - ax;
        let acy = cy - ay;
        let acz = cz - az;

        let normal_x = aby * acz - abz * acy;
        let normal_y = abz * acx - abx * acz;
        let normal_z = abx * acy - aby * acx;

        let d = -(normal_x * ax + normal_y * ay + normal_z * az);

        let z = -(normal_x * px + normal_y * py + d) / normal_z;

        z
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_in_triangle() {
        let eps = 1.0e-9;

        let v1 = Vertex {
            x: -1.0,
            y: 0.0,
            z: 0.5,
        };
        let v2 = Vertex {
            x: 1.0,
            y: 0.0,
            z: 1.5,
        };
        let v3 = Vertex {
            x: 0.0,
            y: 1.0,
            z: 1.0,
        };

        let triangle = Triangle {
            x1: v1.x,
            y1: v1.y,
            z1: v1.z,
            x2: v2.x,
            y2: v2.y,
            z2: v2.z,
            x3: v3.x,
            y3: v3.y,
            z3: v3.z,
        };

        // Evaluate centroid
        let p = triangle.centroid();
        let z = triangle.get_z(&p);
        assert!(f64::abs(p.z - z) < eps);

        // Evaluate each corner
        for i in 0..3 {
            let p = triangle.vertex(i);
            let z = triangle.get_z(&p);
            assert!(f64::abs(p.z - z) < eps);
        }

        // Evaluate each base
        for i in 0..3 {
            let va = triangle.vertex(i);
            let vb = triangle.vertex((i + 1) % 3);
            let p = Vertex {
                x: 0.5 * (va.x + vb.x),
                y: 0.5 * (va.y + vb.y),
                z: 0.5 * (va.z + vb.z),
            };
            let z = triangle.get_z(&p);
            assert!(f64::abs(p.z - z) < eps);
        }
    }
}

impl Mesh {
    fn new(file_path: &str, y_up: bool) -> Mesh {
        let mut nodes: Vec<Node> = vec![];
        let mut triangles = Mesh::parse_obj(&file_path, y_up);

        let start_index = 0;
        let end_index = triangles.len();

        // Determine triangle bounds and splitting axis
        let (axis, _, _, _, _) = Mesh::triangle_bounds(&mut triangles, start_index, end_index);

        // Sort triangles along the splitting axis
        Mesh::sort_triangles(&mut triangles, axis, start_index, end_index);

        // Generate the root node
        nodes.push(Mesh::build_node(
            &mut triangles,
            axis,
            start_index,
            end_index,
        ));

        // Recursively build the node tree, by constantly splitting the nodes
        let mut node_idx = 0;
        while node_idx < nodes.len() {
            Mesh::split_node(&mut triangles, &mut nodes, node_idx);
            node_idx += 1;
        }

        let mesh = Mesh { nodes, triangles };

        return mesh;
    }

    fn get_len(&self) -> usize {
        return self.triangles.len();
    }

    fn build_node(
        triangles: &mut Vec<Triangle>,
        axis: Axis,
        start_index: usize,
        end_index: usize,
    ) -> Node {
        // Calculate the triangle bounds for the right node and push to the nodelist
        let (_, x_min, x_max, y_min, y_max) =
            Mesh::triangle_bounds(triangles, start_index, end_index);

        let (min, max) = if axis == Axis::X {
            (x_min, x_max)
        } else {
            (y_min, y_max)
        };

        Node {
            axis,
            min,
            max,
            start_index,
            end_index,
            child_index: 0,
        }
    }

    fn split_node(triangles: &mut Vec<Triangle>, nodes: &mut Vec<Node>, node_idx: usize) -> bool {
        let leafsize = 16;

        let node = nodes[node_idx].clone();

        if node.end_index - node.start_index < leafsize {
            return true;
        }

        let mid = (node.start_index + node.end_index) / 2;

        // Calculate the child index of the current node
        // This is the index of the left node; the right node is the next index
        let child_index = nodes.len();

        // Determine triangle bounds and splitting axis
        let (axis, _, _, _, _) = Mesh::triangle_bounds(triangles, node.start_index, node.end_index);

        // Sort triangles along the splitting axis
        Mesh::sort_triangles(triangles, axis, node.start_index, node.end_index);

        // Build and push the left side node
        nodes.push(Mesh::build_node(triangles, axis, node.start_index, mid));

        // Build and push the right side node
        nodes.push(Mesh::build_node(triangles, axis, mid, node.end_index));

        // Set the current node child index
        nodes[node_idx].child_index = child_index;

        false
    }

    fn triangle_bounds(
        triangles: &Vec<Triangle>,
        start_index: usize,
        end_index: usize,
    ) -> (Axis, f64, f64, f64, f64) {
        let (mut x_min, mut x_max) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut y_min, mut y_max) = (f64::INFINITY, f64::NEG_INFINITY);

        for triangle in triangles[start_index..end_index].iter() {
            x_min = x_min.min(triangle.min_x());
            x_max = x_max.max(triangle.max_x());
            y_min = y_min.min(triangle.min_y());
            y_max = y_max.max(triangle.max_y());
        }

        if x_max - x_min > y_max - y_min {
            (Axis::X, x_min, x_max, y_min, y_max)
        } else {
            (Axis::Y, x_min, x_max, y_min, y_max)
        }
    }

    fn sort_triangles(
        triangles: &mut Vec<Triangle>,
        axis: Axis,
        start_index: usize,
        end_index: usize,
    ) {
        triangles[start_index..end_index].sort_by(|a, b| {
            if axis == Axis::X {
                return a.centroid_sum_x().partial_cmp(&b.centroid_sum_x()).unwrap();
            } else {
                return a.centroid_sum_y().partial_cmp(&b.centroid_sum_y()).unwrap();
            };
        });
    }

    fn parse_obj(file_path: &str, y_up: bool) -> Vec<Triangle> {
        let file = File::open(file_path).unwrap();
        let reader = BufReader::new(file);

        let mut vertices = vec![];
        let mut triangles = vec![];

        for line in reader.lines() {
            let line = line.unwrap();
            let parts: Vec<&str> = line.split_whitespace().collect();

            if parts.len() == 0 {
                continue;
            }

            match parts[0] {
                "v" => {
                    let x = parts[1].parse::<f64>().unwrap();
                    let y = parts[2].parse::<f64>().unwrap();
                    let z = parts[3].parse::<f64>().unwrap();

                    if y_up {
                        vertices.push(Vertex { x, y: z, z: y });
                    } else {
                        vertices.push(Vertex { x, y, z });
                    }
                }
                "f" => {
                    let vertex_count = parts.len() - 1;
                    if vertex_count < 3 {
                        continue;
                    }

                    let mut vertex_indices = vec![];

                    for i in 1..vertex_count + 1 {
                        let vertex_index = parts[i]
                            .split('/')
                            .next()
                            .unwrap()
                            .parse::<usize>()
                            .unwrap();
                        vertex_indices.push(vertex_index - 1);
                    }

                    for i in 2..vertex_count {
                        let tri = Triangle {
                            x1: vertices[vertex_indices[0]].x,
                            y1: vertices[vertex_indices[0]].y,
                            z1: vertices[vertex_indices[0]].z,
                            x2: vertices[vertex_indices[i - 1]].x,
                            y2: vertices[vertex_indices[i - 1]].y,
                            z2: vertices[vertex_indices[i - 1]].z,
                            x3: vertices[vertex_indices[i]].x,
                            y3: vertices[vertex_indices[i]].y,
                            z3: vertices[vertex_indices[i]].z,
                        };

                        triangles.push(tri);
                    }
                }
                _ => (),
            }
        }
        triangles
    }

    fn traverse(&self, point: &Vertex) -> f64 {
        let mut height = f64::NEG_INFINITY;

        let mut stack: [usize; 64] = [0; 64];
        let mut size: usize = 1;

        while size > 0 {
            size -= 1;
            let node = &self.nodes[stack[size]];

            if node.child_index > 0 {
                if self.nodes[node.child_index + 0].point_in(point) {
                    stack[size] = node.child_index + 0;
                    size += 1;
                }
                if self.nodes[node.child_index + 1].point_in(point) {
                    stack[size] = node.child_index + 1;
                    size += 1;
                }
            } else {
                // Calculate new maximum height
                height = self.triangles[node.start_index..node.end_index]
                    .iter()
                    .fold(height, |h, triangle| triangle.get_z(&point).max(h));

                /*
                for triangle in self.triangles[node.start_index..node.end_index].iter() {
                    height = height.max(triangle.get_z(&point));
                }
                */
            }
        }

        height
    }
}

fn main() {
    let file_path = "./data/paul_ricard_colidable.obj";
    let y_up = true;

    let start_time = Instant::now();
    let mesh = Mesh::new(file_path, y_up);
    let duration = start_time.elapsed();
    println!("Loaded mesh in: {:?} [s]", duration);

    let tlen = mesh.get_len();

    let start_time = Instant::now();

    let mut match_count = 0;

    let eps = 1.0e-9;

    let mut p = Vertex {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    p.z = mesh.traverse(&p);

    // Evaluate the x/y coordinate of each triangle centroid
    // and compare the resulting z coordinate
    mesh.triangles.iter().for_each(|tri| {
        let p = tri.centroid();
        let z = mesh.traverse(&p);

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
        (tlen as f64) / duration.as_secs_f64() / 1000.0,
        (match_count as f64) / (tlen as f64) * 100.0
    );

    println!("Finished!");
}
