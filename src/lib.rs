use std::fs::File;
use std::io::{BufRead, BufReader};
use std::mem::swap;

#[derive(Debug, PartialEq, Clone, Copy)]
enum Axis {
    X,
    Y,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle {
    x1: f64,
    y1: f64,
    z1: f64,
    x2: f64,
    y2: f64,
    z2: f64,
    x3: f64,
    y3: f64,
    z3: f64,
    // n: Vertex,
}

pub struct Mesh {
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

impl Vertex {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    fn magnitude(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    fn normalize(self) -> Self {
        let len = self.magnitude();
        Self {
            x: self.x / len,
            y: self.y / len,
            z: self.z / len,
        }
    }
}

impl Node {
    #[inline(always)]
    fn point_in(&self, point: Vertex) -> bool {
        if self.axis == Axis::X {
            return !(point.x < self.min as f64 || point.x > self.max as f64);
        } else {
            return !(point.y < self.min as f64 || point.y > self.max as f64);
        };
    }
}

impl Triangle {
    fn new(v1: Vertex, v2: Vertex, v3: Vertex) -> Self {
        let determinant = -v2.y * v3.x + v1.y * (v3.x - v2.x) + v1.x * (v2.y - v3.y) + v2.x * v3.y;

        let mut v1 = v1;
        let mut v2 = v2;

        if determinant > 0.0 {
            swap(&mut v1, &mut v2);
        };

        let tri = Self {
            x1: v1.x,
            y1: v1.y,
            z1: v1.z,
            x2: v2.x,
            y2: v2.y,
            z2: v2.z,
            x3: v3.x,
            y3: v3.y,
            z3: v3.z,
            // n: Vertex::default(),
        };

        // tri.n = tri.calc_normal();
        tri
    }

    pub fn centroid(&self) -> Vertex {
        Vertex {
            x: self.centroid_x(),
            y: self.centroid_y(),
            z: self.centroid_z(),
        }
    }

    pub fn vertex(&self, index: u8) -> Vertex {
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

    pub fn calc_normal(&self) -> Vertex {
        let [(ax, ay, az), (bx, by, bz), (cx, cy, cz)] = [
            (self.x1, self.y1, self.z1),
            (self.x2, self.y2, self.z2),
            (self.x3, self.y3, self.z3),
        ];

        let abx = bx - ax;
        let aby = by - ay;
        let abz = bz - az;

        let acx = cx - ax;
        let acy = cy - ay;
        let acz = cz - az;

        let normal_x = aby * acz - abz * acy;
        let normal_y = abz * acx - abx * acz;
        let normal_z = abx * acy - aby * acx;

        Vertex {
            x: normal_x,
            y: normal_y,
            z: normal_z,
        }
        .normalize()
    }

    fn get_z(&self, point: Vertex) -> f64 {
        const EPS: f64 = 1.0e-9;
        let (px, py) = (point.x, point.y);
        let [(ax, ay, az), (bx, by, bz), (cx, cy, cz)] = [
            (self.x1, self.y1, self.z1),
            (self.x2, self.y2, self.z2),
            (self.x3, self.y3, self.z3),
        ];

        let abx = bx - ax;
        let aby = by - ay;
        let abz = bz - az;

        let acx = cx - ax;
        let acy = cy - ay;
        let acz = cz - az;

        let bcx = cx - bx;
        let bcy = cy - by;

        let d1 = aby * (px - ax) - abx * (py - ay);
        let d2 = bcy * (px - bx) - bcx * (py - by);
        let d3 = acx * (py - cy) - acy * (px - cx);

        // pre-calculate and store the normal and d value as part of the triangle
        let normal_x = aby * acz - abz * acy;
        let normal_y = abz * acx - abx * acz;
        let normal_z = abx * acy - aby * acx;

        let d = normal_x * ax + normal_y * ay + normal_z * az;

        let hit: bool = d1 > -EPS && d2 > -EPS && d3 > -EPS;
        if hit {
            (d - normal_x * px - normal_y * py) / normal_z
        } else {
            f64::NEG_INFINITY
        }
    }
}

impl Mesh {
    pub fn new(file_path: &str, y_up: bool) -> Mesh {
        let mut triangles = Mesh::parse_obj(&file_path, y_up);
        Self::build(&mut triangles)
    }

    pub fn get_len(&self) -> usize {
        return self.triangles.len();
    }

    pub fn get_triangles(&self) -> &Vec<Triangle> {
        &self.triangles
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
                        let tri = Triangle::new(
                            vertices[vertex_indices[0]],
                            vertices[vertex_indices[i - 1]],
                            vertices[vertex_indices[i]],
                        );

                        triangles.push(tri);
                    }
                }
                _ => (),
            }
        }
        triangles
    }

    fn build(triangles: &mut Vec<Triangle>) -> Mesh {
        let mut nodes: Vec<Node> = vec![];
        let start_index = 0;
        let end_index = triangles.len();

        // Determine triangle bounds and splitting axis
        let (axis, _, _, _, _) = Mesh::triangle_bounds(triangles, start_index, end_index);

        // Sort triangles along the splitting axis
        Mesh::sort_triangles(triangles, axis, start_index, end_index);

        // Generate the root node
        nodes.push(Mesh::build_node(triangles, axis, start_index, end_index));

        // Recursively build the node tree, by constantly splitting the nodes
        let mut node_idx = 0;
        while node_idx < nodes.len() {
            Mesh::split_node(triangles, &mut nodes, node_idx);
            node_idx += 1;
        }

        let mesh = Mesh {
            nodes,
            triangles: triangles.to_vec(),
        };

        return mesh;
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

        // boundary padding
        const EPS: f64 = 1.0e-9;

        Node {
            axis,
            min: (min - EPS) as f64,
            max: (max + EPS) as f64,
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

    pub fn traverse(&self, point: Vertex) -> f64 {
        self.traverse_clamped(point, f64::INFINITY)
    }

    pub fn traverse_clamped(&self, point: Vertex, max_height: f64) -> f64 {
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
                /*
                height = self.triangles[node.start_index..node.end_index]
                    .iter()
                    .fold(height, |h, triangle| triangle.get_z(point).max(h));
                */
                for tri in self.triangles[node.start_index..node.end_index].iter() {
                    let z_test = tri.get_z(point);
                    if z_test > height && z_test <= max_height {
                        height = z_test;
                    }
                }
            }
        }

        height
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_vert() {
        assert_eq!(
            Vertex::new(1.0, 2.0, 3.0),
            Vertex {
                x: 1.0,
                y: 2.0,
                z: 3.0
            }
        );
        assert_eq!(
            Vertex::default(),
            Vertex {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn test_normalize() {
        assert_eq!(Vertex::new(1.0, 2.0, 3.0).magnitude(), 3.7416573867739413);

        assert_eq!(
            Vertex::new(1.0, 2.0, 3.0).normalize(),
            Vertex {
                x: 0.2672612419124244,
                y: 0.5345224838248488,
                z: 0.8017837257372732
            }
        );
    }

    #[test]
    fn test_node_bounds() {
        let node = Node {
            axis: Axis::X,
            min: -1.0,
            max: 1.0,
            child_index: 0,
            start_index: 0,
            end_index: 1,
        };
        assert!(node.point_in(Vertex::new(0.0, 0.0, 0.0)));
        assert!(node.point_in(Vertex::new(-1.0, -1.0, 0.0)));
        assert!(node.point_in(Vertex::new(1.0, 1.0, 0.0)));
        assert!(!node.point_in(Vertex::new(1.00001, 1.00001, 0.0)));
        assert!(!node.point_in(Vertex::new(-1.00001, 1.00001, 0.0)));

        let node = Node {
            axis: Axis::Y,
            min: -1.0,
            max: 1.0,
            child_index: 0,
            start_index: 0,
            end_index: 1,
        };
        assert!(node.point_in(Vertex::new(-1.0, 1.0, 0.0)));
        assert!(node.point_in(Vertex::new(1.0, -1.0, 0.0)));
        assert!(!node.point_in(Vertex::new(-1.00001, 1.00001, 0.0)));
        assert!(!node.point_in(Vertex::new(1.00001, -1.00001, 0.0)));
    }

    #[test]
    fn triangle_math() {
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

        let triangle: Triangle = Triangle::new(v1, v2, v3);

        // test out of range to see if default case is returned
        assert_eq!(triangle.vertex(4), triangle.vertex(2));

        // test centroid sum values (used for node sorting)
        assert_eq!(triangle.centroid_sum_x(), v1.x + v2.x + v3.x);
        assert_eq!(triangle.centroid_sum_y(), v1.y + v2.y + v3.y);

        // test triangle vert min / max values (used for node sorting)
        assert_eq!(triangle.min_x(), v1.x.min(v2.x).min(v3.x));
        assert_eq!(triangle.min_y(), v1.y.min(v2.y).min(v3.y));
        assert_eq!(triangle.max_x(), v1.x.max(v2.x).max(v3.x));
        assert_eq!(triangle.max_y(), v1.y.max(v2.y).max(v3.y));
    }

    #[test]
    fn triangle_normal() {
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

        let triangle: Triangle = Triangle::new(v1, v2, v3);

        assert_eq!(
            triangle.calc_normal(),
            Vertex {
                x: 0.4472135954999579,
                y: 0.0,
                z: -0.8944271909999159
            }
        );
    }

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

        let triangle = Triangle::new(v1, v2, v3);

        // Evaluate centroid
        let p = triangle.centroid();
        let z = triangle.get_z(p);
        assert!(f64::abs(p.z - z) < eps);

        // Evaluate each corner
        for i in 0..3 {
            let p = triangle.vertex(i);
            let z = triangle.get_z(p);
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
            let z = triangle.get_z(p);
            assert!(f64::abs(p.z - z) < eps);
        }
    }

    #[test]
    fn test_triangle_bounds() {
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

        let triangle: Triangle = Triangle::new(v1, v2, v3);

        let triangles = vec![triangle];
        let (axis, x_min, x_max, y_min, y_max) = Mesh::triangle_bounds(&triangles, 0, 1);

        assert_eq!(axis, Axis::X);
        assert_eq!(x_min, -1.0);
        assert_eq!(x_max, 1.0);
        assert_eq!(y_min, 0.0);
        assert_eq!(y_max, 1.0);

        let v1 = Vertex {
            x: 0.0,
            y: -1.0,
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

        let triangle: Triangle = Triangle::new(v1, v2, v3);

        let triangles = vec![triangle];
        let (axis, x_min, x_max, y_min, y_max) = Mesh::triangle_bounds(&triangles, 0, 1);

        assert_eq!(axis, Axis::Y);
        assert_eq!(x_min, 0.0);
        assert_eq!(x_max, 1.0);
        assert_eq!(y_min, -1.0);
        assert_eq!(y_max, 1.0);
    }

    #[test]
    fn test_triangle_sort() {
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

        let t1: Triangle = Triangle::new(v1, v2, v3);

        let v1 = Vertex {
            x: 0.0,
            y: -1.0,
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

        let t2: Triangle = Triangle::new(v1, v2, v3);

        let mut triangles = vec![t1, t2];
        assert_eq!(triangles[0], t1);
        assert_eq!(triangles[1], t2);

        Mesh::sort_triangles(&mut triangles, Axis::Y, 0, 2);
        assert_eq!(triangles[1], t1);
        assert_eq!(triangles[0], t2);

        Mesh::sort_triangles(&mut triangles, Axis::X, 0, 2);
        assert_eq!(triangles[0], t1);
        assert_eq!(triangles[1], t2);
    }

    #[test]
    fn test_build_node() {
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

        let t1: Triangle = Triangle::new(v1, v2, v3);

        let v1 = Vertex {
            x: 0.0,
            y: -1.0,
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

        let t2: Triangle = Triangle::new(v1, v2, v3);
        let mut triangles = vec![t1, t2];

        let (_, x_min, x_max, y_min, y_max) = Mesh::triangle_bounds(&triangles, 0, 2);

        let node = Mesh::build_node(&mut triangles, Axis::X, 0, 2);

        // bounds checks
        assert!(node.max >= x_max);
        assert!(node.min <= x_min);

        assert_eq!(node.axis, Axis::X);

        assert_eq!(node.child_index, 0);
        assert_eq!(node.start_index, 0);
        assert_eq!(node.end_index, 2);

        let node = Mesh::build_node(&mut triangles, Axis::Y, 0, 2);

        // bounds checks
        assert!(node.max >= y_max);
        assert!(node.min <= y_min);

        // println!("{:?}", node);
    }

    #[test]
    fn obj_loader() {
        let file_path = "./tests/sample.obj";
        let y_up = false;

        let tris = Mesh::parse_obj(file_path, y_up);
        for t in tris.iter() {
            println!("{:?}", t);
        }

        // expect 12 faces
        assert_eq!(tris.len(), 12);

        assert_eq!(
            *tris.first().unwrap(),
            Triangle {
                x1: 1.0,
                y1: -1.0,
                z1: -1.0,
                x2: 1.0,
                y2: -1.0,
                z2: 1.0,
                x3: -1.0,
                y3: -1.0,
                z3: 1.0
            }
        );
        assert_eq!(
            *tris.last().unwrap(),
            Triangle {
                x1: 1.0,
                y1: 1.0,
                z1: -1.0,
                x2: -1.0,
                y2: -1.0,
                z2: -1.0,
                x3: -1.0,
                y3: 1.0,
                z3: -1.0
            }
        );

        // this is just to bump the test coverage
        let y_up = true;

        let tris = Mesh::parse_obj(file_path, y_up);
        for t in tris.iter() {
            println!("{:?}", t);
        }

        // expect 12 faces
        assert_eq!(tris.len(), 12);

        assert_eq!(
            *tris.first().unwrap(),
            Triangle {
                x1: 1.0,
                y1: 1.0,
                z1: -1.0,
                x2: 1.0,
                y2: -1.0,
                z2: -1.0,
                x3: -1.0,
                y3: 1.0,
                z3: -1.0
            }
        );
    }

    fn dummy_mesh(n: usize, m: usize, amplitude: f64, frequency: f64) -> Vec<Triangle> {
        let mut triangles = Vec::new();
        let mut vertices = Vec::new();

        // Generate vertices with sine wave manipulation
        for i in 0..n {
            for j in 0..m {
                let x = j as f64;
                let y = i as f64;
                let z = amplitude * ((frequency * x).sin() + (frequency * y).cos()); // Manipulate z-coordinate with sine function

                vertices.push(Vertex::new(x, y, z));
            }
        }
        for i in 0..n {
            for j in 0..m {
                // Create two triangles for each grid cell
                if i < n - 1 && j < m - 1 {
                    let top_left = i * m + j;
                    let top_right = i * m + j + 1;
                    let bottom_left = (i + 1) * m + j;
                    let bottom_right = (i + 1) * m + j + 1;

                    // First triangle (top-left, bottom-left, top-right)
                    triangles.push(Triangle::new(
                        vertices[top_left],
                        vertices[bottom_left],
                        vertices[top_right],
                    ));

                    // Second triangle (top-right, bottom-left, bottom-right)
                    triangles.push(Triangle::new(
                        vertices[top_right],
                        vertices[bottom_left],
                        vertices[bottom_right],
                    ));
                }
            }
        }

        triangles
    }

    #[test]
    fn dummy_mesh_traversal() {
        let mut triangles = dummy_mesh(10, 20, 0.1, 0.5);
        let mesh = Mesh::build(&mut triangles);

        for t in triangles.iter() {
            let point = t.centroid();
            let z = mesh.traverse(point);
            let dz = point.z - z;

            let eps = 1.0e-9;
            assert!(dz.abs() < eps);
        }

        for t in triangles.iter() {
            for i in 0..3
            {
                let point = t.vertex(i);
                let z = mesh.traverse(point);
                let dz = point.z - z;

                let eps = 1.0e-9;
                assert!(dz.abs() < eps);
            }
        }
    }

    #[test]
    fn obj_mesh_load() {
        let file_path = "./tests/sample.obj";
        let y_up = false;

        let mesh = Mesh::new(file_path, y_up);

        assert_eq!(mesh.get_len(), 12);
        assert_eq!(mesh.get_triangles().len(), 12);
    }
}
