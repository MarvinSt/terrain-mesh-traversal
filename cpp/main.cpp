#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>

#define FLOAT_TYPE double

struct Hit
{
    FLOAT_TYPE height = std::numeric_limits<FLOAT_TYPE>::quiet_NaN();
    bool is_hit = false;
};

class Vertex
{
public:
    FLOAT_TYPE x, y, z;

    Vertex(FLOAT_TYPE px, FLOAT_TYPE py, FLOAT_TYPE pz)
    {
        x = px;
        y = py;
        z = pz;
    };
};

class AABB
{
public:
    FLOAT_TYPE x_min, x_max;
    FLOAT_TYPE y_min, y_max;

    AABB(Vertex v1, Vertex v2, Vertex v3)
    {
        /*
        x_min = v1.x < v2.x ? (v1.x < v3.x ? v1.x : v3.x) : (v2.x < v3.x ? v2.x : v3.x);
        y_min = v1.y < v2.y ? (v1.y < v3.y ? v1.y : v3.y) : (v2.y < v3.y ? v2.y : v3.y);
        x_max = v1.x > v2.x ? (v1.x > v3.x ? v1.x : v3.x) : (v2.x > v3.x ? v2.x : v3.x);
        y_max = v1.y > v2.y ? (v1.y > v3.y ? v1.y : v3.y) : (v2.y > v3.y ? v2.y : v3.y);
        */

        x_min = fmin(fmin(v1.x, v2.x), v3.x);
        y_min = fmin(fmin(v1.y, v2.y), v3.y);
        x_max = fmax(fmax(v1.x, v2.x), v3.x);
        y_max = fmax(fmax(v1.y, v2.y), v3.y);
    };

    AABB(AABB a, AABB b)
    {
        /*
        x_min = a.x_min < b.x_min ? a.x_min : b.x_min;
        y_min = a.y_min < b.y_min ? a.y_min : b.y_min;
        x_max = a.x_max > b.x_max ? a.x_max : b.x_max;
        y_max = a.y_max > b.y_max ? a.y_max : b.y_max;
        */

        x_min = fmin(a.x_min, b.x_min);
        y_min = fmin(a.y_min, b.y_min);
        x_max = fmax(a.x_max, b.x_max);
        y_max = fmax(a.y_max, b.y_max);
    }

    uint8_t GetSplitAxis()
    {
        return x_max - x_min > y_max - y_min ? 0 : 1;
    }
};

class Triangle
{
public:
    size_t v1, v2, v3;

    Triangle(size_t ver1, size_t ver2, size_t ver3)
    {
        v1 = ver1;
        v2 = ver2;
        v3 = ver3;
    };

    AABB GetAABB(std::vector<Vertex> &vertices)
    {
        return AABB(vertices[v1], vertices[v2], vertices[v3]);
    };

    Vertex GetCentroid(std::vector<Vertex> &vertices)
    {
        return Vertex(
            (vertices[v1].x + vertices[v2].x + vertices[v3].x) / 3.0,
            (vertices[v1].y + vertices[v2].y + vertices[v3].y) / 3.0,
            (vertices[v1].z + vertices[v2].z + vertices[v3].z) / 3.0);
    };

    FLOAT_TYPE GetSortValue(std::vector<Vertex> &vertices, uint8_t axis)
    {
        return axis == 0 ? vertices[v1].x + vertices[v2].x + vertices[v3].x : vertices[v1].y + vertices[v2].y + vertices[v3].y;
    }

    inline FLOAT_TYPE PointInTriangleHeightNan(std::vector<Vertex> &vertices, Vertex p)
    {
        double eps = 1.0e-9;

        double px = p.x;
        double py = p.y;
        double ax = vertices[v1].x;
        double ay = vertices[v1].y;
        double az = vertices[v1].z;
        double bx = vertices[v2].x;
        double by = vertices[v2].y;
        double bz = vertices[v2].z;
        double cx = vertices[v3].x;
        double cy = vertices[v3].y;
        double cz = vertices[v3].z;

        auto abx = bx - ax;
        auto aby = by - ay;
        auto abz = bz - az;

        auto acx = cx - ax;
        auto acy = cy - ay;
        auto acz = cz - az;

        auto bcx = cx - bx;
        auto bcy = cy - by;

        auto d1 = aby * (px - ax) - abx * (py - ay);
        auto d2 = bcy * (px - bx) - bcx * (py - by);
        auto d3 = acx * (py - cy) - acy * (px - cx);

        // pre-calculate and store the normal and d value as part of the triangle
        auto normal_x = aby * acz - abz * acy;
        auto normal_y = abz * acx - abx * acz;
        auto normal_z = abx * acy - aby * acx;

        auto d = normal_x * ax + normal_y * ay + normal_z * az;
        // auto hit = d1 > -eps && d2 > -eps && d3 > -eps;
        auto hit = d1 < eps && d2 < eps && d3 < eps || d1 > -eps && d2 > -eps && d3 > -eps;
        if (hit)
            return (d - normal_x * px - normal_y * py) / normal_z;
        else
            return std::numeric_limits<FLOAT_TYPE>::quiet_NaN();
    }

    inline bool
    PointInTriangle(std::vector<Vertex> &vertices, Vertex p)
    {
        double eps = 1.0e-9;

        double px = p.x;
        double py = p.y;
        double ax = vertices[v1].x;
        double ay = vertices[v1].y;
        double az = vertices[v1].z;
        double bx = vertices[v2].x;
        double by = vertices[v2].y;
        double bz = vertices[v2].z;
        double cx = vertices[v3].x;
        double cy = vertices[v3].y;
        double cz = vertices[v3].z;

        double d1 = (px - ax) * (by - ay) - (bx - ax) * (py - ay);
        double d2 = (px - bx) * (cy - by) - (cx - bx) * (py - by);
        double d3 = (px - cx) * (ay - cy) - (ax - cx) * (py - cy);

        bool hit = d1 < eps && d2 < eps && d3 < eps || d1 > eps && d2 > eps && d3 > eps;

        return hit;
    }

    inline FLOAT_TYPE PointInTriangleHeight(std::vector<Vertex> &vertices, Vertex p)
    {
        double px = p.x;
        double py = p.y;
        double ax = vertices[v1].x;
        double ay = vertices[v1].y;
        double az = vertices[v1].z;
        double bx = vertices[v2].x;
        double by = vertices[v2].y;
        double bz = vertices[v2].z;
        double cx = vertices[v3].x;
        double cy = vertices[v3].y;
        double cz = vertices[v3].z;

        double abx = bx - ax;
        double aby = by - ay;
        double abz = bz - az;
        double acx = cx - ax;
        double acy = cy - ay;
        double acz = cz - az;

        double normal_x = aby * acz - abz * acy;
        double normal_y = abz * acx - abx * acz;
        double normal_z = abx * acy - aby * acx;

        double d = -(normal_x * ax + normal_y * ay + normal_z * az);

        double z = -(normal_x * px + normal_y * py + d) / normal_z;

        return z;
    }
};

class Node
{
public:
    size_t start, end;
    FLOAT_TYPE min, max;
    uint8_t axis;
    size_t child_index;

    Node(size_t range_start, size_t range_end, AABB aabb, uint8_t split_axis, size_t child_node = 0)
    {
        start = range_start;
        end = range_end;
        min = split_axis == 0 ? aabb.x_min : aabb.y_min;
        max = split_axis == 0 ? aabb.x_max : aabb.y_max;
        axis = split_axis;
        child_index = child_node;
    }

    inline bool ContainsPoint(Vertex p)
    {
        // return axis == 0 ? (min <= p.x && p.x <= max) : (min <= p.y && p.y <= max);
        return axis == 0 ? !(min > p.x || p.x > max) : !(min > p.y || p.y > max);
    }
};

class Mesh
{

public:
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    std::vector<Node> nodes;

    int leaf_size = 8;

    Mesh() {};

    ~Mesh()
    {
        vertices.clear();
        triangles.clear();
        nodes.clear();
    };

    AABB GetTriangleListBounds(size_t range_start, size_t range_end)
    {
        auto aabb = triangles[range_start].GetAABB(vertices);
        for (size_t i = range_start; i < range_end; i++)
        {
            aabb = AABB(aabb, triangles[i].GetAABB(vertices));
        }
        return aabb;
    };

    bool SplitNode(Node &node)
    {
        size_t range_start = node.start;
        size_t range_end = node.end;

        // Check if small enough batch for a leaf node
        if (range_end - range_start < leaf_size)
        {
            return true;
        }

        // Step 1: Get extreme bounds
        auto aabb = GetTriangleListBounds(range_start, range_end);

        // Step 2: Determine splitting axis
        uint8_t axis = aabb.GetSplitAxis();

        // Step 3: Sort along split axis
        // std::sort(triangles.begin() + range_start, triangles.begin() + range_end,
        //           [this, axis](auto &lhs, auto &rhs)
        //           {
        //               return lhs.GetSortValue(vertices, 1 - axis) < rhs.GetSortValue(vertices, 1 - axis);
        //           });

        std::sort(triangles.begin() + range_start, triangles.begin() + range_end,
                  [this, axis](auto &lhs, auto &rhs)
                  {
                      return lhs.GetSortValue(vertices, axis) < rhs.GetSortValue(vertices, axis);
                  });

        // Step 4: Create split node
        size_t split_idx = (range_start + range_end) / 2.0;
        auto bounds_a = GetTriangleListBounds(range_start, split_idx);
        auto bounds_b = GetTriangleListBounds(split_idx, range_end);
        Node node_a(range_start, split_idx, bounds_a, axis);
        Node node_b(split_idx, range_end, bounds_b, axis);

        // Set index to first child (second can be inferred)
        node.child_index = nodes.size();

        // Push nodes to tree
        nodes.push_back(node_a);
        nodes.push_back(node_b);

        return false;
    }

    void BuildTree()
    {
        // Push root node
        AABB aabb = GetTriangleListBounds(0, triangles.size());
        uint8_t axis = aabb.GetSplitAxis();
        Node node = Node(0, triangles.size(), aabb, axis);
        nodes.push_back(node);

        for (size_t i = 0; i < nodes.size(); i++)
        {
            SplitNode(nodes[i]);
        }
    }

    Hit TraverseTree(Vertex p)
    {
        std::vector<size_t> node_list;
        node_list.push_back(0);

        Hit hit = Hit{};

        while (!node_list.empty())
        {
            // hit.stack_size = fmax(node_list.size(), hit.stack_size);

            // Pop last element of the list
            size_t cur_idx = node_list[node_list.size() - 1];
            node_list.pop_back();

            // Check if the query point is within the bounds
            Node cur_node = nodes[cur_idx];

            if (cur_node.child_index)
            {
                // Add child nodes to the node list
                if (nodes[cur_node.child_index + 0].ContainsPoint(p))
                    node_list.push_back(cur_node.child_index + 0);

                if (nodes[cur_node.child_index + 1].ContainsPoint(p))
                    node_list.push_back(cur_node.child_index + 1);
            }
            else
            {
                for (size_t i = cur_node.start; i < cur_node.end; i++)
                {
                    auto height = triangles[i].PointInTriangleHeightNan(vertices, p);
                    if (!std::isnan(height))
                    {
                        if (!hit.is_hit)
                        {
                            hit.is_hit = true;
                            hit.height = height;
                        }
                        else
                        {
                            hit.height = fmax(height, hit.height);
                        }
                    }
                }
            }
        }

        return hit;
    }

    Hit TraverseTreeStack(Vertex p)
    {
        size_t node_list[64]{};
        size_t stack_idx = 0;

        // add root node to the stack
        node_list[stack_idx++] = 0;

        Hit hit = Hit{};

        while (stack_idx)
        {
            // Pop last element of the list
            size_t cur_idx = node_list[--stack_idx];

            // Check if the query point is within the bounds
            Node cur_node = nodes[cur_idx];

            if (cur_node.child_index)
            {
                // Add child nodes to the node list
                if (nodes[cur_node.child_index + 0].ContainsPoint(p))
                    node_list[stack_idx++] = (cur_node.child_index + 0);

                if (nodes[cur_node.child_index + 1].ContainsPoint(p))
                    node_list[stack_idx++] = (cur_node.child_index + 1);
            }
            else
            {
                for (size_t i = cur_node.start; i < cur_node.end; i++)
                {
                    // Test each triangle in the current leaf node
                    auto height = triangles[i].PointInTriangleHeightNan(vertices, p);
                    if (!std::isnan(height))
                    {
                        if (!hit.is_hit)
                        {
                            hit.is_hit = true;
                            hit.height = height;
                        }
                        else
                        {
                            hit.height = fmax(height, hit.height);
                        }
                    }
                }
            }
        }

        return hit;
    }

    Hit TraverseTreeRec(Vertex p, size_t cur_idx = 0, Hit h = Hit{})
    {
        // Check if the query point is within the bounds
        Node cur_node = nodes[cur_idx];
        if (cur_node.child_index)
        {
            // Add child nodes to the node list
            auto idx1 = cur_node.child_index + 0;
            if (nodes[idx1].ContainsPoint(p))
                h = TraverseTreeRec(p, idx1, h);

            auto idx2 = cur_node.child_index + 1;
            if (nodes[idx2].ContainsPoint(p))
                h = TraverseTreeRec(p, idx2, h);
        }
        else
        {
            for (size_t i = cur_node.start; i < cur_node.end; i++)
            {
                auto height = triangles[i].PointInTriangleHeightNan(vertices, p);
                if (!std::isnan(height))
                {
                    if (!h.is_hit)
                    {
                        h.is_hit = true;
                        h.height = height;
                    }
                    else
                    {
                        h.height = fmax(height, h.height);
                    }
                }
            }
        }

        return h;
    }

    void LoadMesh(std::string filename, bool y_up = false)
    {
        vertices.clear();
        triangles.clear();
        nodes.clear();

        std::ifstream in(filename, std::ios::in);
        if (!in)
        {
            std::cerr << "Cannot open " << filename << std::endl;
            exit(1);
        }
        std::string line;
        while (std::getline(in, line))
        {
            if (line.substr(0, 2) == "v ")
            {
                std::istringstream v(line.substr(2));
                double x, y, z;
                v >> x;
                v >> y;
                v >> z;
                if (y_up)
                {
                    auto vert = Vertex(x, z, y);
                    vertices.push_back(vert);
                }
                else
                {
                    auto vert = Vertex(x, y, z);
                    vertices.push_back(vert);
                }
            }

            else if (line.substr(0, 2) == "f ")
            {
                // size_t a, b, c; //to store mesh index
                /*
                int a, b, c; //to store mesh index
                const char* chh = line.c_str();
                sscanf(chh, "f %i %i %i", &a, &b, &c);
                auto triangle = Triangle(a - 1, b - 1, c - 1);
                triangles.push_back(triangle);
                */

                const char *chh = line.c_str();

                std::string delim = " ";
                std::string delim_bar = "/";
                std::vector<size_t> parts;

                auto start = 0U;
                auto end = line.find(delim);
                while (end != std::string::npos)
                {
                    start = end + delim.length();
                    end = line.find(delim, start);
                    auto part = line.substr(start, end - start);
                    auto bar_end = part.find(delim_bar, 0U);
                    if (std::string::npos == bar_end)
                        parts.push_back(std::stoi(part));
                    else
                        parts.push_back(std::stoi(part.substr(0U, bar_end)));
                }

                auto triangle = Triangle(parts[0] - 1, parts[1] - 1, parts[2] - 1);
                triangles.push_back(triangle);
            }
        }
        in.close();
    }
};

int main()
{
    std::cout << "Loading mesh" << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    Mesh mesh;
    mesh.LoadMesh("../data/paul_ricard_colidable.obj", true);
    // mesh.LoadMesh("../data/le_mans_collidable.obj");
    mesh.BuildTree();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);

    std::cout << "Mesh loaded in: " << duration.count() << " [ms] " << std::endl;

    size_t tris = mesh.triangles.size();
    size_t verts = mesh.vertices.size();
    size_t nodes = mesh.nodes.size();

    std::cout << "Triangles: " << tris << std::endl;
    std::cout << "Vertices: " << verts << std::endl;
    std::cout << "Nodes: " << nodes << std::endl;

    auto p = Vertex(0.0, 0.0, 0.0);

    Hit lc;

    start = std::chrono::high_resolution_clock::now();

    // size_t max_stack = 0;
    size_t match_count = 0;
    for (auto triangle : mesh.triangles)
    {
        auto pt = triangle.GetCentroid(mesh.vertices);
        lc = mesh.TraverseTreeStack(pt);
        if (fabs(lc.height - pt.z) < 1.0e-6)
            match_count++;
        // else
        //    std::cout << "x,y: " << pt.x << "," << pt.y << " : " << lc.height << " " << pt.z << std::endl;
    }

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);

    std::cout << "Time taken by function: "
              << duration.count() << " [ms] "
              << mesh.triangles.size() / duration.count() << " [tri/ms] "
              << match_count * 100.0 / double(mesh.triangles.size()) << " [%]" << std::endl;

    std::cout << "Finished!";
    return 0;
}