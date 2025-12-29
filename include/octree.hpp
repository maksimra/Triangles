#include <memory>
#include <vector>
#include "geometry.hpp"

namespace Space
{
    const int LEAF_CAPACITY = 5; // max number of triangles in leaf

    class OctreeNode
    {
        std::array<std::unique_ptr<OctreeNode>, 8> children;
        std::vector<std::shared_ptr<Geometry::Triangle>> objects;

        Geometry::Point topLeft;
        Geometry::Point bottomRight;
        bool isLeaf;
      public:
        void insert(std::shared_ptr<Geometry::Triangle> object)
        {
            objects.push_back(object);
            if (isLeaf && objects.size() > LEAF_CAPACITY)
            {
                split();
            }c
        }
    };

    class Octree
    {
        std::unique_ptr<OctreeNode> root;

        void insert(std::shared_ptr<Geometry::Triangle> object)
        {
            root.get()->insert(object);
        }
    };
}

// Plan: first: i get all triangles and store them in shared ptr vector
//       then i have size of volume and can create Octree class object.
//       and can create root object.
//       then with cycle process all Nodes and split them if number of Traingles bigger than 5