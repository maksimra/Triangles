#pragma once

#include "geometry.hpp"
#include <cmath>
#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <utility>
#include <vector>

class Octree {
  struct Node {
    Geometry::Box box;
    std::list<std::pair<Geometry::Triangle *, size_t>> objects;
    Node *parent;
    std::array<Node *, 8> children;
    uint8_t activeNodes = 0;

    Node(Node *prnt, const Geometry::Box &box_,
         std::list<std::pair<Geometry::Triangle *, size_t>> &&objects_)
        : parent(prnt), box(box_), objects(std::move(objects_)) {}

    Node(Node *prnt, const Geometry::Box &box_) : parent(prnt), box(box_) {}
  };

  enum OctreeState { NOT_BUILT = 0, ALREADY_BUILT = 1 };

  const double minBoxSize = 0.5;
  const int typicalNodeCapacity = 3;
  OctreeState state = NOT_BUILT;
  Node *root = nullptr;
  size_t areaSize = 0;
  std::list<std::pair<Geometry::Triangle *, size_t>> objects;

public:
  Octree() {}
  Octree(const Octree &rhs) = delete;
  Octree(Octree &rhs) = delete;
  Octree &operator=(const Octree &rhs) = delete;
  Octree &operator=(Octree &&rhs) = delete;
  ~Octree() { deleteTree(root); }

  void insert(Geometry::Triangle *object) {
    assert(state == NOT_BUILT && "Insertion must precede tree construction");

    double p1x = object->p1.x;
    double p1y = object->p1.y;
    double p1z = object->p1.z;
    double p2x = object->p2.x;
    double p2y = object->p2.y;
    double p2z = object->p2.z;
    double p3x = object->p3.x;
    double p3y = object->p3.y;
    double p3z = object->p3.z;

    double farthestPoint =
        std::max({std::abs(p1x), std::abs(p1y), std::abs(p1z), std::abs(p2x),
                  std::abs(p2y), std::abs(p2z), std::abs(p3x), std::abs(p3y),
                  std::abs(p3z)});

    size_t boundingBoxSize = static_cast<int>(std::ceil(farthestPoint));
    if (boundingBoxSize > areaSize)
      areaSize = boundingBoxSize;

    objects.push_back({object, objects.size()}); // triangle + triangle number
  }

  void build() {
    assert(state == NOT_BUILT && "Building a tree is possible only once");

    double boxSize = static_cast<double>(areaSize);

    root =
        new Node{/*prnt =*/nullptr,
                 Geometry::Box{Geometry::Point{boxSize, boxSize, boxSize},
                               Geometry::Point{-boxSize, -boxSize, -boxSize}},
                 std::move(objects)};

    auto getBox = [](const Geometry::Point &topRight, double boxSize) {
      return Geometry::Box{topRight, Geometry::Point{topRight.x - boxSize,
                                                     topRight.y - boxSize,
                                                     topRight.z - boxSize}};
    };

    std::queue<std::pair<Node *, double>> queue;
    queue.push({root, boxSize});

    while (!queue.empty()) {
      Node *currentNode = queue.front().first;
      boxSize = queue.front().second;
      queue.pop();
      if (currentNode->objects.size() > typicalNodeCapacity &&
          boxSize > minBoxSize) {
        Geometry::Box box = currentNode->box;
        Geometry::Point topRight = box.getTopRight();
        Geometry::Box newBoxes[8] = {
            getBox(box.getTopRight(), boxSize),
            getBox(Geometry::Point{topRight.x - boxSize, topRight.y - boxSize,
                                   topRight.z - boxSize},
                   boxSize),
            getBox(Geometry::Point{topRight.x - boxSize, topRight.y,
                                   topRight.z - boxSize},
                   boxSize),
            getBox(Geometry::Point{topRight.x - boxSize, topRight.y - boxSize,
                                   topRight.z},
                   boxSize),
            getBox(Geometry::Point{topRight.x, topRight.y - boxSize,
                                   topRight.z - boxSize},
                   boxSize),
            getBox(
                Geometry::Point{topRight.x - boxSize, topRight.y, topRight.z},
                boxSize),
            getBox(
                Geometry::Point{topRight.x, topRight.y - boxSize, topRight.z},
                boxSize),
            getBox(
                Geometry::Point{topRight.x, topRight.y, topRight.z - boxSize},
                boxSize)};

        for (auto objectIt = currentNode->objects.begin();
             objectIt != currentNode->objects.end();) {
          Geometry::Triangle *object = (*objectIt).first;
          bool boxFound = false;
          for (size_t i = 0; i < 8; i++) {
            if (newBoxes[i].contain(*object)) {
              boxFound = true;

              // if child node already created
              if (currentNode->activeNodes & (1 << i)) {
                currentNode->children[i]->objects.push_back(*objectIt);
                objectIt = currentNode->objects.erase(objectIt);
                break;
              }

              currentNode->children[i] = new Node{currentNode, newBoxes[i]};
              currentNode->activeNodes += (1 << i);
              queue.push({currentNode->children[i], boxSize / 2});
              currentNode->children[i]->objects.push_back(*objectIt);
              objectIt = currentNode->objects.erase(objectIt);
              break;
            }
          }
          if (!boxFound)
            ++objectIt;
        }
      }
    }
    state = ALREADY_BUILT;
  }

  void populateIntersectedNumbers(std::set<size_t> &intersecNumbers) const {
    assert(state == ALREADY_BUILT);

    std::list<std::pair<Geometry::Triangle *, size_t>> prntObjects;
    fillNumbers(root, prntObjects, intersecNumbers);
  }

private:
  void
  fillNumbers(Node *node,
              std::list<std::pair<Geometry::Triangle *, size_t>> &prntObjects,
              std::set<size_t> &intersectedNumbers) const {

    // check intersections with parrent nodes' objects
    for (auto pobj : prntObjects) {
      for (auto obj : node->objects) {
        if (obj.first->trianglesIntersection(*(pobj.first))) {
          intersectedNumbers.insert(obj.second);
          intersectedNumbers.insert(pobj.second);
        }
      }
    }

    // check intersections inside this node
    auto endIt = node->objects.end();
    for (auto it = node->objects.begin(); it != endIt; ++it) {
      for (auto nextIt = std::next(it); nextIt != endIt; ++nextIt) {
        if ((*it).first->trianglesIntersection(*((*nextIt).first))) {
          intersectedNumbers.insert((*it).second);
          intersectedNumbers.insert((*nextIt).second);
        }
      }
    }

    if (!node->activeNodes)
      return;

    size_t oldSize = prntObjects.size();
    prntObjects.insert(prntObjects.end(), node->objects.begin(),
                       node->objects.end());

    for (size_t i = 0; i < 8; ++i) {
      if (node->activeNodes & (1 << i)) {
        fillNumbers(node->children[i], prntObjects, intersectedNumbers);
      }
    }

    while (prntObjects.size() > oldSize) {
      prntObjects.pop_back();
    }
  }

  void deleteTree(Node *root) {
    for (auto object : objects)
      delete object.first;

    if (root == nullptr)
      return;

    std::queue<Node *> queue;
    queue.push(root);
    while (!queue.empty()) {
      Node *node = queue.front();
      queue.pop();

      for (size_t i = 0; i < 8; i++) {
        if (node->activeNodes & (1 << i))
          queue.push(node->children[i]);
      }
      delete node;
    }
  }
};