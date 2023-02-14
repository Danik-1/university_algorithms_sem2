/*
 *   Copyright (c) 2023 
 *   All rights reserved.
 */

#ifndef KULAPINGRAPH_HPP_
#define KULAPINGRAPH_HPP_


#include <iostream>
#include <queue>
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>


namespace graph {
template <typename T>
struct DefaultEdge : std::pair<T, T> {
  DefaultEdge(const T& first, const T& second)
  : std::pair<T, T>(first, second) {}
  using BaseClass = std::pair<T, T>;
  const T& Start() const { return BaseClass::first; }
  const T& Finish() const { return BaseClass::second; }
};

template <typename Vertex = int, typename Edge = DefaultEdge<Vertex>>
class AbstractGraph {
 public:
  explicit AbstractGraph(size_t vertices_num, size_t edges_num = 0) :
      vertices_number_(vertices_num), edges_number_(edges_num) {}

  size_t GetVerticesNumber() const { return vertices_number_; }
  size_t GetEdgesNumber() const { return edges_number_; }

  virtual std::vector<Vertex> GetNeighbours(const Vertex& vertex) = 0;
 protected:
  size_t vertices_number_ = 0;
  size_t edges_number_ = 0;
};
}  // namespace graph


namespace graph {
template <typename Vertex = int, typename Edge = DefaultEdge<Vertex>>
class AdjacencyListGraph : public AbstractGraph<Vertex, Edge> {
 public:
  AdjacencyListGraph(size_t vertices_num, const std::vector<Edge>& edges) :
      AbstractGraph<Vertex, Edge>(vertices_num, edges.size()) {
    for (const auto& edge : edges) {
      list_[edge.Start()].push_back(edge.Finish());
      list_[edge.Finish()].push_back(edge.Start());
    }
  }

  std::vector<Vertex> GetNeighbours(const Vertex& vertex) final {
    return list_[vertex];
  }

 private:
  std::unordered_map<Vertex, std::vector<Vertex>> list_;
};
}  // namespace graph


namespace traverses::visitors {
template <class Vertex, class Edge>
class BfsVisitor {
 public:
  virtual void TreeEdge(const Edge& /*edge*/) = 0;
  virtual ~BfsVisitor() = default;
};
}  // namespace traverses::visitors


namespace traverses::visitors {
template <class Vertex, class Edge>
class AncestorBfsVisitor : BfsVisitor<Vertex, Edge> {
 public:
  virtual void TreeEdge(const Edge& edge) {
    ancestors_[edge.Finish()] = edge.Start();
  }

  std::unordered_map<Vertex, Vertex> GetMap() const {
    return ancestors_;
  }

  virtual ~AncestorBfsVisitor() = default;

 private:
  std::unordered_map<Vertex, Vertex> ancestors_;
};
}  // namespace traverses::visitors



namespace traverses {

template <class Vertex, class Graph, class Visitor>
void BreadthFirstSearch(Vertex origin_vertex, const Graph &graph,
                        Visitor visitor) {
  std::queue<Vertex> bfs_queue;
  std::unordered_set<Vertex> visited_vertices;

  bfs_queue.push(origin_vertex);
  visited_vertices.insert(origin_vertex);

  while (!bfs_queue.empty()) {
    auto cur_vertex = bfs_queue.front();
    bfs_queue.pop();
    for (auto& neighbour : graph.GetNeighbours(cur_vertex)) {
      if (visited_vertices.find(neighbour) == visited_vertices.end()) {
        visitor.TreeEdge({cur_vertex, neighbour});
        bfs_queue.push(neighbour);
        visited_vertices.insert(neighbour);
      }
    }
  }
}
}  // namespace traverses


#endif  // KULAPINGRAPH_HPP_

