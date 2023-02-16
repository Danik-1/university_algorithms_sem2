#include <algorithm>
#include <iostream>
#include <vector>
#include <utility>
#include <unordered_map>

template <typename Vertex, typename Edge = std::pair<Vertex, Vertex>>
class AbstractGraph {
 public:
  using VertexType = Vertex;
  using EdgeType = Edge;

  explicit AbstractGraph(size_t vertices_num, size_t edges_num = 0) :
      vertices_number_(vertices_num), edges_number_(edges_num) {}

  size_t GetVerticesNumber() const { return vertices_number_; }

  virtual std::vector<Vertex> GetNeighbours(const Vertex& vertex) = 0;
 protected:
  size_t vertices_number_ = 0;
  size_t edges_number_ = 0;
};

template <typename Vertex, typename Edge = std::pair<Vertex, Vertex>>
class ListGraph : public AbstractGraph<Vertex, Edge> {
 public:
  ListGraph(size_t vertices_num, const std::vector<Edge>& edges) :
      AbstractGraph<Vertex, Edge>(vertices_num, edges.size()) {
    for (const auto& edge : edges)
      list_[edge.first].push_back(edge.second);
  }

  std::vector<Vertex> GetNeighbours(const Vertex& vertex) final {
    return list_[vertex];
  }

 private:
  std::unordered_map<Vertex, std::vector<Vertex>> list_;
};

template <typename T = size_t>
std::vector<std::pair<T, T>> GetEdges(const size_t& edges_number) {
  std::vector<std::pair<T, T>> edges(edges_number);
  for (size_t i = 0; i < edges_number; ++i)
    std::cin >> edges[i].first >> edges[i].second;

  return edges;
}

enum Color {
  white,
  grey,
  black
};

template <typename Graph, typename VType = typename Graph::VertexType>
bool DFS(const VType& from, Graph* graph, std::vector<VType>* reverse_sorted,
         std::unordered_map<VType, Color>& colors) {
  colors[from] = grey;
  for (auto v : graph->GetNeighbours(from)) {
    if (colors[v] == grey ||
        (colors[v] == white && DFS(v, graph, reverse_sorted, colors)))
      return true;
  }
  colors[from] = black;
  reverse_sorted->push_back(from);
  return false;
}

template <typename Graph, typename VType = typename Graph::VertexType>
std::vector<VType> TopSort(Graph* graph) {
  std::unordered_map<VType, Color> colors;
  std::vector<VType> reverse_sorted;

  for (size_t i = 1; i <= graph->GetVerticesNumber(); ++i)
    if (colors[i] == white && DFS(i, graph, &reverse_sorted, colors))
      return {};

  std::reverse(reverse_sorted.begin(), reverse_sorted.end());
  return reverse_sorted;
}

template <typename VType = size_t>
void Print(std::vector<VType> sorted_vertexes) {
  if (sorted_vertexes.empty()) {
    std::cout << -1;
  } else {
    for (size_t j = 0; j < sorted_vertexes.size(); ++j)
      std::cout << sorted_vertexes[j] << " ";
  }
}


int main() {
  size_t vertexes_number = 0;
  size_t edges_number = 0;
  std::cin >> vertexes_number >> edges_number;

  ListGraph<size_t> graph(vertexes_number, GetEdges<size_t>(edges_number));
  Print(TopSort(&graph));
}
