#include <iostream>
#include <vector>
#include <utility>
#include <unordered_map>

typedef VertexType type;
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
  std::vector<std::pair<size_t, size_t>> edges(edges_number);
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
bool FindCycleDFS(VType& from, Graph* graph,
                  std::unordered_map<VType, Color>& colors,
                  std::vector<VType>* parents) {
  colors[from] = grey;
  for (auto v : graph->GetNeighbours(from)) {
    if (colors[v] == grey ||
        (colors[v] == white && FindCycleDFS(v, graph, colors, parents))) {
      parents->push_back(v);
      return true;
    }
  }
  parents->clear();
  colors[from] = black;
  return false;
}

template <typename Graph, typename VType = type>
std::vector<VType> GetCycle(Graph* graph) {
  std::unordered_map<VType, Color> colors;
  std::vector<VType> cycle;

  for (size_t i = 1; i <= graph->GetVerticesNumber(); ++i)
    if (colors[i] == white && FindCycleDFS(i, graph, colors, &cycle))
      return cycle;
  return {};
}

template <typename VType = size_t>
void PrintCycle(const std::vector<VType>& cycle) {
  if (cycle.empty()) {
    std::cout << "NO";
  } else {
    std::cout << "YES" << "\n";

    std::cout << cycle[0] << " ";
    for (int j = cycle.size() - 1; j > 0; --j)
      std::cout << cycle[j] << " ";
  }
}


int main() {
  size_t vertecies_number = 0;
  size_t edges_number = 0;
  std::cin >> vertecies_number >> edges_number;

  ListGraph<size_t> graph(vertecies_number, GetEdges<size_t>(edges_number));
  PrintCycle(GetCycle(&graph));
}
