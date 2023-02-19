#include <iostream>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

template <typename VType, typename EType = std::pair<VType, VType>>
class AbstractGraph {
 public:
  using VertexType = VType;
  using EdgeType = EType;

  explicit AbstractGraph(size_t vertices_num, size_t edges_num = 0)
      : vertices_number_(vertices_num), edges_number_(edges_num) {}

  size_t GetVerticesNumber() const { return vertices_number_; }

  virtual std::vector<VType> GetNeighbours(const VType& vertex) = 0;

 protected:
  size_t vertices_number_ = 0;
  size_t edges_number_ = 0;
};

template <typename VType, typename EType = std::pair<VType, VType>>
class ListGraph : public AbstractGraph<VType, EType> {
 public:
  ListGraph(size_t vertices_num, const std::vector<EType>& edges)
      : AbstractGraph<VType, EType>(vertices_num, edges.size()) {
    for (const auto& edge : edges) {
      list_[edge.first].push_back(edge.second);
      list_[edge.second].push_back(edge.first);
    }
  }

  std::vector<VType> GetNeighbours(const VType& vertex) final {
    return list_[vertex];
  }

 private:
  std::unordered_map<VType, std::vector<VType>> list_;
};

template <typename T = size_t>
std::vector<std::pair<T, T>> GetEdges(const size_t& edges_number) {
  std::vector<std::pair<size_t, size_t>> edges(edges_number);
  for (size_t i = 0; i < edges_number; ++i) {
    std::cin >> edges[i].first >> edges[i].second;
  }

  return edges;
}

template <typename Graph, typename VType = typename Graph::VertexType>
std::vector<VType> DFS(VType& from, Graph* graph, std::vector<bool>& visited,
                       std::vector<VType>& component) {
  visited[from] = true;
  component.push_back(from);
  for (auto now_vertex : graph->GetNeighbours(from)) {
    if (!visited[now_vertex]) {
      DFS(now_vertex, graph, visited, component);
    }
  }
  return component;
}

template <typename Graph, typename VType = typename Graph::VertexType>
std::vector<std::vector<VType>> GetComponents(Graph* graph) {
  std::vector<std::vector<VType>> components;
  std::vector<bool> visited(graph->GetVerticesNumber(), false);

  for (size_t i = 1; i <= graph->GetVerticesNumber(); ++i) {
    std::vector<VType> component;
    if (!visited[i]) {
      components.push_back(DFS(i, graph, visited, component));
    }
  }
  return components;
}

template <typename T = size_t>
void PrintVectorSizeAndElements(const std::vector<T>& vector) {
  int size = vector.size();
  std::cout << size << '\n';
  for (int i = 0; i < size; ++i) {
    std::cout << vector[i] << ' ';
  }

  std::cout << '\n';
}

template <typename T = size_t>
void PrintComponents(const std::vector<std::vector<T>>& components) {
  std::cout << components.size() << '\n';
  for (const auto& component : components) {
    PrintVectorSizeAndElements(component);
  }
}

int main() {
  size_t vertecies_number = 0;
  size_t edges_number = 0;
  std::cin >> vertecies_number >> edges_number;

  ListGraph<size_t> graph(vertecies_number, GetEdges<size_t>(edges_number));
  PrintComponents(GetComponents(&graph));
}
