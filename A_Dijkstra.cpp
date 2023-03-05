#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

template <typename VType = size_t>
struct Edge {
  VType from = 0;
  VType to = 0;
  size_t weight = 0;
};

template <typename VType, typename EType = Edge<VType>>
class AbstractGraph {
 public:
  explicit AbstractGraph(const size_t& vertexes_num,
                         const size_t& edges_num = 0)
      : vertexes_number_(vertexes_num), edges_number_(edges_num) {}

  size_t GetVertexesNumber() const { return vertexes_number_; }

  virtual std::vector<std::pair<VType, size_t>> GetNeighbours(
      const VType& vertex) = 0;

 protected:
  size_t vertexes_number_ = 0;
  size_t edges_number_ = 0;
};

template <typename VType = size_t, typename EType = Edge<VType>>
class ListGraph : public AbstractGraph<VType> {
 public:
  ListGraph(const size_t& vertexes_num,
            const std::vector<EType>& edges)
      : AbstractGraph<VType, EType>(vertexes_num, edges.size()) {
    for (size_t i = 0; i < vertexes_num; ++i) {
      lists_[i] = std::vector<std::pair<VType, size_t>>();
    }
    for (const auto& edge : edges) {
      lists_[edge.from].push_back({edge.to, edge.weight});
      lists_[edge.to].push_back({edge.from, edge.weight});
    }
  }

  std::vector<std::pair<VType, size_t>> GetNeighbours(
      const VType& vertex) final {
    return lists_[vertex];
  }

 private:
  std::unordered_map<VType, std::vector<std::pair<VType, size_t>>> lists_;
};

template <typename VType = size_t, typename EType = Edge<VType>>
std::vector<EType> ReadEdgesAndCosts(
    const size_t& edges_number) {
  std::vector<EType> edges(edges_number);
  for (size_t i = 0; i < edges_number; ++i) {
    std::cin >> edges[i].from >> edges[i].first.to;
    std::cin >> edges[i].weight;
  }
  return edges;
}

template <typename EType = Edge<size_t>>
std::vector<EType> ReadEdges(const size_t& number_of_edges) {
  std::vector<EType> edges(number_of_edges);
  size_t from = 0, to = 0, weight = 0;
  for (size_t i = 0; i < number_of_edges; ++i) {
    std::cin >> from >> to >> weight;
    edges.push_back({from, to, weight});
  }

  return edges;
}

template <typename VType = size_t>
ListGraph<VType> ReadGraph() {
  size_t number_of_vertexes = 0;
  size_t number_of_edges = 0;
  std::cin >> number_of_vertexes >> number_of_edges;

  return ListGraph<VType>(number_of_vertexes, ReadEdges(number_of_edges));
}

template <typename VType = size_t, typename GraphType = ListGraph<VType>>
std::vector<size_t> GetDistancesDijkstra(GraphType& graph,
                                          const VType& from) {
  size_t vertexes_number = graph.GetVertexesNumber();
  const int64_t INF = 2'009'000'999;
  std::vector<size_t> distances(vertexes_number, INF);
  distances[from] = 0;
  std::vector<bool> visited(vertexes_number, false);

  for (size_t i = 0; i < vertexes_number; ++i) {
    int32_t v = -1;
    for (size_t j = 0; j < vertexes_number; ++j) {
      if (!visited[j] && (v == -1 || distances[j] < distances[v])) {
        v = j;
      }
    }
    if (distances[v] == INF){
      break;
    }
    visited[v] = true;

    for (size_t j = 0; j < graph.GetNeighbours(v).size(); ++j) {
      VType to = graph.GetNeighbours(v)[j].first;
      size_t len = graph.GetNeighbours(v)[j].second;
      if (distances[v] + len < distances[to]) {
        distances[to] = distances[v] + len;
      }
    }
  }

  return distances;
}

template <typename T = size_t>
void PrintVector(const std::vector<T>& vector) {
  for (size_t i = 0; i < vector.size(); ++i) {
    std::cout << vector[i] << " ";
  }
}

int main() {
  int8_t number_of_maps;
  std::cin >> number_of_maps;

  std::vector<ListGraph<size_t>> maps;
  for (int8_t i = 0; i < 1; ++i) {
    maps.push_back(ReadGraph());
  }

  size_t from;
  std::cin >> from;
  for (auto& map : maps) {
    std::vector<size_t> distances = GetDistancesDijkstra(map, from);
    PrintVector(distances);
  }

    return 0;
}