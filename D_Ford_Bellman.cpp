#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

template <typename VType = int32_t>
struct WeightedEdge {
 public:
  VType from = 0;
  VType to = 0;
  int32_t weight = 0;
};

template <typename VType = size_t, typename EType = WeightedEdge<VType>>
class ListGraph {
 public:
  ListGraph(const size_t& vertexes_num, const std::vector<EType>& edges)
      : vertexes_number_(vertexes_num),
        edges_(edges),
        edges_number_(edges.size()) {
    for (size_t i = 0; i < vertexes_num; ++i) {
      lists_[i] = std::vector<VType>();
    }

    for (const auto& edge : edges) {
      lists_[edge.from].push_back(edge.to);
    }
  }

  std::vector<VType> GetNeighbours(const VType& vertex) const {
    return lists_[vertex];
  }

  std::vector<EType> GetEdges() const { return edges_; }

  size_t GetVertexesNumber() const { return vertexes_number_; }

 private:
  std::unordered_map<VType, std::vector<VType>> lists_;
  size_t vertexes_number_ = 0;
  std::vector<EType> edges_;
  size_t edges_number_ = 0;
};

template <typename EType = WeightedEdge<int32_t>>
std::vector<EType> ReadWeightedEdges(const size_t& number_of_edges) {
  std::vector<EType> edges;
  int32_t from = 0, to = 0, weight = 0;
  for (size_t i = 0; i < number_of_edges; ++i) {
    std::cin >> from >> to >> weight;
    edges.push_back({from, to, weight});
  }

  return edges;
}

template <typename VType = int32_t, typename GraphType = ListGraph<VType>>
std::vector<int32_t> FordBellman(const GraphType& graph, const VType& from) {
  size_t vertexes_number = graph.GetVertexesNumber();
  std::vector<int32_t> distances(vertexes_number + 1, 30'000);
  distances[from] = 0;
  for (size_t i = 1; i < vertexes_number; ++i) {
    bool something_changed = false;
    for (const auto& edge : graph.GetEdges()) {
      if (distances[edge.from] != 30'000) {
        if (distances[edge.to] > distances[edge.from] + edge.weight) {
          distances[edge.to] = distances[edge.from] + edge.weight;
          something_changed = true;
        }
      }
    }
    if (!something_changed) {
      break;
    }
  }

  return distances;
}

template <typename T = size_t>
void PrintVector(std::vector<T>& vector) {
  for (size_t i = 1; i < vector.size(); ++i) {
    std::cout << vector[i] << " ";
  }
}

int main() {
  size_t number_of_vertexes = 0;
  size_t number_of_edges = 0;
  std::cin >> number_of_vertexes >> number_of_edges;
  std::vector<WeightedEdge<int32_t>> edges = ReadWeightedEdges(number_of_edges);

  ListGraph<int32_t> graph(number_of_vertexes, edges);

  std::vector<int32_t> lengths = FordBellman(graph, 1);
  PrintVector(lengths);
  return 0;
}