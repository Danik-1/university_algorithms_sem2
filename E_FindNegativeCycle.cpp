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
  using VertexType = VType;
  using EdgeType = EType;

  ListGraph(const size_t& vertexes_num, const std::vector<EType>& edges)
      : vertexes_number_(vertexes_num), edges_(edges),
        edges_number_(edges.size())
         {
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

  std::vector<EType> GetEdges() const {
    return edges_;
  }

  size_t GetVertexesNumber() const {
    return vertexes_number_;
  }

 private:
  std::unordered_map<VType, std::vector<VType>> lists_;
  size_t vertexes_number_ = 0;
  std::vector<EType> edges_;
  size_t edges_number_ = 0;
};

template <typename EType = WeightedEdge<int32_t>>
std::vector<EType> ReadMatrixEdges(const size_t& number_of_vertexes) {
  std::vector<EType> edges;
  int32_t weight = 0;
  for (size_t to = 1; to < number_of_vertexes + 1; ++to) {
    for (size_t from = 1; from < number_of_vertexes + 1; ++from) {
      std::cin >> weight;
      if (weight < 100'000) {
        edges.push_back({(int32_t)from, (int32_t)to, weight});
      }
    }
  }

  return edges;
}

template <typename VType = int32_t, typename GraphType = ListGraph<VType>>
std::vector<VType> FindNegativeCycle(const GraphType& graph, const VType& from) {
  std::vector<VType> cycle;
  size_t vertexes_number = graph.GetVertexesNumber();
  std::vector<int32_t> distances(vertexes_number + 1, 100'000);
  distances[from] = 0;
  for (size_t i = 1; i <= vertexes_number; ++i) {
    bool distance_decreased = false;
    for (const auto& edge : graph.GetEdges()) {
      if (distances[edge.from] != 100'000) {
        if (distances[edge.to] > distances[edge.from] + edge.weight) {
          distances[edge.to] = distances[edge.from] + edge.weight;
          distance_decreased = true;
        }
      }
      if (i == vertexes_number && distance_decreased) {
        cycle.push_back(edge.to);
      }
    }
  }

  return cycle;
}

template <typename T = int32_t>
void PrintCycle(const std::vector<T>& cycle) {
  std::cout << cycle.size() + 1 << '\n';
  for (const auto& element : cycle) {
    std::cout << element << ' ';
  }
  std::cout << cycle.front();
}

template <typename VType = int32_t>
void PrintAnswer(std::vector<VType>& cycle) {
  if (cycle.empty()) {
    std::cout << "NO";
  } else {
    std::cout << "YES" << '\n';
    std::reverse(cycle.begin(), cycle.end());
    PrintCycle(cycle);
  }
}

int main() { 
  size_t number_of_vertexes = 0;
  std::cin >> number_of_vertexes;

  std::vector<WeightedEdge<int32_t>> edges 
    = ReadMatrixEdges(number_of_vertexes);
  ListGraph<int32_t> graph(number_of_vertexes, edges);

  std::vector<int32_t> cycle = FindNegativeCycle(graph, 1);
  PrintAnswer(cycle);

  return 0;
}