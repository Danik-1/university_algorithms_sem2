#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

template <typename VType = int32_t>
struct Edge {
  VType from;
  VType to;
  int32_t weight = 1;
};

template <typename VType = size_t, typename EType = Edge<VType>>
class ListGraph {
 public:
  using VertexType = VType;
  using EdgeType = EType;

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

template <typename EType = Edge<int32_t>>
std::vector<EType> ReadEdgesFromMatrix(const int32_t& number_of_vertexes) {
  std::vector<EType> edges;
  int32_t weight = 0;
  for (int32_t to = 1; to < number_of_vertexes + 1; ++to) {
    for (int32_t from = 1; from < number_of_vertexes + 1; ++from) {
      std::cin >> weight;
      if (weight < 100'000) {
        edges.push_back({from, to, weight});
      }
    }
  }

  return edges;
}

template <typename VType = int32_t>
std::vector<VType> RecoverCycle(const std::vector<VType>& parents,
                                const VType& from) {
  VType y = from;
  for (size_t i = 0; i < parents.size(); ++i) {
    y = parents[y];
  }

  std::vector<VType> cycle;
  for (int32_t cur = y; cur != y || cycle.empty(); cur = parents[cur]) {
    cycle.push_back(cur);
  }

  return cycle;
}

template <typename VType = int32_t, typename GraphType = ListGraph<VType>>
std::vector<VType> FindNegativeCycle(const GraphType& graph,
                                     const VType& from) {
  size_t vertexes_number = graph.GetVertexesNumber();
  std::vector<VType> parent(vertexes_number + 1);
  std::vector<int32_t> dist(vertexes_number + 1, 100'000);
  dist[from] = 0;
  for (size_t i = 1; i < vertexes_number + 1; ++i) {
    for (const auto & [ from, to, weight ] : graph.GetEdges()) {
      if (dist[to] > dist[from] + weight) {
        dist[to] = dist[from] + weight;
        parent[to] = from;

        if (i == vertexes_number) {
          return RecoverCycle(parent, from);
        }
      }
    }
  }

  return {};
}

template <typename T = int32_t>
void PrintVector(const std::vector<T>& vector) {
  for (const auto& element : vector) {
    std::cout << element << ' ';
  }
}

template <typename VType = int32_t>
void PrintAnswer(const std::vector<VType>& cycle) {
  if (cycle.empty()) {
    std::cout << "NO";
  } else {
    std::cout << "YES" << '\n';
    std::cout << cycle.size() + 1 << '\n' << cycle.back() << ' ';
    PrintVector(cycle);
  }
}

int main() {
  int32_t number_of_vertexes = 0;
  std::cin >> number_of_vertexes;

  ListGraph<int32_t> graph(number_of_vertexes,
                           ReadEdgesFromMatrix(number_of_vertexes));

  PrintAnswer(FindNegativeCycle(graph, 1));

  return 0;
}
