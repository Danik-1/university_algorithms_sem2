#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <optional>
#include <algorithm>

template <typename VType, typename EType = std::pair<VType, VType>>
class AbstractGraph {
public:
  explicit AbstractGraph(const size_t& vertexes_num, const size_t& edges_num = 0) :
      vertexes_number_(vertexes_num), edges_number_(edges_num) {}

  size_t GetVerticesNumber() const { return vertexes_number_; }

  virtual std::vector<std::pair<VType, size_t>> GetNeighbours(const VType& vertex) = 0;
protected:
  size_t vertexes_number_ = 0;
  size_t edges_number_ = 0;
};

template <typename VType, typename EType = std::pair<VType, VType>>
class ListGraph : public AbstractGraph<VType> {
public:
  ListGraph(const size_t& vertexes_num, const size_t& edges_num = 0) :
      AbstractGraph<VType, EType>(vertexes_num, edges_num) {}

  ListGraph(const size_t& vertexes_num, const std::vector<EType>& edges) :
      AbstractGraph<VType, EType>(vertexes_num, edges.size()) {
    for (size_t i = 1; i <= vertexes_num; ++i)
      lists_[i] = std::vector<std::pair<VType, size_t>>();

    size_t i = 1;
    for (const auto& edge : edges) {
      lists_[edge.first].push_back({edge.second, i});
      lists_[edge.second].push_back({edge.first, i++});
    }
  }

  std::vector<std::pair<VType, size_t>> GetNeighbours(const VType& vertex) final {
    return lists_[vertex];
  }

private:
  std::unordered_map<VType, std::vector<std::pair<VType, size_t>>> lists_;
};

template <typename VType = size_t, typename EType = std::pair<VType, VType>>
std::vector<EType> ReadEdges(const size_t& edges_number) {
  std::vector<EType> edges(edges_number);
  for (size_t i = 0; i < edges_number; ++i) {
    std::cin >> edges[i].first >> edges[i].second;
  }
  return edges;
}

static constexpr size_t kInfty = std::numeric_limits<size_t>::max();

template <typename VType = size_t, typename Counter = size_t>
void BridgesDfs(
    ListGraph<VType>& graph,
    const VType& vertex,
    const VType& prev_vert,
    std::vector<Counter>& time_in,
    std::vector<Counter>& ret,
    const Counter& time,
    std::vector<size_t>& bridges
) {
  ret[vertex] = time_in[vertex] = time;
  for (auto& v : graph.GetNeighbours(vertex)) {
    VType next_vert = v.first;
    if (next_vert == prev_vert) continue;

    if (time_in[next_vert] != kInfty) {
      ret[vertex] = std::min(ret[vertex], time_in[next_vert]);
    } else {
      BridgesDfs(graph, next_vert, vertex, time_in, ret,time + 1, bridges);
      ret[vertex] = std::min(ret[next_vert], ret[vertex]);
    }

    if (ret[next_vert] > time_in[vertex])
      bridges.push_back(v.second);
  }
}

template <typename VType = size_t, typename Counter = size_t>
std::vector<Counter> GetBridgesIDs(ListGraph<VType>& graph) {
  size_t vertexes_number = graph.GetVerticesNumber();
  std::vector<Counter> time_in(vertexes_number + 1, kInfty);
  std::vector<Counter> ret(vertexes_number + 1, kInfty);
  Counter time = 0;
  std::vector<Counter> bridges;

  for (size_t i = 1; i <= vertexes_number; ++i)
    if (time_in[i] == kInfty)
      BridgesDfs(graph, i, kInfty, time_in, ret, time, bridges);

  std::sort(bridges.begin(), bridges.end());
  return bridges;
}

template <typename VType = size_t>
void PrintVectorSizeAndElements(const std::vector<VType>& vector) {
  int size = vector.size();
  std::cout << size << '\n';
  for (int i = 0; i < size; ++i)
    std::cout << vector[i] << ' ';
}

int main() {
  size_t vertexes_number = 0;
  size_t edges_number = 0;
  std::cin >> vertexes_number >> edges_number;

  ListGraph<size_t> graph(vertexes_number, ReadEdges(edges_number));
  PrintVectorSizeAndElements(GetBridgesIDs(graph));
}
