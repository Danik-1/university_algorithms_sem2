#include <iostream>
#include <queue>
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

template <typename VType, typename EType = std::pair<VType, size_t>>
class AbstractGraph {
public:
  explicit AbstractGraph(const size_t& vertexes_num, const size_t& edges_num = 0) :
      vertexes_number_(vertexes_num), edges_number_(edges_num) {}

  size_t GetvertexesNumber() const { return vertexes_number_; }

  virtual std::vector<std::pair<VType, size_t>> GetNeighbours(const VType& vertex) = 0;
protected:
  size_t vertexes_number_ = 0;
  size_t edges_number_ = 0;
};

template <typename VType = size_t, typename EType = std::pair<VType, VType>>
class ListGraph : public AbstractGraph<VType> {
public:
  ListGraph(const size_t& vertexes_num, const size_t& edges_num = 0) :
      AbstractGraph<VType, EType>(vertexes_num, edges_num) {}

  ListGraph(const size_t& vertexes_num, const std::vector<std::pair<EType, size_t>>& edges)
      : AbstractGraph<VType, EType>(vertexes_num, edges.size()) {
    for (size_t i = 0; i < vertexes_num; ++i) {
      lists_[i] = std::vector<std::pair<VType, size_t>>();
    }
    for (const auto& edge : edges) {
      lists_[edge.first.first].push_back({edge.first.second, edge.second});
    }
  }

  std::vector<std::pair<VType, size_t>> GetNeighbours(const VType& vertex) final {
    return lists_[vertex];
  }

private:
  std::unordered_map<VType, std::vector<std::pair<VType, size_t>>> lists_;
};

template <typename VType = size_t, typename EType = std::pair<VType, VType>>
std::vector<std::pair<EType, size_t>> ReadEdgesAndCosts(
    const size_t& edges_number) {
  std::vector<std::pair<EType, size_t>> edges(edges_number);
  for (size_t i = 0; i < edges_number; ++i) {
    std::cin >> edges[i].first.first >> edges[i].first.second;
    std::cin >> edges[i].second;
  }
  return edges;
}

template <typename VType = size_t>
void CostOfCheapestRouteBFS(const VType& start, const VType& finish,
                              ListGraph<VType>& graph) {
  std::queue<VType> bfs_queue;
  std::vector<size_t> dist(graph.GetvertexesNumber() + 1, 0);

  bfs_queue.push(start);
  dist[start] = 0;

  while (!bfs_queue.empty()) {
    VType cur_vertex = bfs_queue.front();
    bfs_queue.pop();
    for (const auto& edge : graph.GetNeighbours(cur_vertex)) {
      auto neighbour = edge.first;
      if (dist[neighbour] == 0 || dist[neighbour] > dist[cur_vertex] + edge.second) {
        dist[neighbour] = dist[cur_vertex] + edge.second;
        bfs_queue.push(neighbour);
      }
    }
  }
  if (dist[finish] != 0) {
    std::cout << dist[finish];
  } else {
    std::cout << -1;
  }
}

template <typename VType = size_t>
void PrintRouteLenghtAndPlanets(std::vector<VType>& route) {
  size_t size = route.size();
  if (size == 0) {
    std::cout << -1;
  } else {
    std::cout << size - 1 << '\n';
    for (size_t i = 0; i < size; ++i)
      std::cout << route[i] << ' ';
  }
}
 
int main() {
  using VType = size_t;

  size_t vertexes_number = 0;
  size_t edges_number = 0;
  std::cin >> vertexes_number >> edges_number;

  VType start = 0;
  VType finish = 0;
  std::cin >> start >> finish;

  ListGraph<VType> graph(vertexes_number, ReadEdgesAndCosts(edges_number));
  CostOfCheapestRouteBFS(start, finish, graph);

  return 0;
}