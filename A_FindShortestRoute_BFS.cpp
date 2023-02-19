#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

template <typename VType = size_t, typename EType = std::pair<VType, VType>>
class ListGraph {
 public:
  ListGraph(const size_t& vertexes_num, const std::vector<EType>& edges)
      : vertexes_number_(vertexes_num), edges_number_(edges.size()) {
    for (size_t i = 0; i < vertexes_num; ++i) {
      lists_[i] = std::vector<VType>();
    }

    for (const auto& edge : edges) {
      lists_[edge.first].push_back(edge.second);
      lists_[edge.second].push_back(edge.first);
    }
  }

  std::vector<VType> GetNeighbours(const VType& vertex) {
    return lists_[vertex];
  }

  size_t GetvertexesNumber() const { return vertexes_number_; }

 private:
  std::unordered_map<VType, std::vector<VType>> lists_;
  size_t vertexes_number_ = 0;
  size_t edges_number_ = 0;
};

template <typename VType = size_t, typename EType = std::pair<VType, VType>>
std::vector<EType> ReadEdges(const size_t& edges_number) {
  std::vector<EType> edges(edges_number);
  for (size_t i = 0; i < edges_number; ++i) {
    std::cin >> edges[i].first >> edges[i].second;
  }
  return edges;
}

template <typename VType = size_t>
std::vector<VType> RecoverRoute(std::unordered_map<VType, VType>& parent,
                                const VType& finish) {
  std::vector<VType> route;
  route.push_back(finish);
  while (parent[route.back()] != 0) {
    route.push_back(parent[route.back()]);
  }

  std::reverse(route.begin(), route.end());
  return route;
}

template <typename VType = size_t>
std::vector<VType> FindShortestRouteBFS(const VType& start, const VType& finish,
                                        ListGraph<VType>& graph) {
  std::queue<VType> bfs_queue;
  bfs_queue.push(start);

  std::vector<bool> visited(graph.GetvertexesNumber() + 1, false);
  visited[start] = true;

  std::unordered_map<VType, VType> parent(graph.GetvertexesNumber() + 1);
  parent[start] = 0;

  while (!bfs_queue.empty()) {
    VType cur_vertex = bfs_queue.front();
    bfs_queue.pop();
    for (const auto& neighbour : graph.GetNeighbours(cur_vertex)) {
      if (!visited[neighbour]) {
        visited[neighbour] = true;
        bfs_queue.push(neighbour);
        parent[neighbour] = cur_vertex;
      }
    }
  }
  if (visited[finish]) {
    return RecoverRoute(parent, finish);
  }
  return {};
}

template <typename VType = size_t>
void PrintRouteLenghtAndPlanets(std::vector<VType>& route) {
  size_t size = route.size();
  if (size == 0) {
    std::cout << -1;
  } else {
    std::cout << size - 1 << '\n';
    for (const auto& item : route) {
      std::cout << item << ' ';
    }
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

  ListGraph<VType> graph(vertexes_number, ReadEdges(edges_number));

  std::vector<VType> route = FindShortestRouteBFS(start, finish, graph);
  PrintRouteLenghtAndPlanets(route);

  return 0;
}