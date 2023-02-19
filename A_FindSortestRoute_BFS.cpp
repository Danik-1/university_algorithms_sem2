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

  ListGraph(const size_t& vertexes_num, const std::vector<EType>& edges) :
      AbstractGraph<VType, EType>(vertexes_num, edges.size()) {
    for (size_t i = 0; i < vertexes_num; ++i)
      lists_[i] = std::vector<std::pair<VType, size_t>>();

    size_t cost = 1;
    for (const auto& edge : edges) {
      lists_[edge.first].push_back({edge.second, cost});
      lists_[edge.second].push_back({edge.first, cost});
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

template <typename VType = size_t>
std::vector<VType> RecoverRoute(std::unordered_map<VType, VType>& from,
                                const VType& finish) {
  std::vector<VType> route;
  route.push_back(finish);
  while (from[route.back()] != 0)
    route.push_back(from[route.back()]);

  std::reverse(route.begin(), route.end());
  return route;
}


template <typename VType = size_t>
std::vector<VType> FindShortestRouteBFS(const VType& start, const VType& finish,
                                       ListGraph<VType>& graph) {
  std::queue<VType> bfs_queue;
  std::unordered_map<VType, size_t> dist;
  std::unordered_set<VType> visited_vertexes;
  std::unordered_map<VType, VType> from;
  from[start] = 0;

  bfs_queue.push(start);
  dist[start] = 0;
  visited_vertexes.insert(start);

  while (!bfs_queue.empty()) {
    VType cur_vertex = bfs_queue.front();
    bfs_queue.pop();
    for (const auto& edge : graph.GetNeighbours(cur_vertex)) {
      auto neighbour = edge.first;
      if (visited_vertexes.find(neighbour) == visited_vertexes.end()) {
        if (dist[neighbour] == 0 || dist[neighbour] > dist[cur_vertex] + 1) {
          dist[neighbour] = dist[cur_vertex] + 1;
          from[neighbour] = cur_vertex;

          if (neighbour == finish)
            return RecoverRoute(from, finish);
        }
        visited_vertexes.insert(neighbour);
        bfs_queue.push(neighbour);
      }
    }
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

  ListGraph<VType> graph(vertexes_number, ReadEdges(edges_number));


  std::vector<VType> route = FindShortestRouteBFS(start, finish, graph);
  PrintRouteLenghtAndPlanets(route);

  return 0;
}