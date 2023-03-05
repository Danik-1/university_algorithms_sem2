#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

template <typename VType = size_t,
          typename Graph = std::vector<std::vector<std::pair<VType, size_t>>>>
Graph ReadGraph() {
  size_t number_of_vertexes = 0;
  size_t number_of_edges = 0;
  std::cin >> number_of_vertexes >> number_of_edges;

  Graph graph(number_of_vertexes);
  size_t from = 0, to = 0, weight = 0;
  for (size_t i = 0; i < number_of_edges; ++i) {
    std::cin >> from >> to >> weight;
    graph[from].push_back({to, weight});
    graph[to].push_back({from, weight});
  }

  return graph;
}

template <typename VType = size_t,
          typename Graph = std::vector<std::vector<std::pair<VType, size_t>>>>
std::vector<size_t> GetDistancesDijkstra(Graph& graph, const VType& from) {
  size_t vertexes_number = graph.size();
  const int64_t kInf = 2'009'000'999;
  std::vector<size_t> distances(vertexes_number, kInf);
  distances[from] = 0;
  std::priority_queue<std::pair<VType, size_t>,
                      std::vector<std::pair<VType, size_t>>,
                      std::greater<std::pair<VType, size_t>>>
      queue;
  queue.emplace(from, distances[from]);

  while (!queue.empty()) {
    auto[cur, dist] = queue.top();
    queue.pop();
    if (dist <= distances[cur]) {
      for (const auto& neighbour : graph[cur]) {
        auto[to, distance] = neighbour;
        if (distances[cur] + distance < distances[to]) {
          distances[to] = distances[cur] + distance;
          queue.emplace(to, distances[to]);
        }
      }
    }
  }

  return distances;
}

template <typename T = size_t>
void PrintVector(const std::vector<T>& vector) {
  for (const auto& item : vector) {
    std::cout << item << ' ';
  }
}

int main() {
  size_t number_of_maps;
  std::cin >> number_of_maps;

  for (size_t i = 0; i < number_of_maps; ++i) {
    std::vector<std::vector<std::pair<size_t, size_t>>> map = ReadGraph();

    size_t from;
    std::cin >> from;
    std::vector<size_t> distances = GetDistancesDijkstra(map, from);
    PrintVector(distances);
    std::cout << '\n';
  }

  return 0;
}