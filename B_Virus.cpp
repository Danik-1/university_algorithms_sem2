#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

template <typename VType = size_t,
          typename Graph = std::vector<std::vector<std::pair<VType, size_t>>>>
Graph ReadGraph(const size_t& number_of_vertexes,
                const size_t& number_of_edges) {
  Graph graph(number_of_vertexes);
  size_t from = 0, to = 0, weight = 0;
  for (size_t i = 0; i < number_of_edges; ++i) {
    std::cin >> from >> to >> weight;
    --from;
    --to;
    graph[from].push_back({to, weight});
    graph[to].push_back({from, weight});
  }

  return graph;
}

template <typename VType = size_t,
          typename Graph = std::vector<std::vector<std::pair<VType, size_t>>>>
std::vector<int64_t> FindTimeForVirus(const Graph& graph,
                                      std::vector<VType>& starts) {
  const int64_t kInf = 2'009'000'999;
  std::vector<size_t> distances(graph.size(), kInf);
  std::priority_queue<std::pair<VType, size_t>,
                      std::vector<std::pair<VType, size_t>>,
                      std::greater<std::pair<VType, size_t>>>
      queue;
  for (size_t i = 0; i < starts.size(); ++i) {
    distances[starts[i]] = 0;
    queue.emplace(distances[starts[i]], starts[i]);
  }

  while (!queue.empty()) {
    auto [cur, dist] = queue.top();
    queue.pop();
    if (dist <= distances[cur]) {
      for (const auto& neighbour : graph[cur]) {
        auto [to, distance] = neighbour;
        if (distances[cur] + distance < distances[to]) {
          distances[to] = distances[cur] + distance;
          queue.emplace(to, distances[to]);
        }
      }
    }
  }

  return distances;
}

template <typename Graph = std::vector<std::vector<std::pair<int, int>>>>
int64_t FindTimeToEscape(const Graph& graph, const size_t& start,
                         const size_t& finish,
                         const std::vector<int64_t>& time) {
  const int64_t kInf = 2'009'000'999;
  std::vector<int64_t> distances(graph.size(), kInf);
  distances[start] = 0;

  std::priority_queue<std::pair<size_t, size_t>,
                      std::vector<std::pair<size_t, size_t>>,
                      std::greater<std::pair<size_t, size_t>>>
      queue;
  queue.emplace(start, distances[start]);

  while (!queue.empty()) {
    auto [cur, dist] = queue.top();
    queue.pop();
    if (dist <= distances[cur]) {
      for (const auto& neighbour : graph[cur]) {
        auto [to, distance] = neighbour;
        if (distances[cur] + distance < distances[to]) {
          distances[to] = distances[cur] + distance;
          queue.emplace(to, distances[to]);
        }
      }
    }
  }

  return distances[finish] == (kInf) || distances[finish] == time[finish]
             ? -1
             : distances[finish];
}

std::vector<int32_t> ReadViruses(const size_t& number_of_viruses) {
  std::vector<int32_t> viruses(number_of_viruses, 0);
  for (auto& item : viruses) {
    std::cin >> item;
    --item;
  }

  return viruses;
}

template <typename T>
void PrinVector(std::vector<T> vector) {
  for (const auto& item : vector) {
    std::cout << item << ' ';
  }
}

int main() {
  size_t number_of_edges, number_of_vertexes, number_of_viruses;
  std::cin >> number_of_vertexes >> number_of_edges >> number_of_viruses;

  auto viruses = ReadViruses(number_of_viruses);
  auto graph = ReadGraph(number_of_vertexes, number_of_edges);

  int32_t start, finish;
  std::cin >> start >> finish;
  std::cout << FindTimeToEscape(graph, start - 1, finish - 1,
                                FindTimeForVirus(graph, viruses));

  return 0;
}