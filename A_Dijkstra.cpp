#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
<<<<<<< HEAD
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
=======
#include <unordered_set>
#include <utility>
#include <vector>

template <typename VType = int32_t>
struct WeightedEdge {
  VType from = 0;
  VType to = 0;
  int32_t weight = 0; 
};

template <typename VType = int32_t, typename EType = WeightedEdge<VType>>
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
std::vector<EType> ReadEdges(const int32_t& number_of_vertexes) {
  std::vector<EType> edges;
  int32_t from = 0, to = 0, weight = 0;
  for (int32_t i = 0; i < number_of_vertexes + 1; ++i) {
    std::cin >> from >> to >> weight;
    edges.push_back({from, to, weight});
  }

  return edges;
}

template <typename VType = int32_t, typename GraphType = ListGraph<VType>>
void Relaxation(const GraphType& graph, const VType& from,
                std::vector<VType>& edges);

template <typename VType = int32_t>
std::vector<std::pair<size_t, size_t>> FindNearestWay(
  const std::vector<std::vector<size_t>>& graph, size_t from, size_t to) {
  std::vector<VType> distances(graph.size(), std::numeric_limits<VType>::max());
  distances[from] = 0;

  std::priority_queue<std::pair<VType, size_t>,
                      std::vector<std::pair<VType, size_t>>,
                      std::greater<std::pair<VType, size_t>>> queue; 
  queue.emplace(distances[from], from);
  while (!queue.empty()) {
     auto [dist, now_near] = queue.top();
     queue.pop();
     if (dist > distances[now_near]) {  
      continue;
     }
     for (const auto& neighbour : graph.GetNeighbours(now_near)) {
      if (distances[now_near] + distance < distances[neighbour]) {
        distances[neighbour] = distances[now_near] + distance;
        queue.emplace(distances[neighbour], neighbour);
      }
     }
>>>>>>> 21a3129 (new file:   A_Dijkstra.cpp)
  }

  return distances;
}

<<<<<<< HEAD
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
=======
template <typename VType = int32_t, typename GraphType = ListGraph<VType>>
std::vector<VType> GetDistancesDijkstra(const GraphType& graph,
                                         const VType& start) {
  const int32_t inf = 2'009'000'999;
  size_t vertexes_number = graph.GetVertexesNumber();
  std::vector<VType> distance_found(vertexes_number + 1);
  std::vector<int32_t> d(vertexes_number + 1, inf);
  std::vector<int32_t> distances(vertexes_number + 1, inf);
  d[start] = distances[start] = 0;
  distance_found.push_back(start);
  for (size_t i = 1; i < vertexes_number + 1; ++i) {
    Relaxation(graph, i, d, distances);
  }

  return distances;
}

template <typename VType = int32_t>
void PrintVector(std::vector<VType>& vector) {
  for (size_t i = 0; i < vector.size(); ++i) {
    std::cout << vector[i] << " ";
  }
  std::cout << '\n';
}

int main() {
  int8_t number_of_inputs = 0;
  std::cin >> number_of_inputs;

  int32_t number_of_vertexes = 0;
  int32_t number_of_edges = 0;
  std::vector<ListGraph<int32_t>> graphs;
  for (int8_t i = 0; i < number_of_inputs; ++i) {
    std::cin >> number_of_vertexes >> number_of_edges;

    ListGraph<int32_t> graph(number_of_vertexes, ReadEdges(number_of_vertexes));
    graphs.push_back(graph);
  }
  int32_t start = 0;
  std::cin >> start;
  for (int8_t i = 0; i < number_of_inputs; ++i) {
    std::vector<int32_t> distances = GetDistancesDijkstra(graphs[i], start);
    PrintVector(distances);
  }

  return 0;
}
>>>>>>> 21a3129 (new file:   A_Dijkstra.cpp)
