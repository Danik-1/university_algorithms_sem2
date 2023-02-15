#include <vector>
#include <iostream>
#include <limits>

static constexpr size_t kInfty = std::numeric_limits<size_t>::max();
namespace graph {
  using GraphType = std::vector<std::vector<size_t>>;
  using EdgesType = std::vector<std::pair<size_t, size_t>>;
  using TimeType = std::vector<size_t>;
}

void BridgesDfs(
    const graph::GraphType& graph,
    const graph::EdgesType& edges,
    graph::TimeType& time_in,
    graph::TimeType& f_up,
    size_t vertex,
    size_t edge_id,
    size_t& time,
    std::vector<size_t>& bridges) {
  time_in[vertex] = time++;
  f_up[vertex] = time_in[vertex];
  for (auto from_edge_id : graph[vertex]) {
    if (from_edge_id == edge_id) {
      continue;
    }
    auto& edge = edges[from_edge_id];
    size_t to = edge.first == vertex ? edge.second : edge.first;
    if (to == vertex) { // loop
      continue;
    }
    if (time_in[to] != kInfty) {
      f_up[vertex] = std::min(f_up[vertex], time_in[to]);
    } else {
      BridgesDfs(
          graph,
          edges,
          time_in,
          //     time_out,
          f_up,
          to,
          from_edge_id,
          time,
          bridges
      );
      f_up[vertex] = std::min(f_up[to], f_up[vertex]);
    }
    if (f_up[to] > time_in[vertex]) {
      bridges.push_back(from_edge_id);
    }
  }
}

std::vector<size_t> GetBridges(const graph::GraphType& graph,
                               const graph::EdgesType& edges) {
  size_t num_vertex = graph.size();
  graph::TimeType time_in(num_vertex, kInfty); // time in
                                                   // std::vector<size_t> time_out(num_vertex, kInfty);
  graph::TimeType f_up(num_vertex, kInfty); // aka RET(from lecture) magic function :)
  std::vector<size_t> bridges; // result - all bridges
  size_t time = 0; // time now

  for (size_t i = 0; i < num_vertex; ++i) {
    if (time_in[i] == kInfty) {
      BridgesDfs(
          graph,
          edges,
          time_in,
          //       time_out,
          f_up,
          i,
          kInfty,
          time,
          bridges
      );
    }
  }
  return bridges;
}

graph::EdgesType ReadEdges(const size_t& edges_num, graph::GraphType& graph) {
  graph::EdgesType edges;
  edges.reserve(edges_num);

  for (size_t id = 0; id < edges_num; ++id) {
    size_t from = 0;
    size_t to = 0;
    std::cin >> from >> to;

    edges.emplace_back(std::min(from, to),std::max(from, to));
    graph[from].push_back(id);
    graph[to].push_back(id);
  }
  return edges;
}

void PrintBridges(const std::vector<size_t>& bridges,
                  const graph::EdgesType& edges) {
  std::cout << "Bridges: \n";
  for (auto bridge : bridges)
    std::cout << edges[bridge].first << " " << edges[bridge].second << "\n";
}

int main() {
  size_t n, m;
  std::cin >> n >> m;
  graph::GraphType graph(n, std::vector<size_t>());
  graph::EdgesType edges = ReadEdges(m, graph);

  PrintBridges(GetBridges(graph, edges), edges);
  return 0;
}