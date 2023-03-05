#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

template <typename EType = int32_t,
          typename Matrix = std::vector<std::vector<EType>>>
Matrix ReadEdgesFromMatrix(const int32_t& number_of_vertexes) {
  Matrix edges(number_of_vertexes, std::vector<EType>(number_of_vertexes, 0));
  int32_t edge = 0;
  for (int32_t from = 0; from < number_of_vertexes; ++from) {
    for (int32_t to = 0; to < number_of_vertexes; ++to) {
      std::cin >> edge;
      edges[from][to] = edge;
    }
  }

  return edges;
}

template <typename EType = int32_t,
          typename Matrix = std::vector<std::vector<EType>>>
Matrix FloydWarshall(Matrix edges) {
  int32_t number_of_vertexes = edges.size();
  for (int32_t k = 0; k < number_of_vertexes; ++k) {
    for (int32_t u = 0; u < number_of_vertexes; ++u) {
      for (int32_t v = 0; v < number_of_vertexes; ++v) {
        edges[u][v] = edges[u][v] || (edges[u][k] && edges[k][v]);
      }
    }
  }

  return edges;
}

template <typename T = int32_t>
void PrintMatrix(const std::vector<std::vector<T>>& matrix) {
  for (const auto& row : matrix) {
    for (const auto& element : row) {
      std::cout << element << ' ';
    }
    std::cout << '\n';
  }
}

int main() {
  int32_t number_of_vertexes = 0;
  std::cin >> number_of_vertexes;

  using Matrix = std::vector<std::vector<int32_t>>;
  Matrix matrix_graph = ReadEdgesFromMatrix(number_of_vertexes);
  Matrix all_paths = FloydWarshall(matrix_graph);

  PrintMatrix(all_paths);

  return 0;
}
