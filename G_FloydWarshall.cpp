#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

template <typename EType = bool,
          typename Matrix = std::vector<std::vector<EType>>>
Matrix ReadEdgesFromMatrix(const int8_t& number_of_vertexes) {
  Matrix edges;
  for (int8_t to = 0; to < number_of_vertexes; ++to) {
    for (int8_t from = 0; from < number_of_vertexes; ++from) {
      bool edge;
      std::cin >> edge;
      edges[from][to] = edge;
    }
  }

  return edges;
}

template <typename EType = bool,
          typename Matrix = std::vector<std::vector<EType>>>
Matrix FloydWarshall(Matrix edges) {
  int8_t number_of_vertexes = edges.size();
  for (int8_t k = 0; k < number_of_vertexes; ++k)
    for (int8_t u = 0; u < number_of_vertexes; ++u)
      for (int8_t v = 0; v < number_of_vertexes; ++v)
        edges[u][v] = edges[u][v] || (edges[u][k] + edges[k][v]);

  return edges;
}

template <typename T = int8_t>
void PrintMatrix(const std::vector<std::vector<T>>& matrix) {
  for (const auto& row : matrix) {
    for (const auto& element : row) {
      std::cout << element << ' ';
    }
    std::cout << '\n';
  }
}

int main() {
  int8_t number_of_vertexes = 0;
  std::cin >> number_of_vertexes;

  std::vector<std::vector<bool>> matrixGraph = ReadEdgesFromMatrix(number_of_vertexes);
  std::vector<std::vector<bool>> allPaths = FloydWarshall(matrixGraph);
  PrintMatrix(allPaths);

  return 0;
}
