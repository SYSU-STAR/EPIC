#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

using namespace std;
using namespace chrono;

const int INF = 1e9;

// 生成随机权重的函数
int generateRandomWeight() {
  return rand() % 100 + 1; // 生成1到100之间的随机权重
}

void floydWarshall(vector<vector<int>>& graph, int V) {
  for (int k = 0; k < V; ++k) {
    for (int i = 0; i < V; ++i) {
      for (int j = 0; j < V; ++j) {
        if (graph[i][k] != INF && graph[k][j] != INF && graph[i][k] + graph[k][j] < graph[i][j]) {
          graph[i][j] = graph[i][k] + graph[k][j];
        }
      }
    }
  }
}

int main() {
  const int V = 1000; // 节点数量

  // 初始化图，INF表示无穷大
  vector<vector<int>> graph(V, vector<int>(V, INF));

  // 设置图的对角线为0，表示节点到自身的距离为0
  for (int i = 0; i < V; ++i) {
    graph[i][i] = 0;
  }

  // 随机生成一些边和权重
  srand(time(0)); // 设置随机数种子

  for (int i = 0; i < V * 2; ++i) {
    int src = rand() % V;  // 随机选择源节点
    int dest = rand() % V; // 随机选择目标节点
    int weight = generateRandomWeight();

    // 设置边和权重
    graph[src][dest] = weight;
  }

  // 计算算法运行时间
  auto start_time = high_resolution_clock::now();

  // 运行弗洛伊德算法
  floydWarshall(graph, V);

  auto end_time = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(end_time - start_time);

  cout << "Floyd-Warshall Algorithm took " << duration.count() << " milliseconds." << endl;

  return 0;
}
