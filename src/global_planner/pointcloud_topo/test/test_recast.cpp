#include <pointcloud_topo/graph.h>
using namespace fast_planner;
int main(int argc, char** argv) {

  BubbleNode::Ptr b1 = std::make_shared<BubbleNode>(5, Eigen::Vector3d(0, 0, 0));
  BubbleNode::Ptr b2 = std::make_shared<BubbleNode>(5, Eigen::Vector3d(0, 9, 0));
  b1->neighbors_.insert(b2);
  b2->neighbors_.insert(b1);
  BubbleNode::Ptr b3 = std::make_shared<BubbleNode>(5, Eigen::Vector3d(10, 0, 0));
  BubbleNode::Ptr b4 = std::make_shared<BubbleNode>(5, Eigen::Vector3d(0, 13, 0));
  b2->neighbors_.insert(b4);
  b4->neighbors_.insert(b2);
  SetCoverage sc;
  if (sc.bubbleRecast(b1, b3)) {
    cout << "yes" << endl;
  } else {
    cout << "no" << endl;
  }
  if (sc.bubbleRecast(b1, b2)) {
    cout << "yes" << endl;
  } else {
    cout << "no" << endl;
  }
  if (sc.bubbleRecast(b1, b4)) {
    cout << "yes" << endl;
  } else {
    cout << "no" << endl;
  }
}