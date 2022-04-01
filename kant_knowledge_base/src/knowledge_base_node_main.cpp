
#include <memory>

#include "kant_knowledge_base/knowledge_base/knowledge_base_node.hpp"
#include "simple_node/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<
      kant::knowledge_base::knowledge_base::KnowledgeBaseNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}