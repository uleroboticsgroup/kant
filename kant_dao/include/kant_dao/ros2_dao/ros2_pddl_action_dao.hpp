
#ifndef KANT_ROS2_PDDL_ACTION_DAO_HPP
#define KANT_ROS2_PDDL_ACTION_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "simple_node/node.hpp"

#include "kant_msgs/srv/get_pddl_action.hpp"
#include "kant_msgs/srv/update_pddl_action.hpp"
#include "std_srvs/srv/empty.hpp"

#include "kant_knowledge_base/parser/dto_msg_parser.hpp"
#include "kant_knowledge_base/parser/msg_dto_parser.hpp"

#include "kant_dao/dao_interface/pddl_action_dao.hpp"
#include "kant_dto/pddl_action_dto.hpp"

namespace kant {
namespace dao {
namespace ros2_dao {

class Ros2PddlActionDao : public kant::dao::dao_interface::PddlActionDao {
private:
  std::unique_ptr<kant::knowledge_base::parser::DtoMsgParser> dto_msg_parser;
  std::unique_ptr<kant::knowledge_base::parser::MsgDtoParser> msg_dto_parser;

  rclcpp::Client<kant_msgs::srv::GetPddlAction>::SharedPtr get_client;
  rclcpp::Client<kant_msgs::srv::UpdatePddlAction>::SharedPtr
      update_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr delete_all_client;

  std::vector<std::shared_ptr<kant::dto::PddlActionDto>>
  ros2_get(std::string action_name);
  bool ros2_update(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto,
                   int update_type);
  void ros2_delete_all();

public:
  Ros2PddlActionDao(simple_node::Node *node);

  std::shared_ptr<kant::dto::PddlActionDto> get(std::string action_name);
  std::vector<std::shared_ptr<kant::dto::PddlActionDto>> get_all();

  bool save(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool update(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool save_update(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);

  bool delete_one(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool delete_all();
};

} // namespace ros2_dao
} // namespace dao
} // namespace kant

#endif
