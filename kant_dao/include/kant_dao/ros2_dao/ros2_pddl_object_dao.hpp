
#ifndef KANT_ROS2_PDDL_OBJECT_DAO_HPP
#define KANT_ROS2_PDDL_OBJECT_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "simple_node/node.hpp"

#include "kant_msgs/srv/get_pddl_object.hpp"
#include "kant_msgs/srv/update_pddl_object.hpp"
#include "std_srvs/srv/empty.hpp"

#include "kant_knowledge_base/parser/dto_msg_parser.hpp"
#include "kant_knowledge_base/parser/msg_dto_parser.hpp"

#include "kant_dao/dao_interface/pddl_object_dao.hpp"
#include "kant_dto/pddl_object_dto.hpp"

namespace kant {
namespace dao {
namespace ros2_dao {

class Ros2PddlObjectDao : public kant::dao::dao_interface::PddlObjectDao {
private:
  std::unique_ptr<kant::knowledge_base::parser::DtoMsgParser> dto_msg_parser;
  std::unique_ptr<kant::knowledge_base::parser::MsgDtoParser> msg_dto_parser;

  rclcpp::Client<kant_msgs::srv::GetPddlObject>::SharedPtr get_client;
  rclcpp::Client<kant_msgs::srv::UpdatePddlObject>::SharedPtr
      update_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr delete_all_client;

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
  ros2_get(std::string object_name);
  bool ros2_update(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto,
                   int update_type);
  void ros2_delete_all();

public:
  Ros2PddlObjectDao(simple_node::Node *node);

  std::shared_ptr<kant::dto::PddlObjectDto> get(std::string object_name);
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> get_all();

  bool save(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool update(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool save_update(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);

  bool delete_one(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool delete_all();
};

} // namespace ros2_dao
} // namespace dao
} // namespace kant

#endif
