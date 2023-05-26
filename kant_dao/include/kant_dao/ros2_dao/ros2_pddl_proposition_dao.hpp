
#ifndef KANT_ROS2_PDDL_PROPOSITION_DAO_HPP
#define KANT_ROS2_PDDL_PROPOSITION_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "simple_node/node.hpp"

#include "kant_msgs/srv/get_pddl_proposition.hpp"
#include "kant_msgs/srv/update_pddl_proposition.hpp"
#include "std_srvs/srv/empty.hpp"

#include "kant_knowledge_base/parser/dto_msg_parser.hpp"
#include "kant_knowledge_base/parser/msg_dto_parser.hpp"

#include "kant_dao/dao_interface/pddl_proposition_dao.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"

namespace kant {
namespace dao {
namespace ros2_dao {

class Ros2PddlPropositionDao
    : public kant::dao::dao_interface::PddlPropositionDao {
private:
  std::unique_ptr<kant::knowledge_base::parser::DtoMsgParser> dto_msg_parser;
  std::unique_ptr<kant::knowledge_base::parser::MsgDtoParser> msg_dto_parser;

  rclcpp::Client<kant_msgs::srv::GetPddlProposition>::SharedPtr
      get_client;
  rclcpp::Client<kant_msgs::srv::UpdatePddlProposition>::SharedPtr
      update_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr delete_all_client;

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  ros2_get(int get_type, std::string predicate_name);
  bool ros2_update(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto,
      int update_type);
  void ros2_delete_all();
  bool ros2_exists(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);

public:
  Ros2PddlPropositionDao(simple_node::Node *node);

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_by_predicate(std::string predicate_name);
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> get_goals();
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> get_no_goals();
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> get_all();

  bool
  save(std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool
  update(std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool save_update(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);

  bool delete_one(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool delete_all();
};

} // namespace ros2_dao
} // namespace dao
} // namespace kant

#endif
