
#ifndef KANT_ROS2_PDDL_PREDICATE_DAO_HPP
#define KANT_ROS2_PDDL_PREDICATE_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "simple_node/node.hpp"

#include "kant_interfaces/srv/get_pddl_predicate.hpp"
#include "kant_interfaces/srv/update_pddl_predicate.hpp"
#include "std_srvs/srv/empty.hpp"

#include "kant_knowledge_base/parser/dto_msg_parser.hpp"
#include "kant_knowledge_base/parser/msg_dto_parser.hpp"

#include "kant_dao/dao_interface/pddl_predicate_dao.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"

namespace kant {
namespace dao {
namespace ros2_dao {

class Ros2PddlPredicateDao : public kant::dao::dao_interface::PddlPredicateDao {
private:
  std::unique_ptr<kant::knowledge_base::parser::DtoMsgParser> dto_msg_parser;
  std::unique_ptr<kant::knowledge_base::parser::MsgDtoParser> msg_dto_parser;

  rclcpp::Client<kant_interfaces::srv::GetPddlPredicate>::SharedPtr get_client;
  rclcpp::Client<kant_interfaces::srv::UpdatePddlPredicate>::SharedPtr
      update_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr delete_all_client;

  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
  ros2_get(std::string predicate_name);
  bool
  ros2_update(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto,
              int update_type);
  void ros2_delete_all();

public:
  Ros2PddlPredicateDao(simple_node::Node *node);

  std::shared_ptr<kant::dto::PddlPredicateDto> get(std::string predicate_name);
  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>> get_all();

  bool save(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool update(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool
  save_update(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);

  bool
  delete_one(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool delete_all();
};

} // namespace ros2_dao
} // namespace dao
} // namespace kant

#endif
