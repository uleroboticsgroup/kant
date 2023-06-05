// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <iostream>
#include <memory>

#include "simple_node/node.hpp"

#include "kant_dto/pddl_action_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

#include "kant_dao/dao_factory/dao_factory_method.hpp"

using namespace kant::dto;

class ExampleNode : public simple_node::Node {

public:
  ExampleNode() : simple_node::Node("example_node") {

    kant::dao::dao_factory::DaoFactoryMethod dao_factory_method;

    auto dao_factory = dao_factory_method.create_dao_factory(
        kant::dao::dao_factory::DaoFamilies::MONGO, this,
        "mongodb://localhost:27017/kant");

    // creating DAOs
    // auto *pddl_type_dao = dao_factory->create_pddl_type_dao();
    auto *pddl_object_dao = dao_factory->create_pddl_object_dao();
    // auto *pddl_predicate_dao = dao_factory->create_pddl_predicate_dao();
    auto pddl_proposition_dao = dao_factory->create_pddl_proposition_dao();
    auto *pddl_action_dao = dao_factory->create_pddl_action_dao();

    // types
    auto robot_type = std::make_shared<PddlTypeDto>(PddlTypeDto("robot"));
    auto wp_type = std::make_shared<PddlTypeDto>(PddlTypeDto("wp"));

    // predicates
    auto robot_at = std::make_shared<PddlPredicateDto>(
        PddlPredicateDto("robot_at", {robot_type, wp_type}));

    // objects
    auto rb1 =
        std::make_shared<PddlObjectDto>(PddlObjectDto(robot_type, "rb1"));
    auto wp1 = std::make_shared<PddlObjectDto>(PddlObjectDto(wp_type, "wp1"));
    auto wp2 = std::make_shared<PddlObjectDto>(PddlObjectDto(wp_type, "wp2"));

    // propositions
    auto pddl_proposition_dto = std::make_shared<PddlPropositionDto>(
        PddlPropositionDto(robot_at, {rb1, wp1}));
    auto pddl_goal_dto = std::make_shared<PddlPropositionDto>(
        PddlPropositionDto(robot_at, {rb1, wp2}, true));

    // actions
    std::shared_ptr<PddlObjectDto> r =
        std::make_shared<PddlObjectDto>(PddlObjectDto(robot_type, "r"));

    std::shared_ptr<PddlObjectDto> s =
        std::make_shared<PddlObjectDto>(PddlObjectDto(wp_type, "s"));

    std::shared_ptr<PddlObjectDto> d =
        std::make_shared<PddlObjectDto>(PddlObjectDto(wp_type, "d"));

    std::shared_ptr<PddlConditionEffectDto> condition_1 =
        std::make_shared<PddlConditionEffectDto>(
            PddlConditionEffectDto(robot_at, {r, s}, AT_START));

    std::shared_ptr<PddlConditionEffectDto> effect_1 =
        std::make_shared<PddlConditionEffectDto>(
            PddlConditionEffectDto(robot_at, {r, s}, true, AT_END));

    std::shared_ptr<PddlConditionEffectDto> effect_2 =
        std::make_shared<PddlConditionEffectDto>(
            PddlConditionEffectDto(robot_at, {r, d}, AT_END));

    std::shared_ptr<PddlActionDto> pddl_action_dto =
        std::make_shared<PddlActionDto>(
            PddlActionDto("navigation", {r, s, d}, {condition_1},
                          {effect_1, effect_2}, true));

    // saving all
    pddl_object_dao->save_update(rb1);
    pddl_object_dao->save_update(wp1);
    pddl_object_dao->save_update(wp2);

    pddl_proposition_dao->save_update(pddl_proposition_dto);
    pddl_proposition_dao->save_update(pddl_goal_dto);

    pddl_action_dao->save_update(pddl_action_dto);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::shutdown();
  return 0;
}