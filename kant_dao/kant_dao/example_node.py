#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


""" DAO Example Node """

import rclpy

from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlActionDto
)

from simple_node import Node


class ExampleNode(Node):
    """ DAO Example Node Class """

    def __init__(self):

        super().__init__("example_node")

        dao_factory_method = DaoFactoryMethod()

        uri = "mongodb://localhost:27017/kant"
        dao_family = DaoFamilies.MONGO

        dao_factory = dao_factory_method.create_dao_factory(dao_family,
                                                            uri=uri,
                                                            node=self)

        # creating DAOs
        pddl_type_dao = dao_factory.create_pddl_type_dao()
        pddl_object_dao = dao_factory.create_pddl_object_dao()
        pddl_predicate_dao = dao_factory.create_pddl_predicate_dao()
        pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()
        pddl_action_dao = dao_factory.create_pddl_action_dao()

        # types
        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")

        # predicates
        robot_at = PddlPredicateDto(
            "robot_at", [robot_type, wp_type])

        # objects
        rb1 = PddlObjectDto(robot_type, "rb1")
        wp1 = PddlObjectDto(wp_type, "wp1")
        wp2 = PddlObjectDto(wp_type, "wp2")

        # propositions
        pddl_proposition_dto = PddlPropositionDto(robot_at, [rb1, wp1])
        pddl_goal_dto = PddlPropositionDto(robot_at, [rb1, wp2], is_goal=True)

        # actions
        r = PddlObjectDto(robot_type, "r")
        s = PddlObjectDto(wp_type, "s")
        d = PddlObjectDto(wp_type, "d")

        condition_1 = PddlConditionEffectDto(robot_at,
                                             [r, s],
                                             time=PddlConditionEffectDto.AT_START)

        effect_1 = PddlConditionEffectDto(robot_at,
                                          [r, s],
                                          time=PddlConditionEffectDto.AT_START,
                                          is_negative=True)

        effect_2 = PddlConditionEffectDto(robot_at,
                                          [r, d],
                                          time=PddlConditionEffectDto.AT_END)

        pddl_action_dto = PddlActionDto(
            "navigation", [r, s, d], [condition_1], [effect_1, effect_2])

        # saving all
        pddl_object_dao.save(rb1)
        pddl_object_dao.save(wp1)
        pddl_object_dao.save(wp2)

        pddl_proposition_dao.save(pddl_proposition_dto)
        pddl_proposition_dao.save(pddl_goal_dto)

        pddl_action_dao.save(pddl_action_dto)


def main(args=None):
    rclpy.init(args=args)

    node = ExampleNode()

    # node.join_spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
