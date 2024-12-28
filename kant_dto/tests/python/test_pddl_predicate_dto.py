# Copyright (C) 2023 Miguel Ángel González Santamarta

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


import unittest
from kant_dto.pddl_type_dto import PddlTypeDto
from kant_dto.pddl_predicate_dto import PddlPredicateDto


class TestPddlPredicateDto(unittest.TestCase):

    def setUp(self):

        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")
        self.pddl_predicate_dto = PddlPredicateDto(
            "robot_at", [robot_type, wp_type])

    def test_pddl_predicate_dto_str(self):
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_predicate_dto))

    def test_pddl_predicate_dto_str_no_types(self):
        self.pddl_predicate_dto.set_types(None)
        self.assertEqual("(robot_at)",
                         str(self.pddl_predicate_dto))

    def test_pddl_predicate_dto_get_name(self):
        self.assertEqual("robot_at",
                         self.pddl_predicate_dto.get_name())

    def test_pddl_predicate_dto_get_types(self):
        pddl_types_list = self.pddl_predicate_dto.get_types()
        self.assertEqual("robot", pddl_types_list[0].get_name())
        self.assertEqual("wp", pddl_types_list[1].get_name())

    def test_pddl_predicate_dto_eq_true(self):
        pddl_predicate_dto = PddlPredicateDto("robot_at", [])
        result = (self.pddl_predicate_dto == pddl_predicate_dto)
        self.assertTrue(result)

    def test_pddl_predicate_dto_eq_false_bad_predicate_name(self):
        pddl_predicate_dto = PddlPredicateDto("robot_on", [])
        result = (self.pddl_predicate_dto == pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_predicate_dto_eq_false_bad_instance(self):
        result = (self.pddl_predicate_dto == 10)
        self.assertFalse(result)
