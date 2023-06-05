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


import unittest
from kant_dto.pddl_type_dto import PddlTypeDto
from kant_dto.pddl_predicate_dto import PddlPredicateDto
from kant_dto.pddl_proposition_dto import PddlPropositionDto
from kant_dto.pddl_object_dto import PddlObjectDto


class TestPddlPropositionDto(unittest.TestCase):

    def setUp(self):

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto(
            "robot_at", [self._robot_type, self._wp_type])

        self._rb1 = PddlObjectDto(self._robot_type, "rb1")
        self._wp1 = PddlObjectDto(self._wp_type, "wp1")
        self.pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._rb1, self._wp1])

    def test_pddl_proposition_dto_str(self):
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_proposition_dto))

    def test_pddl_proposition_dto_str_no_objects(self):
        self.pddl_proposition_dto.set_objects([])
        self.assertEqual("(robot_at)",
                         str(self.pddl_proposition_dto))

    def test_pddl_proposition_dto_get_predicate(self):
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_proposition_dto.get_predicate()))

    def test_pddl_proposition_dto_get_pddl_objects(self):
        pddl_objects_list = self.pddl_proposition_dto.get_objects()
        self.assertEqual("rb1",                         str(
            pddl_objects_list[0].get_name()))
        self.assertEqual("wp1",                         str(
            pddl_objects_list[1].get_name()))

    def test_pddl_proposition_dto_get_is_goal(self):
        self.assertFalse(self.pddl_proposition_dto.get_is_goal())

    def test_pddl_proposition_dto_eq_true(self):
        pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._rb1, self._wp1])
        result = (self.pddl_proposition_dto == pddl_proposition_dto)
        self.assertTrue(result)

    def test_pddl_proposition_dto_eq_false_bad_objects(self):
        pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._wp1, self._rb1])
        result = (self.pddl_proposition_dto == pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dto_eq_false_bad_len(self):
        pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._wp1])
        result = (self.pddl_proposition_dto == pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dto_eq_false_bad_predicate(self):
        _robot_at = PddlPredicateDto(
            "robot_on", [self._robot_type, self._wp_type])
        pddl_proposition_dto = PddlPropositionDto(
            _robot_at, [self._rb1, self._wp1])
        result = (self.pddl_proposition_dto == pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dto_eq_false_bad_instance(self):
        result = (self.pddl_proposition_dto == 10)
        self.assertFalse(result)
