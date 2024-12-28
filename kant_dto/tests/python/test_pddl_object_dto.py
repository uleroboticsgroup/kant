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
from kant_dto.pddl_object_dto import PddlObjectDto


class TestPddlObjectDto(unittest.TestCase):

    def setUp(self):

        self.pddl_type_dto = PddlTypeDto("robot")
        self.pddl_object_dto = PddlObjectDto(self.pddl_type_dto, "rb1")

    def test_pddl_object_dto_str(self):
        self.assertEqual("rb1 - robot", str(self.pddl_object_dto))

    def test_pddl_type_dto_get_type(self):
        self.assertEqual("robot", str(self.pddl_object_dto.get_type()))

    def test_pddl_type_dto_get_name(self):
        self.assertEqual("rb1", self.pddl_object_dto.get_name())

    def test_pddl_object_dto_eq_true(self):
        pddl_object_dto = PddlObjectDto(self.pddl_type_dto, "rb1")
        result = (self.pddl_object_dto == pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_object_dto_eq_false_bad_object_name(self):
        pddl_object_dto = PddlObjectDto(self.pddl_type_dto, "rb2")
        result = (self.pddl_object_dto == pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_type_dto_eq_false_bad_instance(self):
        result = (self.pddl_object_dto == 10)
        self.assertFalse(result)
