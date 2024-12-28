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


class TestPddlTypeDto(unittest.TestCase):

    def setUp(self):

        self.pddl_type_dto = PddlTypeDto("robot")

    def test_pddl_type_dto_str(self):
        self.assertEqual("robot", str(self.pddl_type_dto))

    def test_pddl_type_dto_get_name(self):
        self.assertEqual("robot", self.pddl_type_dto.get_name())

    def test_pddl_type_dto_eq_true(self):
        pddl_type_dto = PddlTypeDto("robot")
        result = (self.pddl_type_dto == pddl_type_dto)
        self.assertTrue(result)

    def test_pddl_type_dto_eq_false_bad_type_name(self):
        pddl_type_dto = PddlTypeDto("wp")
        result = (self.pddl_type_dto == pddl_type_dto)
        self.assertFalse(result)

    def test_pddl_type_dto_eq_false_bad_instance(self):
        result = (self.pddl_type_dto == 10)
        self.assertFalse(result)
