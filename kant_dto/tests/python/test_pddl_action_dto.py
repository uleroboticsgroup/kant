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
from kant_dto.pddl_action_dto import PddlActionDto
from kant_dto.pddl_object_dto import PddlObjectDto
from kant_dto.pddl_condition_effect_dto import PddlConditionEffectDto


class TestPddlActionDto(unittest.TestCase):

    def setUp(self):

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto(
            "robot_at", [self._robot_type, self._wp_type])

        r = PddlObjectDto(self._robot_type, "r")
        s = PddlObjectDto(self._wp_type, "s")
        d = PddlObjectDto(self._wp_type, "d")

        self._condition_1 = PddlConditionEffectDto(self._robot_at,
                                                   [r, s],
                                                   time=PddlConditionEffectDto.AT_START)

        self._effect_1 = PddlConditionEffectDto(self._robot_at,
                                                [r, s],
                                                time=PddlConditionEffectDto.AT_START,
                                                is_negative=True)

        self._effect_2 = PddlConditionEffectDto(self._robot_at,
                                                [r, d],
                                                time=PddlConditionEffectDto.AT_END)

        self.pddl_action_dto = PddlActionDto(
            "navigation", [r, s, d], [self._condition_1], [self._effect_1, self._effect_2])

    def test_pddl_action_dto_str(self):
        self.maxDiff = None
        self.pddl_action_dto.set_durative(False)
        self._condition_1.set_time("")
        self._effect_1.set_time("")
        self._effect_2.set_time("")
        self.assertEqual("""\
(:action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:precondition (and
\t\t(robot_at ?r ?s)
\t)
\t:effect (and
\t\t(not (robot_at ?r ?s))
\t\t(robot_at ?r ?d)
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_action_dto_str_durative(self):
        self.maxDiff = None
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at start (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_action_dto_str_durative_no_effects(self):
        self.maxDiff = None
        self.pddl_action_dto.set_effects([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_action_dto_str_durative_no_conditions(self):
        self.maxDiff = None
        self.pddl_action_dto.set_conditions([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t)
\t:effect (and
\t\t(at start (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_action_dto_str_durative_no_parameters(self):
        self.maxDiff = None
        self.pddl_action_dto.set_conditions([])
        self.pddl_action_dto.set_effects([])
        self.pddl_action_dto.set_parameters([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ()
\t:duration (= ?duration 10)
\t:condition (and
\t)
\t:effect (and
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_action_dto_get_name(self):
        self.assertEqual("navigation", self.pddl_action_dto.get_name())

    def test_pddl_action_dto_get_prameters_list(self):
        params_list = self.pddl_action_dto.get_parameters()
        self.assertEqual("r - robot", str(params_list[0]))
        self.assertEqual("s - wp", str(params_list[1]))
        self.assertEqual("d - wp", str(params_list[2]))

    def test_pddl_action_dto_get_conditions(self):
        conditions_list = self.pddl_action_dto.get_conditions()
        self.assertEqual("(at start (robot_at ?r ?s))",
                         str(conditions_list[0]))

    def test_pddl_action_dto_get_effects(self):
        effects_list = self.pddl_action_dto.get_effects()
        self.assertEqual("(at start (not (robot_at ?r ?s)))",
                         str(effects_list[0]))
        self.assertEqual("(at end (robot_at ?r ?d))",
                         str(effects_list[1]))

    def test_pddl_action_dto_get_durative(self):
        self.assertTrue(self.pddl_action_dto.get_durative())

    def test_pddl_action_dto_get_duration(self):
        self.assertEqual(10, self.pddl_action_dto.get_duration())

    def test_pddl_action_dto_eq_true(self):
        pddl_action_dto = PddlActionDto("navigation")
        result = (self.pddl_action_dto == pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_action_dto_eq_false_bad_action_name(self):
        pddl_action_dto = PddlActionDto("other")
        result = (self.pddl_action_dto == pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dto_eq_false_bad_instance(self):
        result = (self.pddl_action_dto == 10)
        self.assertFalse(result)
