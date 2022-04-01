
import unittest
from kant_knowledge_base.parser import DtoMsgParser

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlActionDto
)


class TestDtoMsgParser(unittest.TestCase):

    def setUp(self):

        self.parser = DtoMsgParser()

        # types
        self.robot_type = PddlTypeDto("robot")
        self.wp_type = PddlTypeDto("wp")

        # objects
        self.rb1 = PddlObjectDto(self.robot_type, "rb1")
        self.wp1 = PddlObjectDto(self.wp_type, "wp1")
        self.wp2 = PddlObjectDto(self.wp_type, "wp2")

        # predicates
        self.robot_at = PddlPredicateDto(
            "robot_at", [self.robot_type, self.wp_type])
        self.wp_checked = PddlPredicateDto(
            "wp_checked", [self.robot_type, self.wp_type])

        # propositions
        self.rb1_robot_at = PddlPropositionDto(
            self.robot_at, [self.rb1, self.wp1])

        # goals
        self.rb1_wp2_wp_checked_goal = PddlPropositionDto(
            self.wp_checked, [self.rb1, self.wp2], is_goal=True)

        # actions
        r = PddlObjectDto(self.robot_type, "r")
        s = PddlObjectDto(self.wp_type, "s")
        d = PddlObjectDto(self.wp_type, "d")

        condition_1 = PddlConditionEffectDto(self.robot_at,
                                             [r, s],
                                             time=PddlConditionEffectDto.AT_START)

        effect_1 = PddlConditionEffectDto(self.robot_at,
                                          [r, s],
                                          time=PddlConditionEffectDto.AT_START,
                                          is_negative=True)

        effect_2 = PddlConditionEffectDto(self.robot_at,
                                          [r, d],
                                          time=PddlConditionEffectDto.AT_END)

        self.navigation_action = PddlActionDto(
            "navigation", [r, s, d], [condition_1], [effect_1, effect_2])

    def test_parse_type(self):
        msg = self.parser.type_dto_to_msg(self.robot_type)
        self.assertEqual("robot", msg.name)

    def test_parse_object(self):
        msg = self.parser.object_dto_to_msg(self.rb1)
        self.assertEqual("rb1", msg.name)
        self.assertEqual("robot", msg.type.name)

    def test_parse_predicate(self):
        msg = self.parser.predicate_dto_to_msg(self.robot_at)
        self.assertEqual("robot_at", msg.name)
        self.assertEqual("robot", msg.types[0].name)
        self.assertEqual("wp", msg.types[1].name)

    def test_parse_proposition(self):
        msg = self.parser.proposition_dto_to_msg(self.rb1_robot_at)
        self.assertEqual("robot_at", msg.predicate.name)
        self.assertEqual("rb1", msg.objects[0].name)
        self.assertEqual("wp1", msg.objects[1].name)
        self.assertFalse(msg.is_goal)

    def test_parse_proposition_goal(self):
        msg = self.parser.proposition_dto_to_msg(self.rb1_wp2_wp_checked_goal)
        self.assertEqual("wp_checked", msg.predicate.name)
        self.assertEqual("rb1", msg.objects[0].name)
        self.assertEqual("wp2", msg.objects[1].name)
        self.assertTrue(msg.is_goal)

    def test_parse_durative_action(self):
        msg = self.parser.action_dto_to_msg(self.navigation_action)
        self.assertEqual("navigation", msg.name)
        self.assertTrue(msg.durative)
        self.assertEqual(10, msg.duration)

        self.assertEqual("robot", msg.parameters[0].type.name)
        self.assertEqual("r", msg.parameters[0].name)
        self.assertEqual("robot", msg.parameters[0].type.name)
        self.assertEqual("s", msg.parameters[1].name)
        self.assertEqual("wp", msg.parameters[1].type.name)
        self.assertEqual("d", msg.parameters[2].name)
        self.assertEqual("wp", msg.parameters[2].type.name)

        self.assertEqual(
            "robot_at", msg.coditions[0].predicate.name)
        self.assertFalse(msg.coditions[0].is_negative)
        self.assertEqual(
            "r", msg.coditions[0].objects[0].name)
        self.assertEqual(
            "robot", msg.coditions[0].objects[0].type.name)
        self.assertEqual(
            "s", msg.coditions[0].objects[1].name)
        self.assertEqual(
            "wp", msg.coditions[0].objects[1].type.name)
        self.assertEqual("at start", msg.coditions[0].time)

        self.assertEqual(
            "robot_at", msg.effects[0].predicate.name)
        self.assertTrue(msg.effects[0].is_negative)
        self.assertEqual(
            "r", msg.effects[0].objects[0].name)
        self.assertEqual(
            "robot", msg.effects[0].objects[0].type.name)
        self.assertEqual(
            "s", msg.effects[0].objects[1].name)
        self.assertEqual(
            "wp", msg.effects[0].objects[1].type.name)
        self.assertEqual("at start", msg.effects[0].time)

        self.assertEqual(
            "robot_at", msg.effects[1].predicate.name)
        self.assertFalse(msg.effects[1].is_negative)
        self.assertEqual(
            "r", msg.effects[1].objects[0].name)
        self.assertEqual(
            "robot", msg.effects[1].objects[0].type.name)
        self.assertEqual(
            "d", msg.effects[1].objects[1].name)
        self.assertEqual(
            "wp", msg.effects[1].objects[1].type.name)
        self.assertEqual("at end", msg.effects[1].time)

    def test_parse_action(self):

        self.navigation_action.set_durative(False)
        self.navigation_action.get_conditions()[0].set_time("")
        self.navigation_action.get_effects()[0].set_time("")
        self.navigation_action.get_effects()[1].set_time("")

        msg = self.parser.action_dto_to_msg(self.navigation_action)
        self.assertEqual("navigation", msg.name)
        self.assertFalse(msg.durative)

        self.assertEqual("robot", msg.parameters[0].type.name)
        self.assertEqual("r", msg.parameters[0].name)
        self.assertEqual("robot", msg.parameters[0].type.name)
        self.assertEqual("s", msg.parameters[1].name)
        self.assertEqual("wp", msg.parameters[1].type.name)
        self.assertEqual("d", msg.parameters[2].name)
        self.assertEqual("wp", msg.parameters[2].type.name)

        self.assertEqual(
            "robot_at", msg.coditions[0].predicate.name)
        self.assertFalse(msg.coditions[0].is_negative)
        self.assertEqual(
            "r", msg.coditions[0].objects[0].name)
        self.assertEqual(
            "robot", msg.coditions[0].objects[0].type.name)
        self.assertEqual(
            "s", msg.coditions[0].objects[1].name)
        self.assertEqual(
            "wp", msg.coditions[0].objects[1].type.name)
        self.assertEqual("", msg.coditions[0].time)

        self.assertEqual(
            "robot_at", msg.effects[0].predicate.name)
        self.assertTrue(msg.effects[0].is_negative)
        self.assertEqual(
            "r", msg.effects[0].objects[0].name)
        self.assertEqual(
            "robot", msg.effects[0].objects[0].type.name)
        self.assertEqual(
            "s", msg.effects[0].objects[1].name)
        self.assertEqual(
            "wp", msg.effects[0].objects[1].type.name)
        self.assertEqual("", msg.effects[0].time)

        self.assertEqual(
            "robot_at", msg.effects[1].predicate.name)
        self.assertFalse(msg.effects[1].is_negative)
        self.assertEqual(
            "r", msg.effects[1].objects[0].name)
        self.assertEqual(
            "robot", msg.effects[1].objects[0].type.name)
        self.assertEqual(
            "d", msg.effects[1].objects[1].name)
        self.assertEqual(
            "wp", msg.effects[1].objects[1].type.name)
        self.assertEqual("", msg.effects[1].time)
