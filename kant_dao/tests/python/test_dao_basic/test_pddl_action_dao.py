import unittest
from kant_dao.dao_factory import DaoFactoryMethod, DaoFamilies

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlConditionEffectDto,
    PddlActionDto,
)


class TestPddlActionDao(unittest.TestCase):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(DaoFamilies.MONGO)

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = dao_factory.create_pddl_action_dao()

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto("robot_at", [self._robot_type, self._wp_type])

        r = PddlObjectDto(self._robot_type, "r")
        s = PddlObjectDto(self._wp_type, "s")
        d = PddlObjectDto(self._wp_type, "d")

        self._condition_1 = PddlConditionEffectDto(
            self._robot_at, [r, s], time=PddlConditionEffectDto.AT_START
        )

        self._effect_1 = PddlConditionEffectDto(
            self._robot_at, [r, s], time=PddlConditionEffectDto.AT_START, is_negative=True
        )

        self._effect_2 = PddlConditionEffectDto(
            self._robot_at, [r, d], time=PddlConditionEffectDto.AT_END
        )

        self.pddl_action_dto = PddlActionDto(
            "navigation", [r, s, d], [self._condition_1], [self._effect_1, self._effect_2]
        )

    def tearDown(self):
        self.pddl_action_dao.delete_all()
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()

    def test_pddl_action_dao_save_true(self):
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_action_dao_save_false_incorrect_condition_types(self):
        self.pddl_action_dto.get_conditions()[0].get_objects().reverse()
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_incorrect_condition_len(self):
        self.pddl_action_dto.get_conditions()[0].set_objects([])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_durative_condition_no_time(self):
        self.pddl_action_dto.get_conditions()[0].set_time("")
        self.pddl_action_dto.set_effects([])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_durative_effect_no_time(self):
        self.pddl_action_dto.get_effects()[1].set_time("")
        self.pddl_action_dto.set_conditions([])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_no_durative_condition_time(self):
        self.pddl_action_dto.set_durative(False)
        self.pddl_action_dto.set_effects([])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_no_durative_effect_time(self):
        self.pddl_action_dto.set_durative(False)
        self.pddl_action_dto.set_conditions([])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_durative_condition_bad_parameter(self):
        r = PddlObjectDto(self._robot_type, "a")
        s = PddlObjectDto(self._wp_type, "s")
        d = PddlObjectDto(self._wp_type, "d")
        self.pddl_action_dto.set_parameters([r, s, d])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_durative_effect_bad_parameter(self):
        r = PddlObjectDto(self._robot_type, "r")
        s = PddlObjectDto(self._wp_type, "s")
        d = PddlObjectDto(self._wp_type, "a")
        self.pddl_action_dto.set_parameters([r, s, d])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_false_action_already_exist(self):
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_get_none(self):
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertIsNone(self.pddl_action_dto)

    def test_pddl_action_dao_get(self):
        self.pddl_action_dao._save(self.pddl_action_dto)

        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertEqual(
            """\
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
            str(self.pddl_action_dto),
        )

    def test_pddl_action_dao_get_all_0(self):
        pddl_action_dto_list = self.pddl_action_dao.get_all()
        self.assertEqual(0, len(pddl_action_dto_list))

    def test_pddl_action_dao_update_true(self):
        self.pddl_action_dao._save(self.pddl_action_dto)
        self.pddl_action_dto.get_effects()[0].set_time(PddlConditionEffectDto.AT_END)
        result = self.pddl_action_dao._update(self.pddl_action_dto)
        self.assertTrue(result)
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertEqual(
            """\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at end (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
            str(self.pddl_action_dto),
        )

    def test_pddl_action_dao_update_flase(self):
        result = self.pddl_action_dao._update(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_save_save_true(self):
        result = self.pddl_action_dao.save(self.pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_action_dao_save_update_true(self):
        result = self.pddl_action_dao.save(self.pddl_action_dto)
        result = self.pddl_action_dao.save(self.pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_action_dao_normal_action(self):
        self.pddl_action_dto.set_durative(False)
        self._condition_1.set_time("")
        self._effect_1.set_time("")
        self._effect_2.set_time("")

        self.pddl_action_dao._save(self.pddl_action_dto)
        result = self.pddl_action_dao._update(self.pddl_action_dto)
        self.assertTrue(result)

        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertEqual(
            """\
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
            str(self.pddl_action_dto),
        )

    def test_pddl_action_dao_delete_false_action_not_exist(self):
        result = self.pddl_action_dao.delete(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_action_dao_delete_true(self):
        self.pddl_action_dao.save(self.pddl_action_dto)
        result = self.pddl_action_dao.delete(self.pddl_action_dto)
        self.assertTrue(result)
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertIsNone(self.pddl_action_dto)

    def test_pddl_action_dao_delete_all(self):
        self.pddl_action_dao.save(self.pddl_action_dto)
        result = self.pddl_action_dao.delete_all()
        self.assertTrue(result)
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertIsNone(self.pddl_action_dto)
