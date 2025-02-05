import unittest
from kant_dao.dao_factory import DaoFactoryMethod, DaoFamilies
from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
)


class TestPddlPropositionDao(unittest.TestCase):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(DaoFamilies.MONGO)

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pddl_object_dao = dao_factory.create_pddl_object_dao()
        self.pddl_predicate_dao = dao_factory.create_pddl_predicate_dao()
        self.pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto("robot_at", [self._robot_type, self._wp_type])

        self._rb1 = PddlObjectDto(self._robot_type, "rb1")
        self._wp1 = PddlObjectDto(self._wp_type, "wp1")
        self.pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._rb1, self._wp1]
        )

    def tearDown(self):
        self.pddl_object_dao.delete_all()
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()
        self.pddl_proposition_dao.delete_all()

    def test_pddl_proposition_dao_save_true(self):
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertTrue(result)
        self.assertEqual(1, len(self.pddl_proposition_dao.get_all()))

    def test_pddl_proposition_dao_save_true_no_objects(self):
        self._robot_at = PddlPredicateDto("robot_at")
        self.pddl_proposition_dto = PddlPropositionDto(self._robot_at)
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertTrue(result)
        self.assertEqual(1, len(self.pddl_proposition_dao.get_all()))

    def test_pddl_proposition_dao_save_false_incorrect_proposition_types(self):
        self.pddl_proposition_dto.get_objects().reverse()
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dao_save_false_incorrect_proposition_len(self):
        self.pddl_proposition_dto.set_objects([])
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dao_save_false_proposition_already_exist(self):
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dao_get_by_predicate_empty(self):
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate("robot_at")
        self.assertEqual([], self.pddl_proposition_dto)

    def test_pddl_proposition_dao_get_by_predicate(self):
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)

        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate(
            "robot_at"
        )[0]
        self.assertEqual("(robot_at rb1 wp1)", str(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_get_goals(self):
        self.pddl_proposition_dto.set_is_goal(True)
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)", str(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_get_goals_empty(self):
        self.pddl_proposition_dto.set_is_goal(False)
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_goals()
        self.assertEqual(0, len(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_get_no_goals(self):
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_no_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)", str(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_get_no_goals_empty(self):
        self.pddl_proposition_dto.set_is_goal(True)
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_no_goals()
        self.assertEqual(0, len(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_get_all_0(self):
        pddl_proposition_dto_list = self.pddl_proposition_dao.get_all()
        self.assertEqual(0, len(pddl_proposition_dto_list))

    def test_pddl_proposition_dao_update_true(self):
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao._update(self.pddl_proposition_dto)
        self.assertTrue(result)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate(
            "robot_at"
        )[0]
        self.assertEqual("(robot_at rb1 wp1)", str(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_update_flase_proposition_not_exists(self):
        result = self.pddl_proposition_dao._update(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dao_update_false_incorrect_proposition_types(self):
        self.pddl_proposition_dto.get_objects().reverse()
        result = self.pddl_proposition_dao._update(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dao_save_save_true(self):
        result = self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        self.assertTrue(result)

    def test_pddl_proposition_dao_save_update_true(self):
        result = self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        self.assertTrue(result)

    def test_pddl_proposition_dao_delete_false_proposition_not_exist(self):
        result = self.pddl_proposition_dao.delete(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_proposition_dao_delete_true(self):
        self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao.delete(self.pddl_proposition_dto)
        self.assertTrue(result)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate("robot_at")
        self.assertEqual(0, len(self.pddl_proposition_dto))

    def test_pddl_proposition_dao_delete_all(self):
        self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao.delete_all()
        self.assertTrue(result)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate("robot_at")
        self.assertEqual(0, len(self.pddl_proposition_dto))
