import unittest
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)
from kant_dto import (
    PddlTypeDto,
    PddlPredicateDto
)


class TestPddlPredicateDao(unittest.TestCase):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO)

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = dao_factory.create_pddl_predicate_dao()

        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")
        self.pddl_predicate_dto = PddlPredicateDto(
            "robot_at", [robot_type, wp_type])

    def tearDown(self):
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()

    def test_pddl_predicate_dao_save_true(self):
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.assertTrue(result)
        self.assertEqual(1, len(self.pddl_predicate_dao.get_all()))

    def test_pddl_predicate_dao_save_true_no_types(self):
        self.pddl_predicate_dto = PddlPredicateDto("robot_at")
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.assertTrue(result)
        self.assertEqual(1, len(self.pddl_predicate_dao.get_all()))

    def test_pddl_predicate_dao_save_false_predicate_already_exist(self):
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_predicate_dao_get_none(self):
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertIsNone(self.pddl_predicate_dto)

    def test_pddl_predicate_dao_get(self):
        self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_predicate_dto))

    def test_pddl_predicate_dao_get_all_0(self):
        pddl_predicate_dto_list = self.pddl_predicate_dao.get_all()
        self.assertEqual(0, len(pddl_predicate_dto_list))

    def test_pddl_predicate_dao_update_true(self):
        self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao._update(self.pddl_predicate_dto)
        self.assertTrue(result)
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_predicate_dto))

    def test_pddl_predicate_dao_update_flase(self):
        result = self.pddl_predicate_dao._update(self.pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_predicate_dao_save_save_true(self):
        result = self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        self.assertTrue(result)

    def test_pddl_predicate_dao_save_update_true(self):
        self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        self.assertTrue(result)

    def test_pddl_predicate_dao_delete_false_predicate_not_exist(self):
        result = self.pddl_predicate_dao.delete(self.pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_predicate_dao_delete_true(self):
        self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao.delete(self.pddl_predicate_dto)
        self.assertTrue(result)
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertIsNone(self.pddl_predicate_dto)

    def test_pddl_predicate_dao_delete_all(self):
        self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_predicate_dao.get_all()))
