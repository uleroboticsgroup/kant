import unittest
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)
from kant_dto import (
    PddlTypeDto,
    PddlObjectDto
)


class TestPddlObjectDao(unittest.TestCase):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO)

        self.pddl_object_dao = dao_factory.create_pddl_object_dao()

        pddl_type_dto = PddlTypeDto("robot")
        self.pddl_object_dto = PddlObjectDto(pddl_type_dto, "rb1")

    def tearDown(self):
        self.pddl_object_dao.delete_all()

    def test_pddl_object_dao_save_true(self):
        result = self.pddl_object_dao._save(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_object_dao_save_false_object_already_exist(self):
        result = self.pddl_object_dao._save(self.pddl_object_dto)
        result = self.pddl_object_dao._save(self.pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_object_dao_get_none(self):
        self.pddl_object_dto = self.pddl_object_dao.get("rb1")
        self.assertIsNone(self.pddl_object_dto)

    def test_pddl_object_dao_get(self):
        self.pddl_object_dao._save(self.pddl_object_dto)
        self.pddl_object_dto = self.pddl_object_dao.get("rb1")
        self.assertEqual("rb1 - robot", str(self.pddl_object_dto))

    def test_pddl_object_dao_get_all_0(self):
        pddl_object_dto_list = self.pddl_object_dao.get_all()
        self.assertEqual(0, len(pddl_object_dto_list))

    def test_pddl_object_dao_update_true(self):
        self.pddl_object_dao._save(self.pddl_object_dto)
        result = self.pddl_object_dao._update(self.pddl_object_dto)
        self.assertTrue(result)
        self.pddl_object_dto = self.pddl_object_dao.get("rb1")
        self.assertEqual("rb1 - robot", str(self.pddl_object_dto))

    def test_pddl_object_dao_update_flase(self):
        result = self.pddl_object_dao._update(self.pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_object_dao_save_save_true(self):
        result = self.pddl_object_dao.save(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_object_dao_save_true(self):
        self.pddl_object_dao._save(self.pddl_object_dto)
        result = self.pddl_object_dao.save(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_object_dao_delete_false_object_not_exist(self):
        result = self.pddl_object_dao.delete(self.pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_object_dao_delete_true(self):
        self.pddl_object_dao.save(self.pddl_object_dto)
        result = self.pddl_object_dao.delete(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_object_dao_delete_all(self):
        self.pddl_object_dao.save(self.pddl_object_dto)
        result = self.pddl_object_dao.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_object_dao.get_all()))
