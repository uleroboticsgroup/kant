import unittest
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)
from kant_dto import PddlTypeDto


class TestPddlTypeDao(unittest.TestCase):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO)

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pddl_type_dto = PddlTypeDto("robot")

    def tearDown(self):
        self.pddl_type_dao.delete_all()

    def test_pddl_type_dao_save_true(self):
        result = self.pddl_type_dao._save(self.pddl_type_dto)
        self.assertTrue(result)

    def test_pddl_type_dao_save_false_type_already_exist(self):
        result = self.pddl_type_dao._save(self.pddl_type_dto)
        result = self.pddl_type_dao._save(self.pddl_type_dto)
        self.assertFalse(result)

    def test_pddl_type_dao_get_none(self):
        self.pddl_type_dto = self.pddl_type_dao.get("robot")
        self.assertIsNone(self.pddl_type_dto)

    def test_pddl_type_dao_get(self):
        self.pddl_type_dao.save(self.pddl_type_dto)
        self.pddl_type_dto = self.pddl_type_dao.get("robot")
        self.assertEqual("robot", str(self.pddl_type_dto))

    def test_pddl_type_dao_get_all_0(self):
        pddl_type_dto_list = self.pddl_type_dao.get_all()
        self.assertEqual(0, len(pddl_type_dto_list))

    def test_pddl_type_dao_update_true(self):
        self.pddl_type_dao._save(self.pddl_type_dto)
        result = self.pddl_type_dao._update(self.pddl_type_dto)
        self.assertTrue(result)
        self.pddl_type_dto = self.pddl_type_dao.get("robot")
        self.assertEqual("robot", str(self.pddl_type_dto))

    def test_pddl_type_dao_update_flase(self):
        result = self.pddl_type_dao._update(self.pddl_type_dto)
        self.assertFalse(result)

    def test_pddl_type_dao_save_save_true(self):
        result = self.pddl_type_dao.save(self.pddl_type_dto)
        self.assertTrue(result)

    def test_pddl_type_dao_save_update_true(self):
        self.pddl_type_dao.save(self.pddl_type_dto)
        result = self.pddl_type_dao.save(self.pddl_type_dto)
        self.assertTrue(result)

    def test_pddl_type_dao_delete_false_type_not_exist(self):
        result = self.pddl_type_dao.delete(self.pddl_type_dto)
        self.assertFalse(result)

    def test_pddl_type_dao_delete_true(self):
        self.pddl_type_dao.save(self.pddl_type_dto)
        result = self.pddl_type_dao.delete(self.pddl_type_dto)
        self.assertTrue(result)
        self.pddl_type_dto = self.pddl_type_dao.get("robot")
        self.assertIsNone(self.pddl_type_dto)

    def test_pddl_type_dao_delete_all(self):
        self.pddl_type_dao.save(self.pddl_type_dto)
        result = self.pddl_type_dao.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_type_dao.get_all()))


# if __name__ == "__main__":
    #cov = coverage.Coverage()
    # cov.start()
    # suite = unittest.TestLoader().loadTestsFromTestCase(TestPddlDaoType)
    # unittest.TextTestRunner(verbosity=2).run(suite)
    # cov.stop()
    # cov.save()
    # cov.html_report()
    # cov.report()
