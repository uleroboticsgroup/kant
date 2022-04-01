
from test_dao_basic.test_pddl_type_dao import TestPddlTypeDao
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)


class TestMongoPddlTypeDao(TestPddlTypeDao):

    def setUp(self):
        super().setUp()
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO)

        dao_factory.set_uri("mongodb://localhost:27017/kant_tests")

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()


del(TestPddlTypeDao)
