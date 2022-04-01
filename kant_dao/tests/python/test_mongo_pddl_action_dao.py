
from test_dao_basic.test_pddl_action_dao import TestPddlActionDao
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)


class TestMongoPddlActionDao(TestPddlActionDao):

    def setUp(self):
        super().setUp()

        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO, uri="mongodb://localhost:27017/kant_tests")

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pdd_dao_predicate = dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = dao_factory.create_pddl_action_dao()


del(TestPddlActionDao)
