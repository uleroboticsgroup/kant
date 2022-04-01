
from test_dao_basic.test_pddl_proposition_dao import TestPddlPropositionDao
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)


class TestMongoPddlPropositionDao(TestPddlPropositionDao):

    def setUp(self):
        super().setUp()
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO)

        dao_factory.set_uri("mongodb://localhost:27017/kant_tests")

        self.pddl_object_dao = dao_factory.create_pddl_object_dao()
        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pdd_dao_predicate = dao_factory.create_pddl_predicate_dao()
        self.pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()


del(TestPddlPropositionDao)
