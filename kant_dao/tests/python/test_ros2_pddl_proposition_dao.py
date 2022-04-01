
from test_dao_basic.test_pddl_proposition_dao import TestPddlPropositionDao
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)
from simple_node import Node
import rclpy


class TestRos2PddlPropositionDao(TestPddlPropositionDao):

    def setUp(self):
        super().setUp()

        rclpy.init()
        self.node = Node("test_kant_pddl_type_dao_node")
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.ROS2, node=self.node)

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()
        self.pddl_object_dao = dao_factory.create_pddl_object_dao()
        self.pddl_predicate_dao = dao_factory.create_pddl_predicate_dao()
        self.pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
        rclpy.shutdown()


del(TestPddlPropositionDao)
