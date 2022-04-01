
from test_dao_basic.test_pddl_type_dao import TestPddlTypeDao
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)
from simple_node import Node
import rclpy


class TestRos2PddlTypeDao(TestPddlTypeDao):

    def setUp(self):
        super().setUp()

        rclpy.init()
        self.node = Node("test_kant_pddl_type_dao_node")
        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.ROS2, node=self.node)

        self.pddl_type_dao = dao_factory.create_pddl_type_dao()

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
        rclpy.shutdown()


del(TestPddlTypeDao)
