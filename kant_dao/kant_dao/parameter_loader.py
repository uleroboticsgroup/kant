
""" Parameter Loader to manage the factory creation using ROS2 parameters """

from rclpy.node import Node
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies,
)
from kant_dao.dao_factory.dao_factories.dao_factory import DaoFactory


class ParameterLoader:
    """ DAO Parameter Loader Class """

    def __init__(self, node: Node):
        # param names
        dao_family_param_name = "dao_family"
        mongo_uri_param_name = "mongo_uri"

        # declaring params
        node.declare_parameter(dao_family_param_name,
                               DaoFamilies.ROS2)
        node.declare_parameter(mongo_uri_param_name,
                               "mongodb://localhost:27017/kant")

        # getting params
        dao_family = node.get_parameter(
            dao_family_param_name).get_parameter_value().integer_value
        mongo_uri = node.get_parameter(
            mongo_uri_param_name).get_parameter_value().string_value

        # creating dao factory
        dao_factory_method = DaoFactoryMethod()
        self.__dao_factory = dao_factory_method.create_dao_factory(
            dao_family, uri=mongo_uri, node=node)

    def get_dao_factory(self) -> DaoFactory:
        """ return the dao factory created in the constructor

        Returns:
            DaoFactory: dao family
        """

        return self.__dao_factory
