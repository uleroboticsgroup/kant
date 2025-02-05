# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""Dao Factory of Factories"""

from kant_dao.dao_factory.dao_families import DaoFamilies

from kant_dao.dao_factory.dao_factories import DaoFactory, Ros2DaoFactory, MongoDaoFactory


class DaoFactoryMethod:
    """Dao Factory of Factories Class"""

    def __init__(self):
        self.dao_families = DaoFamilies
        self.__families_to_factory = {
            self.dao_families.ROS2: Ros2DaoFactory,
            self.dao_families.MONGO: MongoDaoFactory,
        }

    def create_dao_factory(self, family: int, **kwargs) -> DaoFactory:
        """create a pddl dao factory of a given family

        Args:
            family (int): number of the pddl dao family to create

        Returns:
            DaoFactory: pddl dao factory
        """

        args_dict = {}

        dao_factory = self.__families_to_factory[family]
        init_args = list(dao_factory.__init__.__code__.co_varnames)

        for key, value in kwargs.items():
            if key in init_args:
                args_dict[key] = value

        return dao_factory(**args_dict)
