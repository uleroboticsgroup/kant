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


from test_dao_basic.test_pddl_object_dao import TestPddlObjectDao
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)


class TestMongoPddlObjectDao(TestPddlObjectDao):

    def setUp(self):
        super().setUp()

        dao_factory_method = DaoFactoryMethod()
        dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO)

        dao_factory.set_uri("mongodb://localhost:27017/kant_tests")

        self.pddl_object_dao = dao_factory.create_pddl_object_dao()
        self.pddl_type_dao = dao_factory.create_pddl_type_dao()


del (TestPddlObjectDao)
