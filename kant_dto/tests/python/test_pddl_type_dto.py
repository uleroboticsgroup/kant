import unittest
from kant_dto.pddl_type_dto import PddlTypeDto


class TestPddlTypeDto(unittest.TestCase):

    def setUp(self):

        self.pddl_type_dto = PddlTypeDto("robot")

    def test_pddl_type_dto_str(self):
        self.assertEqual("robot", str(self.pddl_type_dto))

    def test_pddl_type_dto_get_name(self):
        self.assertEqual("robot", self.pddl_type_dto.get_name())

    def test_pddl_type_dto_eq_true(self):
        pddl_type_dto = PddlTypeDto("robot")
        result = (self.pddl_type_dto == pddl_type_dto)
        self.assertTrue(result)

    def test_pddl_type_dto_eq_false_bad_type_name(self):
        pddl_type_dto = PddlTypeDto("wp")
        result = (self.pddl_type_dto == pddl_type_dto)
        self.assertFalse(result)

    def test_pddl_type_dto_eq_false_bad_instance(self):
        result = (self.pddl_type_dto == 10)
        self.assertFalse(result)
