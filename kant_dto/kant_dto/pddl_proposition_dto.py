
""" Pddl Proposition Dto """

from typing import List
from kant_dto.dto import Dto
from kant_dto.pddl_predicate_dto import PddlPredicateDto
from kant_dto.pddl_object_dto import PddlObjectDto


class PddlPropositionDto(Dto):
    """ Pddl Proposition Dto Class """

    def __init__(self, predicate: PddlPredicateDto,
                 objects: List[PddlObjectDto] = None,
                 is_goal: bool = False):

        self.set_predicate(predicate)
        self.set_objects(objects)
        self.set_is_goal(is_goal)

        Dto.__init__(self)

    def get_is_goal(self) -> bool:
        """ is goal getter

        Returns:
            bool: is this proposition a goal
        """

        return self._is_goal

    def set_is_goal(self, is_goal: bool):
        """ is goal setter

        Args:
            is_goal (bool): is this proposition a goal
        """

        self._is_goal = is_goal

    def get_predicate(self) -> PddlPredicateDto:
        """ pddl predicate getter

        Returns:
            PddlPredicateDto: pddl predicate
        """

        return self._predicate

    def set_predicate(self, predicate: PddlPredicateDto):
        """ pddl predicate setter

        Args:
            predicate (PddlPredicateDto): pddl predicate
        """

        self._predicate = predicate

    def get_objects(self) -> List[PddlObjectDto]:
        """ pddl objects list getter

        Returns:
            List[PddlObjectDto]: list of pddl objects
        """

        return self._objects

    def set_objects(self, objects: List[PddlObjectDto]):
        """ pddl objects list setter

        Args:
            objects (List[PddlObjectDto]): list of pddl objects
        """

        if objects:
            self._objects = objects
        else:
            self._objects = []

    def __str__(self):
        string = "(" + self._predicate.get_name()

        for pddl_object in self._objects:
            string += " " + pddl_object.get_name()

        string += ")"

        return string

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlPropositionDto):

            if not other.get_predicate() == self.get_predicate():
                return False

            if not len(other.get_objects()) == len(self.get_objects()):
                return False

            for pddl_object, other_pddl_object in zip(self.get_objects(),
                                                      other.get_objects()):
                if not pddl_object == other_pddl_object:
                    return False

            return True

        return False
