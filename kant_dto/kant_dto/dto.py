
""" Dto Abstract Class """

from abc import ABC, abstractmethod


class Dto(ABC):
    """ Dto Abstract Class """

    @abstractmethod
    def __str__(self):
        return "Dto abstract class"

    @abstractmethod
    def __eq__(self, other) -> bool:
        return False
