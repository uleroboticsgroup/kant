
""" Mongo Dao Interface """

from abc import ABC, abstractmethod
from mongoengine import Document, disconnect, connect
from kant_dto import Dto


class MongoDao(ABC):
    """ Mongo Dao Abstract Class """

    def __init__(self, uri: str = "mongodb://localhost:27017/kant"):
        self.set_uri(uri)

    def connect(self):
        """ connect to current uri
        """

        disconnect()
        connect(host=self._uri)

    def get_uri(self) -> str:
        """ uri getter

        Returns:
            str: Mongo uri
        """

        return self._uri

    def set_uri(self, uri: str):
        """ uri setter

        Args:
            uri (str): Mongo uri
        """

        self._uri = uri

    @abstractmethod
    def _get_model(self, dto: Dto) -> Document:
        """ get the Mongoengine document corresponding to a give Dto

        Args:
            dto (Dto): Dto

        Returns:
            Document: Mongoengine document
        """

    @abstractmethod
    def _model_to_dto(self, pddl_mongoengine: Document) -> Dto:
        """ convert a Mongoengine document into a Dto

        Args:
            pddl_mongoengine (Document): Mongoengine document

        Returns:
            Dto: Dto
        """

    @abstractmethod
    def _dto_to_model(self, dto: Dto) -> Document:
        """ convert a Dto into a Mongoengine document

        Args:
            dto (Dto): Dto

        Returns:
            Document: Mongoengine document
        """

    @abstractmethod
    def _exist_in_mongo(self, dto: Dto) -> bool:
        """ check if Dto exists

        Args:
            dto (Dto): Dto

        Returns:
            bool: Dto exists?
        """
