
""" Dao Familis Enumeration """

from enum import IntEnum, auto


class DaoFamilies(IntEnum):
    """ Enum Class of Dao Families """

    def _generate_next_value_(self, _start, count, _last_values):
        """Generate consecutive automatic numbers starting from zero."""
        return count

    ROS2 = auto()
    MONGO = auto()
