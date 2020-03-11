import json
from dataclasses import dataclass


class DataPacket:
    """ Base class for custom JSON serializable data containers. Derive from this class and add attributes as needed.
    Use to_bytes to send over a socket. """

    def to_dict(self) -> dict:
        """ Converts the object into a dictionary of attributes. The dict includes the name of the class"""

        #  Populate the dictionary with object meta data:
        obj_dict = {'__class__': self.__class__.__name__}

        #  Populate the dictionary with object properties:
        obj_dict.update(self.__dict__)

        return obj_dict

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    def to_bytes(self) -> bytes:
        return bytes(self.to_json(), 'utf-8')


@dataclass
class StateUpdate(DataPacket):
    position: list
    attitude: list
