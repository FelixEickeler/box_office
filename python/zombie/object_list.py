from pathlib import Path

from base_verification import BaseVerification

class PointCloudWrapper(BaseVerification):

    def __init__(self, pc):
        pass



class ObjectList(BaseVerification):

    def __init__(self, verification, objects):
        super().__init__(verification)
        self.__objects__ = objects

    def __len__(self):
        return len(self.__objects__)

    @classmethod
    def from_ol(cls, path):
        path = Path(path)
        objects = {}
        with open(path, 'r') as f:
            for line in f:
                nr, obj = line.strip().split(" ")
                objects[nr.strip()] = obj.strip()

        verification = objects.pop("#")
        return ObjectList(verification=verification, objects=objects)
