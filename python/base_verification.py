class BaseVerification:
    def __init__(self, verification):
        self.__verification__ = verification

    def verify(self, other):
        if isinstance(other, BaseVerification):
            if self.__verification__ == other.__verification__:
                return True
        else:
            if self.__verification__ == other.__str__:
                return True
        return False