class InvalidInitVector(Exception):
    def __init__(self, message: str):
        super().__init__(message)


class InvalidInitVectorShape(InvalidInitVector):
    def __init__(self, message: str):
        super().__init__(message)


class InvalidInitVectorLength(InvalidInitVector):
    def __init__(self, message: str):
        super().__init__(message)


class InvalidInitVectorElements(InvalidInitVector):
    def __init__(self, message: str):
        super().__init__(message)


class PointOutOfReach(Exception):
    def __init__(self, message: str) -> None:
        super().__init__(message)
