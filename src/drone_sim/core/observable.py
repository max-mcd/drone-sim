from typing import Callable, List


class Observable:
    def __init__(self):
        self._observers: List[Callable] = []

    def add_observer(self, observer: Callable) -> None:
        self._observers.append(observer)

    def notify(self, *args, **kwargs) -> None:
        for observer in self._observers:
            observer(*args, **kwargs) 