class Position:
    def __init__(self, px: float = float('-inf'), py: float = float('-inf'), pz: float = float('-inf'),
                 prio: float = float('inf')):
        self._px = px
        self._py = py
        self._pz = pz

    @property
    def px(self):
        return self._px

    @property
    def py(self):
        return self._py

    @property
    def pz(self):
        return self._pz