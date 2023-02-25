from trajectory_generation.Position import Position


class Waypoint:
    def __init__(self, px, py, pz, vx, vy, vz):
        self._position = Position(px=px, py=py, pz=pz)

        self._vx = vx
        self._vy = vy
        self._vz = vz

    @property
    def px(self):
        return self._position.px

    @property
    def py(self):
        return self._position.py

    @property
    def pz(self):
        return self._position.pz

    @property
    def vx(self):
        return self._vx

    @property
    def vy(self):
        return self._vy

    @property
    def vz(self):
        return self._vz
