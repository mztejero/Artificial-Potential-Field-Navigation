from dataclasses import dataclass

@dataclass
class Dimensions:
    L: float = 3.0
    W: float = 2.0

@dataclass
class FrameParams:
    xmin: float = -25
    xmax: float = 25
    ymin: float = -25
    ymax: float = 25

@dataclass
class InitialConditions:
    v_long: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    vr: float = 0.0
    vl: float = 0.0

    v_ref: float = 12.0

@dataclass
class PIDParams:
    Kp_speed: float = 0.5
    Ki_speed: float = 0.0
    Kd_speed: float = 0.0

    Kp_steer: float = 6.0
    Ki_steer: float = 0.8
    Kd_steer: float = 0.01

    K_scalling: float = 4.0

@dataclass
class APFParams:
    d: float = 3.0

@dataclass
class Params:
    dimensions: Dimensions
    frame: FrameParams
    init: InitialConditions
    pid: PIDParams
    apf: APFParams

    dt: float = 0.1
