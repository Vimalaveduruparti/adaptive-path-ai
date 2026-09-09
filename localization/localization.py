from __future__ import annotations
import math, random
from dataclasses import dataclass
from typing import Optional
from interfaces import EgoState, WorldState
@dataclass
class SensorNoiseConfig:
    x_std: float=0.0; y_std: float=0.0; theta_std: float=0.0; v_std: float=0.0; seed: Optional[int]=None
def _coerce_cfg(c):
    if c is None:return SensorNoiseConfig()
    if isinstance(c,SensorNoiseConfig):return c
    if isinstance(c,dict):return SensorNoiseConfig(c.get('x_std',0.0),c.get('y_std',0.0),c.get('theta_std',0.0),c.get('v_std',0.0),c.get('seed'))
    raise TypeError('sensor_noise_cfg must be a SensorNoiseConfig, a dict, or None')
def _finite(v):
    try:return math.isfinite(float(v))
    except (TypeError,ValueError):return False
class LocalizationEstimator:
    def __init__(self,smoothing_alpha=0.6):
        if not 0.0<smoothing_alpha<=1.0: raise ValueError('smoothing_alpha must be in (0, 1]')
        self.smoothing_alpha=smoothing_alpha; self._filtered=None; self._rng=random.Random()
    def reset(self): self._filtered=None
    def localize(self,world_state,sensor_noise_cfg=None):
        cfg=_coerce_cfg(sensor_noise_cfg)
        if cfg.seed is not None:self._rng.seed(cfg.seed)
        raw=self._sanitize(self._measure(world_state,cfg)); self._filtered=self._smooth(raw); return self._filtered
    def _measure(self,ws,cfg):
        ego=getattr(ws,'ego',None)
        if ego is None:return EgoState(float('nan'),float('nan'),float('nan'),float('nan'))
        def n(v,s): return v+self._rng.gauss(0,s) if _finite(v) and s and s>0 else v
        return EgoState(n(ego.x,cfg.x_std),n(ego.y,cfg.y_std),n(ego.theta,cfg.theta_std),n(ego.v,cfg.v_std))
    def _sanitize(self,r):
        f=self._filtered or EgoState(0.0,0.0,0.0,0.0)
        return EgoState(r.x if _finite(r.x) else f.x,r.y if _finite(r.y) else f.y,r.theta if _finite(r.theta) else f.theta,r.v if _finite(r.v) else f.v)
    def _smooth(self,r):
        if self._filtered is None:return r
        a=self.smoothing_alpha;p=self._filtered
        px,py=math.cos(p.theta),math.sin(p.theta);rx,ry=math.cos(r.theta),math.sin(r.theta)
        return EgoState(a*r.x+(1-a)*p.x,a*r.y+(1-a)*p.y,math.atan2(a*ry+(1-a)*py,a*rx+(1-a)*px),a*r.v+(1-a)*p.v)
_default_estimator=LocalizationEstimator()
def localize(world_state,sensor_noise_cfg=None):return _default_estimator.localize(world_state,sensor_noise_cfg)
def reset_default_estimator():_default_estimator.reset()
