import math, sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from interfaces import EgoState, WorldState
from localization.localization import LocalizationEstimator, SensorNoiseConfig
def make_world_state(x=0.0,y=0.0,theta=0.0,v=0.0,timestamp=0.0): return WorldState(ego=EgoState(x,y,theta,v),obstacles=[],road_boundary=[],timestamp=timestamp)
def test_position_estimation_reasonable():
 r=LocalizationEstimator().localize(make_world_state(10,5,0,2)); assert math.isclose(r.x,10); assert math.isclose(r.y,5)
def test_heading_preserved(): assert math.isclose(LocalizationEstimator().localize(make_world_state(theta=math.pi/4)).theta,math.pi/4)
def test_velocity_preserved(): assert math.isclose(LocalizationEstimator().localize(make_world_state(v=7.5)).v,7.5)
def test_small_noise_stays_close():
 e=LocalizationEstimator(.5);c=SensorNoiseConfig(.05,.05,.01,.05,42)
 for _ in range(10):r=e.localize(make_world_state(20,-10,.2,3),c)
 assert abs(r.x-20)<1 and abs(r.y+10)<1 and abs(r.theta-.2)<.5 and abs(r.v-3)<1
def test_consecutive_measurements_stable():
 e=LocalizationEstimator(.3);c=SensorNoiseConfig(2,2,0,1,7);p=None;m=0
 for i in range(30):
  r=e.localize(make_world_state(i,0,0,1),c);assert all(math.isfinite(v) for v in (r.x,r.y,r.theta,r.v))
  if p:m=max(m,abs(r.x-p.x))
  p=r
 assert m<5
def test_nan_measurement_handled_safely():
 e=LocalizationEstimator();g=e.localize(make_world_state(5,5,0,1));r=e.localize(make_world_state(float('nan'),float('inf'),float('nan'),float('-inf')));assert r==g
def test_nan_measurement_with_no_prior_state_falls_back_to_zero(): assert LocalizationEstimator().localize(make_world_state(float('nan'),float('nan'),float('nan'),float('nan')))==EgoState(0,0,0,0)
def test_zero_velocity_stays_near_zero():
 e=LocalizationEstimator();c=SensorNoiseConfig(v_std=.02,seed=1)
 for _ in range(10):r=e.localize(make_world_state(),c)
 assert abs(r.v)<.2
def test_heading_changes_tracked():
 e=LocalizationEstimator(.8);r1=e.localize(make_world_state(theta=0))
 for _ in range(10):r2=e.localize(make_world_state(theta=math.pi/2))
 assert r2.theta>r1.theta and math.isclose(r2.theta,math.pi/2,abs_tol=.01)
def test_output_is_egostate_instance(): assert isinstance(LocalizationEstimator().localize(make_world_state(1,2,.3,4)),EgoState)
def test_valid_world_state_does_not_crash():
 r=LocalizationEstimator().localize(make_world_state(100,-50,1,12),{'x_std':.1,'y_std':.1,'theta_std':.01,'v_std':.1});assert isinstance(r,EgoState) and all(math.isfinite(v) for v in (r.x,r.y,r.theta,r.v))
def test_module_level_localize_function():
 from localization.localization import localize,reset_default_estimator
 reset_default_estimator();r=localize(make_world_state(3,4,0,1));assert isinstance(r,EgoState) and math.isfinite(r.x)
