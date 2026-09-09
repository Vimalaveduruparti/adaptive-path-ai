# Localization Module

Simulation-grade vehicle state estimation for the hackathon. Uses configurable Gaussian measurement noise and an exponential moving average (EMA) filter to estimate `x`, `y`, `theta`, and `v` from `WorldState.ego`. Invalid measurements fall back safely to the last known-good state or zero.

## Testing

```bash
pytest tests/test_localization.py -v
```
