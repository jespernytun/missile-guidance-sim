# missile-guidance-sim
Kinematic missile guidance simulation comparing Pole Placement, LQR, and Proportional Navigation. Tracks performance and accumulated steering effort for each method for comparison.

## How to run
Dependencies \
```python
pygame
matplotlib.pyplot
numpy
sciplot.signal place_poles
scipy.linalg solve_contininus_are
```

```console
git clone https://github.com/jespernytun/missile-guidence-sim
python3 -m venv venv
pip install numpy
pip install scipy
pip install pygame
pip install matplotlib
source bin/activate
./missile.py
```

Parameters \
User is free to play around with tau, N and (Q,R) \
-  tau represents the response time for the proportional controller
-  N is the gain for the proportional navigator
-  (Q,R) represent the cost function for the LQR controller

## About the code

## What I have learned
