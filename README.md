# missile-guidance-sim
Kinematic missile guidance simulation comparing Pole Placement, LQR, and Proportional Navigation. Tracks performance and accumulated steering effort for each method for comparison.

## How to run
**Dependencies**
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

**Parameters**
User is free to play around with tau, N and (Q,R) \
-  tau represents the response time for the proportional controller
-  N is the gain for the proportional navigator
-  (Q,R) represent the cost function for the LQR controller

## About the code
This project is a 2D missile guidence simulator, made to make me understand the limits a proportional controller, and an introduction to the LQR and Proportional Navigation controllers. 

The simulation displayed using pygame, and plots the real time position of the different missiles and the target, with a HUD with useful telemetry for eachh of the 3 missiles. The target is self is random — it spawns somwhere between ((200,600)(200,600)), with an undetermined speed and angle. 

After the simulation is finished, two plots will appear to compare the performance of the missiles. The first plot is the total accumulated effort. It is calculaed by adding up all the changes to theta. This variable is something you'd like to minimize, to optimalize performance.
```python
effort.append(effort[-1] + abs(u*dt))
```

## What I have learned
