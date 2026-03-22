#======================================================================================================
#
#   2D Missile Tracking Simulation
#   
#   Author      : Jesper Nytun
#   Date        : March 2026
#   University  : Insitit National des Sciences Appliquées de Toulouse
#
#   Description : 2D kinematic missile simulation implementing and comparing three guidance laws:
#                 - P controller (pole placement)
#                 - Linear Quadratic Regulator (LQR)
#                 - Proportional Navigation (PN)
#
#   Model       : State space representation in (r, lambda) coordinates
#                 States  : r (range), lambda (line of sight angle)
#                 Input   : u = theta_dot (missile turn rate)
#
#======================================================================================================

import pygame
import math
import random
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import place_poles
from scipy.linalg import solve_continuous_are

#------------------------------------------------------------------------------------------------------
#                                          Simulation Parameters
#------------------------------------------------------------------------------------------------------

WIDTH, HEIGHT = 1200, 800
FPS = 30
dt = 1/FPS
V = 150.0
t = 0.0

hit_P, hit_LQR, hit_N = [False]*3
HIT_RADIUS = 10 # Connects after if within 10px

# Proportional Controller
tau = 1

# Proportional Navigation
N = 4

# LQR
Q = np.array([[1.0]])
R = np.array([[10]])

#------------------------------------------------------------------------------------------------------
#                                             Init Pygame
#------------------------------------------------------------------------------------------------------
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Missile tracking system")
clock = pygame.time.Clock()

#------------------------------------------------------------------------------------------------------
#                           Functions to draw missile and target to screen
#------------------------------------------------------------------------------------------------------

def draw_target(surface, x, y, radius):
    pygame.draw.circle(surface, (50, 150, 255), (int(x), int(y)), radius, 2)

def draw_missile(surface, x, y, radius, angle, color):
    length = 5*V * dt
    tip_x = x + length * math.cos(angle)
    tip_y = y + length * math.sin(angle)
    pygame.draw.line(surface, color, (int(x), int(y)), (int(tip_x), int(tip_y)), 4)
    pygame.draw.circle(surface, color, (int(x), int(y)), radius, 2)

def draw_line(surface, start, end, width):
    pygame.draw.line(surface, (35, 35, 40), start, end, width)

#------------------------------------------------------------------------------------------------------
#                                           Initial Conditions
#------------------------------------------------------------------------------------------------------

# Create a random target
target_angle = float( random.uniform(0, 2 * math.pi))  # Random direction of target
speed = float(random.uniform(1.5, 1.0))# Random speed for target
target = {
    'x': random.randint(200, 600),
    'y': random.randint(200, 600),
    'vx': speed * math.cos(target_angle),
    'vy': speed * math.sin(target_angle),
    'speed' : speed,
    'radius': 20.0
}
missile_P = {'x': 10.0, 'y':HEIGHT-10.0, 'vx':0.0, 'vy':-V, 'radius':2.0}
missile_LQR = {'x': 10.0, 'y':HEIGHT-10.0, 'vx':0.0, 'vy':-V, 'radius':2.0}
missile_N = {'x': 10.0, 'y':HEIGHT-10.0, 'vx':0.0, 'vy':-V, 'radius':2.0}

theta_P, theta_LQR, theta_N = [-math.pi/2]*3

effort_P = [0]
steering_P = [0]
pos_P = []
timer_P = [t]

effort_LQR = [0]
steering_LQR = [0]
pos_LQR = []
timer_LQR = [t]

effort_N = [0]
steering_N = [0]
pos_N = []
timer_N = [t]

lambda_current_P, lambda_current_LQR, lambda_current_N = [math.atan2((target['y']-missile_P['y']), (target['x']-missile_P['x']))]*3

#------------------------------------------------------------------------------------------------------
#                                               Controllers 
#------------------------------------------------------------------------------------------------------

def pole_placement(missile, target, theta, lambda_current, tau):
    
    # Compute guidance states
    dx             = target['x'] - missile['x']
    dy             = target['y'] - missile['y']
    r              = max(math.sqrt(dx**2 + dy**2), 1.0)
    lambda_prev    = lambda_current
    lambda_current = math.atan2(dy, dx)
    lambda_dot     = (lambda_current - lambda_prev) / dt
    error          = lambda_current - theta

    # State space model
    A = np.array([[V / r]])
    B = np.array([[-1.0]])

    # Compute gain K via pole placement
    p      = [-1.0 / tau]
    result = place_poles(A, B, p)
    K      = result.gain_matrix[0][0]

    # Control law
    u      = -K * error
    theta += u * dt
    effort_P.append(effort_P[-1] + abs(u*dt))
    steering_P.append(u*dt)

    # Update missile position
    missile['x']  += V * math.cos(theta) * dt
    missile['y']  += V * math.sin(theta) * dt
    missile['vx']  = V * math.cos(theta)
    missile['vy']  = V * math.sin(theta)

    pos_P.append([missile['x'], missile['y']])

    return missile, theta, lambda_current

def lqr_controller(missile, target, theta, lambda_current, Q, R):
    
    # Compute guidance states
    dx             = target['x'] - missile['x']
    dy             = target['y'] - missile['y']
    r              = max(math.sqrt(dx**2 + dy**2), 1.0)
    lambda_current = math.atan2(dy, dx)
    error          = lambda_current - theta

    # State space model
    A = np.array([[V / r]])
    B = np.array([[-1.0]])

    # Compute gain K via pole placement
    P = solve_continuous_are(A, B, Q, R)
    K = (1.0 / R[0][0]) * B.T @ P

    # Control law
    u      = -K[0][0] * error
    theta += u * dt
    effort_LQR.append(effort_LQR[-1] + abs(u*dt))
    steering_LQR.append(u*dt)

    # Update missile position
    missile['x']  += V * math.cos(theta) * dt
    missile['y']  += V * math.sin(theta) * dt
    missile['vx']  = V * math.cos(theta)
    missile['vy']  = V * math.sin(theta)

    pos_LQR.append([missile['x'], missile['y']])
                 
    return missile, theta, lambda_current

def proportional_navigation(missile, target, theta, lambda_current, N):
    
    # Compute guidance states
    dx             = target['x'] - missile['x']
    dy             = target['y'] - missile['y']
    r              = max(math.sqrt(dx**2 + dy**2), 1.0)
    lambda_prev    = lambda_current
    lambda_current = math.atan2(dy, dx)
    lambda_dot     = (lambda_current - lambda_prev) / dt
    error          = lambda_current - theta

    # Control law
    u      = N * lambda_dot
    theta += u * dt
    effort_N.append(effort_N[-1] + abs(u*dt))
    steering_N.append(u*dt)

    # Update missile position
    missile['x']  += V * math.cos(theta) * dt
    missile['y']  += V * math.sin(theta) * dt
    missile['vx']  = V * math.cos(theta)
    missile['vy']  = V * math.sin(theta)

    pos_N.append([missile['x'], missile['y']])
                 
    return missile, theta, lambda_current

#------------------------------------------------------------------------------------------------------
#                                           HUD Rendering
#------------------------------------------------------------------------------------------------------
font = pygame.font.SysFont("monospace", 18)

def draw_hud(surface, missile_P, missile_LQR, missile_PN, theta_P, theta_LQR, theta_N, target):
    dx_P   = target['x'] - missile_P['x']
    dy_P   = target['y'] - missile_P['y']
    dx_LQR = target['x'] - missile_LQR['x']
    dy_LQR = target['y'] - missile_LQR['y']
    dx_PN  = target['x'] - missile_PN['x']
    dy_PN  = target['y'] - missile_PN['y']

    # Transform randians to degrees to readability
    a_P   = math.degrees(theta_P)
    a_LQR = math.degrees(theta_LQR)
    a_PN  = math.degrees(theta_N)

    # Colors 
    col_P   = (235, 64,  52)
    col_LQR = (47,  209, 29)
    col_PN  = (255, 165, 0)
    col_hud = (180, 180, 180)

    r_P   = math.sqrt(dx_P**2   + dy_P**2)
    r_LQR = math.sqrt(dx_LQR**2 + dy_LQR**2)
    r_PN  = math.sqrt(dx_PN**2  + dy_PN**2)


    # Header
    surface.blit(font.render("       r(px)       angle(°)   steering   effort", True, col_hud), (10, 10))
    
    # P
    surface.blit(font.render(
        f"P    {r_P:7.1f}    {a_P:8.1f}    {steering_P[-1]:8.3f}    {effort_P[-1]:6.2f}",
        True, col_P), (10, 30))

    # LQR
    surface.blit(font.render(
        f"LQR  {r_LQR:7.1f}    {a_LQR:8.1f}    {steering_LQR[-1]:8.3f}    {effort_LQR[-1]:6.2f}",
        True, col_LQR), (10, 50))

    # PN
    surface.blit(font.render(
        f"PN   {r_PN:7.1f}    {a_PN:8.1f}    {steering_N[-1]:8.3f}    {effort_N[-1]:6.2f}",
        True, col_PN), (10, 70))

    # time and target speed
    surface.blit(font.render(f"t: {t:.2f}s   target speed: {target['speed']:.1f} px/s", True, col_hud), (10, 90))


#------------------------------------------------------------------------------------------------------
#                                           Main Loop
#------------------------------------------------------------------------------------------------------

running = True
while running:
    clock.tick(FPS) # Refreshrate

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
    
    # Move target according to velocity
    target['x'] += target['vx']
    target['y'] += target['vy']

    # Edge collision detection
    if target['x'] < target['radius'] or target['x'] > WIDTH - target['radius']:
       target['vx'] *= -1
    if target['y'] < target['radius'] or target['y'] > HEIGHT - target['radius']:
        target['vy'] *= -1

    if not hit_P:
        missile_P, theta_P, lambda_current_P = pole_placement(missile_P, target, theta_P, lambda_current_P, tau)
        timer_P.append(t)
        if math.sqrt((missile_P['x'] - target['x'])**2 + (missile_P['y'] - target['y'])**2) < HIT_RADIUS:
            hit_P = True

    if not hit_LQR:
        missile_LQR, theta_LQR, lambda_current_LQR = lqr_controller(missile_LQR, target, theta_LQR, lambda_current_LQR, Q, R)
        timer_LQR.append(t)
        if math.sqrt((missile_LQR['x'] - target['x'])**2 + (missile_LQR['y'] - target['y'])**2) < HIT_RADIUS:
            hit_LQR = True

    if not hit_N:
        missile_N, theta_N, lambda_current_N = proportional_navigation(missile_N, target, theta_N, lambda_current_N, N)
        timer_N.append(t)
        if math.sqrt((missile_N['x'] - target['x'])**2 + (missile_N['y'] - target['y'])**2) < HIT_RADIUS:
            hit_N = True
            
    # Timer for plotting errors later
    t += dt
    
#------------------------------------------------------------------------------------------------------
#                                       display simulation
#------------------------------------------------------------------------------------------------------


    screen.fill((15, 15, 25))  # dark background
    draw_target(screen, target['x'], target['y'], target['radius'])

    # Draw missiles
    if not hit_P:
        draw_missile(screen, missile_P['x'], missile_P['y'], missile_P['radius'], theta_P, (235, 64, 52))
    if not hit_LQR:
        draw_missile(screen, missile_LQR['x'], missile_LQR['y'], missile_LQR['radius'], theta_LQR, (47, 209, 29))
    if not hit_N:
        draw_missile(screen, missile_N['x'], missile_N['y'], missile_N['radius'], theta_N, (255, 165, 0))

    # Draw tracers
    draw_line(screen, (missile_P['x'], missile_P['y']), (target['x'], target['y']), 2)
    draw_line(screen, (missile_LQR['x'], missile_LQR['y']), (target['x'], target['y']), 2)
    draw_line(screen, (missile_N['x'], missile_N['y']), (target['x'], target['y']), 2)

    for pos in pos_P:
        pygame.draw.circle(screen, (235, 64, 52), pos, 1)

    for pos in pos_LQR:
        pygame.draw.circle(screen, (47, 209, 29), pos, 1)

    for pos in pos_N:
        pygame.draw.circle(screen, (200, 200, 30), pos, 1)
        
    
    draw_hud(screen, missile_P, missile_LQR, missile_N, theta_P, theta_LQR, theta_N, target)

    pygame.display.flip()
    
pygame.quit()

#------------------------------------------------------------------------------------------------------
#                                     plots
#------------------------------------------------------------------------------------------------------

# Plot total effort
plt.plot(timer_P, effort_P, color="red")
plt.plot(timer_LQR, effort_LQR, color="green")
plt.plot(timer_N, effort_N, color="yellow")
plt.title("Total steering effort over time")
plt.xlabel("Time (s)")
plt.ylabel("Effort")
plt.legend(["Proportional", "LQR", "Navigation"])
plt.show()

# Plot steering
plt.plot(timer_P, steering_P, color="red")
plt.plot(timer_LQR, steering_LQR, color="green")
plt.plot(timer_N, steering_N, color="yellow")
plt.title("Steering input over time")
plt.xlabel("Time (s)")
plt.ylabel("Steering (degrees)")
plt.legend(["Proportional", "LQR", "Navigation"])
plt.show()
