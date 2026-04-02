#!/usr/bin/env python3
"""
Minimal LQR demo for beginners.

This script intentionally avoids ROS, Gazebo and the full project code.
It keeps only the core idea:

1. Define a very small linear model of an inverted pendulum.
2. Choose Q and R to describe what we care about.
3. Solve for the LQR gain K.
4. Run a simulation in the terminal and print the result.

Run:
    python3 examples/lqr_minimal_demo.py
"""

from __future__ import annotations

import math

import numpy as np
from scipy.linalg import solve_discrete_are


def solve_lqr_gain(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Solve the discrete-time LQR problem and return the feedback gain K.

    The discrete-time system is:

        x[k+1] = A x[k] + B u[k]

    LQR finds K for the control law:

        u[k] = -K x[k]

    so that the long-term cost

        sum(x^T Q x + u^T R u)

    is minimized.
    """
    p = solve_discrete_are(a, b, q, r)
    k = np.linalg.inv(b.T @ p @ b + r) @ (b.T @ p @ a)
    return k


def main() -> None:
    # Time step of the discrete simulation.
    dt = 0.02

    # We use a very small 2-state inverted pendulum model:
    #
    #   x = [theta, theta_dot]
    #
    # theta      : pendulum angle from upright (rad)
    # theta_dot  : angular velocity (rad/s)
    #
    # The continuous idea is roughly:
    #
    #   theta_ddot = a * theta + b * u
    #
    # where:
    # - positive theta tends to make the pendulum fall farther away
    # - u is our control input
    #
    # For a small beginner demo, we discretize this into:
    a = np.array([
        [1.0, dt],
        [9.0 * dt, 1.0],
    ])
    b = np.array([
        [0.0],
        [1.0 * dt],
    ])

    # Q says how much we care about state errors.
    #
    # Here we care a lot about theta being small, and somewhat less about
    # theta_dot being small.
    q = np.diag([40.0, 4.0])

    # R says how much we want to avoid very aggressive control effort.
    r = np.array([[0.5]])

    k = solve_lqr_gain(a, b, q, r)

    print("Minimal LQR demo")
    print("================")
    print("State: x = [theta, theta_dot]")
    print("Control law: u = -Kx")
    print(f"A =\n{a}")
    print(f"B =\n{b}")
    print(f"Q =\n{q}")
    print(f"R =\n{r}")
    print(f"K = {np.array2string(k, precision=3)}")
    print()

    # Initial condition:
    # Start slightly tilted away from upright.
    x = np.array([0.20, 0.0])

    # Simulate for a few seconds.
    total_time = 5.0
    steps = int(total_time / dt)

    print("time(s)  theta(rad)  theta(deg)  theta_dot(rad/s)  control(u)")
    print("-------  ----------  ----------  ----------------  ----------")

    for step in range(steps):
        t = step * dt

        # LQR control.
        u = -(k @ x).item()

        # Optional saturation so the output stays physically reasonable.
        u = max(-5.0, min(5.0, u))

        # Discrete state update.
        x = a @ x + b.flatten() * u

        # Print every 0.1 seconds to keep the terminal readable.
        if step % 5 == 0:
            theta = x[0]
            theta_deg = math.degrees(theta)
            theta_dot = x[1]
            print(
                f"{t:7.2f}  "
                f"{theta:10.4f}  "
                f"{theta_deg:10.2f}  "
                f"{theta_dot:16.4f}  "
                f"{u:10.4f}"
            )

    print()
    print("Interpretation:")
    print("- If |theta| goes toward 0, the pendulum is moving back toward upright.")
    print("- If theta_dot also goes toward 0, the motion is settling down.")
    print("- That is the core idea of LQR: choose u from the full state x, not")
    print("  from a single error term only.")


if __name__ == "__main__":
    main()
