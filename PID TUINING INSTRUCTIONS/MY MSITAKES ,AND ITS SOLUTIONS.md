# PID Tuning Guide

## Why I Created This Guide

During the development of my Self-Balancing Robot V1, PID tuning was by far the most difficult and time-consuming part of the project.
I spent many hours testing different values, troubleshooting unstable behavior, and trying to understand how each PID parameter affected the robot.
Since every self-balancing robot is slightly different due to variations in weight, motors, battery voltage, wheel size, and center of gravity, 
there is no single set of PID values that works for everyone.I created this guide to document the tuning process that worked for me so that anyone
building this project can understand the process and avoid repeating the same mistakes I made.

---

# My PID Tuning Process

## Step 1 – Start with Only Kp

I began by setting both Ki and Kd to zero.

```cpp
Kp = 25
Ki = 0
Kd = 0
```

With only Kp enabled, I gradually increased its value until the robot started reacting to its tilt.

### What I Observed
- If Kp was too low, the robot simply fell over.
- If Kp was too high, the motors reacted too aggressively and the robot oscillated continuously.

The goal was to find a value where the robot could almost balance on its own without excessive shaking.

---

## Step 2 – Add Kd

Once Kp was close to the correct value, I started increasing Kd in small steps.

Kd acts like a damper and smooths the robot's movement.

### What I Observed
- Oscillations became smaller.
- The robot recovered more smoothly after being pushed.
- Too much Kd made the robot slow to react.

---

## Step 3 – Add Ki (Only If Needed)

After Kp and Kd were working well, I tested whether the robot slowly drifted while standing.

If it did, I added a very small Ki value.

### What I Observed
- Small drift was corrected.
- Too much Ki caused slow oscillations and instability.

For many balancing robots, Ki can remain very close to zero.

---

# Things I Learned

Throughout the tuning process, I realized that changing only one parameter at a time makes debugging much easier.

I also learned that even small hardware changes—such as moving the battery, changing the wheels, or replacing the motors—can require the PID values to be tuned again.

---

# Common Problems

| Problem | Possible Reason |
|---------|-----------------|
| Robot falls immediately | Kp is too low |
| Robot shakes violently | Kp is too high |
| Robot reacts slowly | Kd is too high |
| Robot drifts slowly | Ki is too low |
| Robot oscillates slowly | Ki is too high |

---

# Tips

- Tune on a flat surface.
- Hold the robot during the first few tests.
- Make only one change at a time.
- Increase values gradually.
- Keep notes of every successful PID combination.
- Fully charge the battery before tuning.

---

# Final Note

PID tuning requires patience. There is no universal set of values that works for every self-balancing robot. The best approach is to understand what each parameter does, make small adjustments, and carefully observe the robot's behavior.

I hope this guide saves you time and helps you avoid the trial-and-error process that I went through while building this project.
