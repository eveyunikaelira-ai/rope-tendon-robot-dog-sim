# rope-tendon-robot-dog-sim

A physics prototype where a “rope-tendon robot dog” moves using tension instead of gears.

## What is included
- `quadruped.urdf`: a minimalist four-legged body with hip and knee hinges.
- `tendon_sim.py`: simulation entry point that converts inverse-kinematics leg targets into antagonistic tendon tensions and applies the resulting torques in PyBullet.

## Running the simulation
The simulation can run headless (default) or with a GUI if your environment supports OpenGL.

```bash
# headless sanity run
python tendon_sim.py --steps 480

# open a GUI window to watch the rope-driven walk cycle
python tendon_sim.py --gui --steps 2400
```

Flags:
- `--steps`: number of physics steps to execute (240 Hz step size when GUI is enabled).
- `--gui`: open a PyBullet window; omit for `p.DIRECT`.
- `--realtime`: optional real-time stepping when the GUI is enabled.

## How the tendon model works
Each joint is paired with two opposing tendons that can only pull. Inverse kinematics finds hip and knee angles for each target foot position. A PD controller computes desired joint torques for those angles. The torques are converted into physical tensions using a fixed moment arm, ensuring that all motion is produced through rope pull rather than direct motor position commands.
