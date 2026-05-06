# Lab Deployment Checklist — QBot Delivery Bot

This is the step-by-step playbook for taking the system from your laptop to the real QBot in the classroom.

---

## What you bring to the lab

- Your laptop with the latest code (git pull beforehand)
- A tape measure (only as backup — calibration uses the robot itself)
- Phone for notes / photos of waypoints
- Optional: small "package" prop for the delivery demo

---

## Phase 0 — Connect to the QBot

```bash
ssh nvidia@192.168.2.59
cd ~/ENGR857_Epuri_kovidh/ros2_ws
git pull   # if your code lives in this workspace
```

If your code is in a separate folder, git clone it on the QBot.

---

## Phase 1 — Bring up the real robot

In one SSH session:
```bash
ros2 launch qbot_platform_driver qbot_platform_driver.launch.py
# (or whatever the exact launch file is for your QBot driver)
```

Verify with `ros2 topic list` — you should see:
- `/cmd_vel` (subscribed by driver)
- `/odom` (published by driver)
- `/scan` (published by LiDAR)
- `/camera/image_raw` (published by camera)

If those exist with publishers/subscribers, the QBot is alive.

---

## Phase 2 — Run your code on the QBot

In another SSH session on the QBot:
```bash
cd ~/qbot-delivery   # wherever you cloned your repo
colcon build --symlink-install
source install/setup.bash

# Launch WITHOUT the fake robot (real one is providing topics):
ros2 launch delivery_bot_bringup bringup.launch.py use_fake_robot:=false
```

State machine + intent communication will start. Check with `ros2 topic echo /robot_state` — should print WAITING.

---

## Phase 3 — Calibrate waypoint coordinates

On your laptop (or another SSH session on the QBot):
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
```

If running on the QBot, your dashboard's `ws://localhost:9090` becomes `ws://192.168.2.59:9090`. Edit `web/index.html` and change `localhost` → `192.168.2.59` (or whatever the robot's IP is).

Then:

1. **Pick the home position** — a corner or memorable spot in the classroom. Put the robot there.
2. **Restart bringup** (so odom resets to (0,0)).
3. **Open dashboard** in your browser.
4. **Use WASD to drive to each landmark** (Teacher's Desk, Front of Class, etc.).
5. **At each landmark, read the Position display** and write down `(x, y)`.
6. **Update the buttons in `web/index.html`**:
```html
   <button onclick="goTo(3.42, 1.85, 'Teacher')">👨‍🏫 Teacher's Desk</button>
```
7. Refresh the page after editing.

---

## Phase 4 — Tune obstacle yield distance

The fake_qbot used `obstacle_distance: 0.6`. Real LiDAR may need adjustment for the classroom environment.

Edit `src/delivery_bot_bringup/config/qbot_params.yaml`:
```yaml
delivery_bot_state_machine:
  ros__parameters:
    obstacle_distance: 0.6   # increase if robot doesn't yield in time
                             # decrease if it yields for distant objects
```

Rebuild + relaunch after editing.

---

## Phase 5 — Test the demo

1. **Drive robot to home position** with WASD
2. **Restart bringup** (clean slate)
3. **Click a destination button** in the dashboard
4. **Watch the badge cycle** through colors as it moves
5. **Walk in front of the robot** mid-trip — it should turn red (YIELDING)
6. **Step aside** — robot continues
7. **Wait for it to return home and reach WAITING** — done

---

## Common gotchas

| Problem | Likely fix |
|---------|------------|
| State stays UNKNOWN | rosbridge isn't running, or dashboard URL points to wrong IP |
| Camera offline | web_video_server isn't running, OR topic name is wrong (try `/image_raw` or check `ros2 topic list`) |
| Robot ignores destination | State machine isn't subscribed to `/destination` — verify with `ros2 topic info /destination` |
| Robot goes wrong direction | Odom frame may be rotated. Drive forward with WASD and check whether `position.x` increases or `position.y` does |
| Robot hits walls | obstacle_distance too small. Increase to 0.8 or 1.0 |

---

## Backup plan if things break

If the dashboard / rosbridge has issues during demo:
```bash
# Send destinations directly via terminal:
ros2 topic pub --once /destination geometry_msgs/msg/Point "{x: 3.0, y: 2.0, z: 0.0}"

# Manual driving via terminal:
ros2 run delivery_bot_teleop wasd_teleop
```

These bypass the web entirely. Always works.
