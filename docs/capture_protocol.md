# Capture protocol: one page

For anyone with an iPhone. No technical knowledge needed. Follow the steps in order.

**You need:** an iPhone. For the 3D scan it must be a Pro model with LiDAR: iPhone 12 Pro or newer (15 Pro, 16 Pro, Pro Max). Photos and video work on any iPhone 15 or newer. Battery above 50%, at least 2 GB free.

## 1. Before you start (5 minutes)

1. Install **Stray Scanner** (free, App Store) on the Pro iPhone. Open it once and allow camera access.
2. Switch on **all lights** in every room. In daylight, open curtains.
3. **Open every interior door fully.** Do not move furniture.
4. Ask other people to stay out of the rooms while you scan.

## 2. 3D scan (Pro iPhone, about 1 minute per room)

One recording covers the whole property.

5. Stand in the first room. In Stray Scanner, **start recording**.
6. Hold the phone **upright at chest height**, screen facing you. Walk **slowly**, about half your normal pace, roughly 1 m from the walls. Turn slowly: never spin around quickly.
7. **In every room:**
   - **Walls:** face each wall and tilt the phone slowly from floor to ceiling.
   - **Ceiling:** stand in the middle of the room and point the phone **straight up at the ceiling** for 3 seconds.
   - **Doors and windows:** for each one, stop 1–2 m in front of it and hold the phone still for 2 seconds, so **both sides of the frame and the top** are on screen. Then, for doors, **walk through** the doorway; do not just look through it.
   - **Floor:** do not point the phone at the floor for long.
8. Go to the next room **through the door** and repeat step 7. Visit every room.
9. **Finish where you started:** go back to the first room and look at its first wall again. Then **stop recording**.

## 3. Photos (any iPhone 15 or newer, Camera app)

10. For each room, take **2 to 8 photos** from the corners and the doorway. Each photo should show the line where the walls meet the floor. Neighbouring photos should overlap by about a third.
11. Keep each room's photos together: make one **album per room**, named `room-1`, `room-2`, …

## 4. Video (any iPhone 15 or newer, Camera app)

12. Record **one video** walking the same route as the 3D scan (steps 6–9), phone upright, slow and steady.

## Avoid

- **Mirrors, glass and shiny wet floors:** scan past them, do not stop and stare at them.
- **Dark rooms:** switch the lights on.
- **People or pets walking in front of the phone.**
- **Covering the camera with your fingers** (back of the phone, top corner).

## 5. Hand the files over

13. **3D scan:** open the **Files** app → *On My iPhone* → *Stray Scanner*. AirDrop the newest recording folder to the laptop (or share it as a zip).
14. **Photos:** AirDrop each room album into a folder named `photos/room-1`, `photos/room-2`, …
15. **Video:** AirDrop the video file.
16. On the laptop, copy everything into the project's `data/raw/` folder and run, for the 3D scan:

    `python -m property_capture run --input data/raw/<recording folder or zip>`

    The plan is written to `outputs/run_<time>/floorplan.png`.

---
*Engineering notes (not part of the operator's page): every step above comes from what the pipeline needs, see `reports/*/README.md`.*
- *Steps 7 ceiling and 9 finish:* ceiling height is only reported where enough ceiling was seen (`reports/ceiling`), and revisits give drift evidence (`reports/drift`).
- *Step 7 doors and windows:* an opening width is only measured where both jambs were seen (`reports/opening_widths`).
- *Steps 7 and 8, walking through doorways:* this is what links rooms (`reports/room_segmentation`).
- *Step 7, not pointing at the floor:* in `single_scan_floor_only` the tops of doorways were never seen, so doors could not be classified (`reports/openings`).
- *Not yet verified:* the app name (Stray Scanner) is inferred from the supplied files' format (assumptions.md B-20), and its button labels and export menu must be checked on a real phone before the defense, because the page is followed literally. Only the 3D scan is processed by the pipeline today; photo and video tiers are not implemented yet (`docs/device_matrix.md`).*
