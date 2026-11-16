# Robotic Configuration and Gripper Selection System

## Run locally or in Antigravity

Extract the ZIP and open the folder containing `app.py`. Python 3.10 or later
and a modern browser are sufficient. The app bundles its assets and uses the
Python standard library, so the local demo runs offline.

```powershell
python app.py
```

Open http://127.0.0.1:8000 in Chrome or Edge. In Antigravity, use the same
command in the project's terminal and open that URL in the browser. Keep the
terminal running during the demo. Press Ctrl+C to stop the server.
If another app already uses port 8000, run `python app.py --port 8001`
and open http://127.0.0.1:8001.

## Deploy to Vercel

Use the entire folder containing `vercel.json` as the project root. Keep
`public/`, `api/` and the root Python files together.

- **CLI.** Open a terminal in the project root, sign in when prompted and
  run `npx vercel@latest` to create a preview. Open the preview URL and follow
  the checks below. Run `npx vercel@latest --prod` to publish.
- **Dashboard.** Push the full folder to a Git repository, import it in
  Vercel and set **Root Directory** to the folder containing `vercel.json`.
  Choose **Other** as the framework preset if prompted. Leave dashboard
  build and output overrides unset so the checked-in configuration applies.

The frontend is served from `public/`. The Python handler at `api/index.py`
serves `/api?action=cell` and the other JSON actions. No environment variables
or external services are required. Browser tabs hold their own simulation
state and the server processes each request independently. The web app and
the API were verified locally. A hosted deployment has yet to be verified.

On the preview URL, verify that the home page shows the default UR5e
recommendation, the source CSV downloads, and **Load recommendation in
simulator** followed by **Run pick & place** places the red block. If the UI
loads but the controls report an API error, open `/api?action=cell` on the
preview URL. It should return JSON. Check the deployed root directory and
function logs if it returns a platform error page.

- [Vercel Python runtime](https://vercel.com/docs/functions/runtimes/python)
- [vercel.json configuration](https://vercel.com/docs/project-configuration/vercel-json)
- [Deploying with the Vercel CLI](https://vercel.com/docs/projects/deploy-from-cli)

## Viva walkthrough

1. Open the app. Review the prefilled Red block comparison, then choose
   **Load recommendation in simulator** and **Run pick & place**. Change the
   preset to Blue or Gold and compare again to see vacuum or magnetic pickup.
   Use **Download comparison CSV** to save the task and its decisions.
2. In the workcell, drag to orbit and scroll to zoom. Choose Perspective,
   Front or Overhead for a preset view. Click a loose part in 3D to load
   its camera-measured XYZ. The workbench surface is at Z = 0 mm.
3. For a manual run, reset the workcell to Articulated + Parallel jaws, click
   **Scan**, select **Red block**, and click **Use detected XYZ**.
   The overhead image and centroid show where the target came from.
4. Click **Move** and then **Grip**. To place the red block manually, enter
   and move to (440, −105, 175), then (650, 95, 175), then (650, 95, 55).
   Click **Release** and raise the wrist to (650, 95, 175).
5. Try moving to (900, 0, 100) to show workspace rejection. Scan and select
   the gold block with parallel jaws to show tool rejection. The Workcell
   setup panel also lets you compare each robot's motion directly.
6. Choose an **Impact scenario** and click **Reset and run impact demo**.
   **Block chain reaction** shows one part tipping and striking another,
   **Table edge fall** shows a part dropping to the floor, and **Arm link
   contact** lowers the cylindrical arm onto a block away from its gripper.
   Scan again to see the remaining bench parts at their new positions.

## Implementation and interpretation

- `public/robots.csv` contains four public-specification records. `selection.py`
  reads the CSV, evaluates motion, repeatability and payload limits, and
  pairs eligible robots with suitable tools. Read `PROJECT_REPORT.md` for
  source notes, ranking criteria and interpretation.
- `robot.py` defines the synthetic workcell parts and four educational
  kinematic models. `pose` returns the joint readings, link geometry and
  simulated tool position for the selected configuration. `move` samples
  the straight tool path and checks the demonstration workspace, joint
  limits and carried-part clearance above the bench. Contact displaces
  loose parts and is reflected in later camera scans.
- `camera` synthesizes a calibrated orthographic label image. Python finds
  each colour's pixel centroid and maps it to millimetres with a 2.5 mm/pixel
  scale. The known part height supplies Z. The model assumes a fixed overhead
  camera, known colours and ideal lighting.
- `physics.py` steps the scene at 60 Hz. The moving links, joint housings
  and gripper use sampled sphere chains. Loose blocks receive impulses,
  rotate, fall under gravity, and settle against the bench or floor.
  Part-to-part impacts use axis-aligned bounds of the rotated blocks.
- `grip` checks alignment, width, part mass, orientation and surface compatibility.
  `release` checks placement height,
  workbench bounds and overlap. Gripping attaches a part to the tool point
  and centres it within the alignment tolerance. The robot follows its
  commanded trajectory through contact. The collision model uses
  simplified inertia, friction and angular damping. Robot deformation,
  self-collision and collisions with the fixed workcell fixtures are outside
  this model.
- `app.py` exposes the Python operations. `public/app.js` renders the perspective
  view and animates the returned poses. The displayed XYZ error is the
  difference between the commanded and reconstructed simulated tool point.
  A reading of 0.000 mm is consistent with ideal kinematics and does not
  measure the accuracy or repeatability of a physical robot.

See `sample-comparison.csv` and `sample-output.json` for examples of comparison
results and the first camera scan and placement.
Visit the website here https://roboticvisionminiprojsimulation.vercel.app/
