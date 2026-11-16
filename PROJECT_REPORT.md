# Robotic Configuration and Gripper Selection System

## Problem and objectives

A manufacturing cell must move parts from a workbench to a drop zone. Part
mass, gripping dimensions and contact surface vary. The project screens four
robot configurations against task requirements, recommends a gripper,
explains the decision and demonstrates vision-assisted handling in a browser.

## Dataset

`public/robots.csv` is the original compiled dataset consumed by the program.
Each row records a model, configuration, payload, published workspace metrics,
repeatability, drive, joint sequence, source and measurement basis. Blank fields
represent unavailable or inapplicable values. R identifies a rotary joint and
P a prismatic joint. The catalogue robots accept application tooling selected
for the task. The three gripper profiles and three coloured workpieces are
synthetic simulation inputs with dimensions, masses and material rules
defined in Python.

- Cartesian. Yamaha SXYx 3-axis ZS6 uses three electric ball-screw axes.
  Maximum configurable travel is 1050 × 650 × 150 mm. The model uses the
  slow-lead Z option's 5 kg payload and the worst linear-axis repeatability
  of ±0.02 mm. [Yamaha model datasheet](https://www.yamaha-motor.co.jp/robot/lineup/xyx/shared/pdf/a_SXYx_3_ZS_C_Arm.pdf)
- Cylindrical. The Seiko RT3300 Standard has base rotation, radial and
  vertical slides and tool rotation. Screening uses its 5 kg rated payload,
  305 mm radial stroke, 120 mm vertical stroke and ±0.025 mm positional
  repeatability. [Seiko D-TRAN documentation, university-hosted copy](https://web.cecs.pdx.edu/~mperkows/CLASS_ROBOTICS/2001/seikoRT3200.pdf)
- Polar. The first Kawasaki-Unimate 2000 is a historical, five-axis,
  12 kg hydraulic robot. The verified source gives the payload and
  configuration. It leaves model-specific reach and repeatability unverified
  for this dataset. [Kawasaki Robotics history](https://kawasakirobotics.com/uploads/sites/11/2026/03/Half_a_century_of_Kawasaki_Robotics_EN.pdf)
- Articulated. The Universal Robots UR5e has six rotary joints, 5 kg
  maximum payload, 850 mm reach and ±0.03 mm pose repeatability per ISO 9283.
  [Universal Robots technical specifications](https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-usr-man/complianceUR5e/H_g5_sections/appendix_g5/tech_spec_sheet.htm)

## Method and Python implementation

1. `selection.py` reads the CSV and rejects known failures for travel or
   reach, requested drive, repeatability and combined part-plus-tool mass.
   A candidate with missing required data and no known failure needs
   verification. An optional absolute accuracy requirement invokes this
   same rule because the selected sources give repeatability data.
2. The tool checks use the part's grip width or usable face width, mass and
   material. For a rigid part with accessible sides, parallel jaws rank
   first. A flat sealable top favours vacuum. A flat ferromagnetic top
   favours magnetic pickup. The simulated tool limits include a 15–82 mm
   usable span for the parallel jaws.
3. The score sums 35% unused payload fraction, 35% smallest unused workspace
   fraction and 30% remaining repeatability tolerance fraction. Scores range
   from 0 to 100. The table gives the outcome, reasons and sources for all
   four models. The browser chart compares eligible scores and exports CSV.
4. `robot.py` models three linear degrees of freedom for the Cartesian
   gantry, base rotation plus two slides for the cylindrical arm,
   azimuth and elevation plus extension for the polar arm, and three rotary
   positioning joints for the articulated arm. The simulated tool stays
   vertical. The articulated model reconstructs its tool point from angles
   and link lengths to display ideal kinematic error.
5. The synthetic overhead image uses a calibrated orthographic camera.
   Colour segmentation yields each part's pixel centroid. A 2.5 mm/pixel
   mapping and known part height give XYZ. The app validates straight-line
   movements and animates scan, approach, grip, lift, transfer and release.
   The sampled arm links, joint housings and gripper can strike loose parts.
   Part-to-part impulses, gravity and damped rotation move the parts and
   allow them to tip or fall from the bench. The commanded path continues.
   `app.py` serves the UI and Python functions over HTTP.

## Results and interpretation

The red block preset recommends the UR5e and parallel jaws. The combined
mass is 1.10 kg, the requested reach is 660 mm, and the repeatability limit
is 0.05 mm. The score is 47.1/100. The Yamaha and Seiko models fall short of
the 175 mm Z travel requirement. The Kawasaki model needs additional reach
and repeatability evidence. See `sample-comparison.csv`.

The overhead scan detects the red block at (440, −105, 55) mm. The demo
places it at (650, 95, 0) mm and retreats to tool position (650, 95, 175) mm.
The display rounds the calculated tool error to 0.000 mm. This value is the
ideal simulator residual. See `sample-output.json`.

## Limitations and conclusion

The catalogue screener covers one example per configuration and uses
preliminary workspace checks. Cartesian stroke assumes suitable mounting
and alignment. Cylindrical stroke checks need installation offsets and
angular sector verification. Articulated and polar reach is a scalar
screening bound. A real workcell also requires the manufacturer's
full workspace, flange fit, tool centre of gravity, inertia and load curves.
Vendor repeatability metrics use different test bases.

The rendered workcell uses educational dimensions and simplified contact
geometry. The robot is kinematic, so impulses affect the loose parts.
The model excludes robot self-collision, fixture collision, deformation,
detailed actuator loads, sensor noise and physical grasp dynamics.
A custom selection form describes a
catalogue task. The 3D demo runs using the selected configuration, tool and
the built-in workpiece preset.

The app demonstrates how to trace a recommendation from a documented
specification and part constraint through to a visible pick-and-place cycle.
