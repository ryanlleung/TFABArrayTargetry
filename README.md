# TFAB Array Targetry

## Description
This repository contains code developed for MATIAS (Modular Array Target Insertion and Alignment System) from Target Fabrication.

## Overview
`kinematics.py` models MATIAS as a serial chain of six fixed offsets (ground → X stage → Z stage → Y base → Y stage → ball → target centroid). The system is driven by five stage coordinates `[X, Y, Z, Rx, Ry]`: X, Y and Z are linear translations, and Rx/Ry are rotations applied at the ball joint. Lengths are in mm (assumed from the geometry values) and all angles are in degrees.

- **Forward kinematics** compose 4×4 homogeneous transforms (`scipy.spatial.transform.Rotation`, `xyz` Euler order) to find the pose of each link and of any point on the target plate.
- **Inverse kinematics** use `scipy.optimize.fsolve` to find the stage positions that place a chosen target point at a requested `[x, y, z, Rx, Ry]` setpoint.

The target plate is modelled as 190 × 290 (width × height), matching the extents used in the visualiser and the pitch/yaw tool.

## Files

| File | Description |
| --- | --- |
| `kinematics.py` | Kinematics of MATIAS. Functions: `get_geometry()` (link offsets), `get_T(pose)` (6-DoF transform), `get_transformation(stagePos)` (ground-to-link transforms), `get_transformation_TP(stagePos, poseTP)` (ground-to-target-point transform), `get_target_plane(stagePos, ...)` (plate surface mesh) and `get_stage_positions(poseSetpoint, poseTP)` (inverse kinematics). Running it directly prints an example inverse-kinematics solution. |
| `stageviz.py` | Applies the kinematics to visualise transformations of points on the system in an interactive 3D matplotlib window. Enter a setpoint (`x, y, z, Rx, Ry`) in the text box and choose a target point with the `TPx`/`TPy` sliders; it solves for and displays the required stage positions, the link positions and the target plane. |
| `pitchyaw_corrector.ipynb` | PyQt5 GUI ("Confocal Data Logger") to correct for pitch and yaw of the target plate. It uses `watchdog` to watch a user-selected directory for new `.csv` files written by the IFD2422 DAQ Tool V3.5.0 confocal sensor software, takes the mean of the second column of each file (after skipping 2 header rows) as the Z height of the next point in the loaded points file, fits a plane and reports pitch (Rx) and yaw (Ry). The XYZ table can be exported to CSV. The selected directory is remembered in `logs/logdir.txt` (the `logs/` folder is not in the repo; create it with `mkdir logs` before first use, or selecting a directory will raise an error). Marked "Not reviewed" in the code. |
| `roll_corrector.ipynb` | To correct for roll of the target plate. Computes a perspective transform (`cv2.getPerspectiveTransform`) from four locator-target positions measured on the plate to their positions on the stage (entered in the notebook, counter-clockwise, origin at the bottom-right corner of the plate), then maps the points in `points/target_points_2020_xy.csv` to stage coordinates, plotting and printing them. |
| `gonio_measurements.ipynb` | Transforms linear extensions to angles on the goniometer. Fits cubic polynomials (and their inverses) to measured extension (mm) vs angle (deg) data for axis 0 (pitch) and axis 1 (yaw), and prints the coefficients. |
| `points/` | CSV files with columns `Point, X, Y` giving XY coordinates of points on the target plate (see below). |

### `points/` directory

| File | Points | Used by |
| --- | --- | --- |
| `gap_points_xy.csv` | 126 (X 15–175, Y 15–275) | `pitchyaw_corrector.ipynb` (default) |
| `target_points_2020_xy.csv` | 29 (X 20–100, Y 20–170) | `roll_corrector.ipynb` |
| `target_points_xy.csv` | 43 (X 20–140, Y 20–170) | Not referenced in code; can be passed to either notebook instead |

## Requirements
The following packages are required:
- numpy
- scipy
- matplotlib
- pandas
- PyQt5
- watchdog
- scikit-learn (plane fit in `pitchyaw_corrector.ipynb`)
- opencv-python (`cv2`, in `roll_corrector.ipynb`)
- Jupyter, to run the notebooks

## Installation
```bash
git clone https://github.com/ryanlleung/TFABArrayTargetry.git
cd TFABArrayTargetry
pip install numpy scipy matplotlib pandas PyQt5 watchdog scikit-learn opencv-python jupyter
```

## Usage
Run everything from the repository root, since the notebooks load files using relative paths such as `points/...`.

- **Kinematics / visualisation**
  ```bash
  python kinematics.py   # prints example stage positions
  python stageviz.py     # interactive 3D visualiser
  ```
  From Python: `get_stage_positions([x, y, z, Rx, Ry], [TPx, TPy, 0, 0, 0])` returns `[X, Y, Z, Rx, Ry]` stage positions.
- **Pitch/yaw correction** (`pitchyaw_corrector.ipynb`): set the points file in the last line of the cell (`points_xy_path=...`). Start the IFD2422 DAQ Tool and set its CSV output path, run the notebook, select that directory and click **Start Logging**. Each snapshot saved in the DAQ tool adds a point; the plot title shows the fitted pitch and yaw. Use **Remove Last Point** to discard the latest measurement (deletes its CSV file) and **Export Data** to save the XYZ table. **Warning:** **Remove All Points** deletes *every* file in the selected directory, not just the logged CSVs, so point the tool at a dedicated folder.
- **Roll correction** (`roll_corrector.ipynb`): enter the measured `plate_points` and `stage_points`, then run all cells to get the transformed target positions in stage coordinates.
- **Goniometer calibration** (`gonio_measurements.ipynb`): edit the measurement arrays if needed and run all cells to get the fitted polynomials.

## License
MIT License, see [LICENSE](LICENSE).
