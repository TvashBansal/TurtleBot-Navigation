> **Historical development/workflow document (15 October 2026).** The earlier claims that MuJoCo was unverified describe the **pre-v0.19 packaging state**, not the current validation. For final v0.19 15/15 MuJoCo results, use [the final validation report](FINAL_VALIDATION_V19.md); for submission use [the final checklist](SUBMISSION_CHECKLIST_V19.md). The current validated configuration is `configs/mujoco_v19_precision_trial.yaml`; hardware is still untested.

# GitHub-first development and evidence workflow

Repository name: **TurtleBot-Navigation** (private until the tutor/submission process requires otherwise).

The project is intended to be developed and executed **inside GitHub Codespaces**, while Git stores the *complete reproducible project*: software, automated tests, selected run outputs, plots, metrics, and short MP4 videos. Lab hardware still requires a physical computer with ROS 2 and the instructor-approved TurtleBot3.

## Start in GitHub Codespaces

1. Open the private `TurtleBot-Navigation` repository on GitHub.
2. Select **Code → Codespaces → Create codespace on main**.
3. Once this update has been uploaded, select **Rebuild Container** so `.devcontainer/` installs Python, headless OpenGL/OSMesa libraries and test dependencies.
4. Run:

```bash
python -m pytest -q
python scripts/verify_archive.py results
```

Codespaces does not guarantee hardware-accelerated graphics; `MUJOCO_GL=osmesa` is configured to *attempt* headless software rendering. The generated MuJoCo scene, camera, LiDAR, video and physics are not yet verified there. If the backend fails, keep error logs and diagnose rather than presenting the kinematic backend as MuJoCo evidence.

## Record permanent test evidence

```bash
python scripts/record_tests.py --out outputs/tests_2026_10_18
python scripts/archive_run.py outputs/tests_2026_10_18 --name tests_2026_10_18
```

The complete JUnit XML, console logs, test counts, versions and source revision are saved under `results/tests/`.

## Record permanent experiment evidence

```bash
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --video --out outputs/nominal_seed7
python scripts/archive_run.py outputs/nominal_seed7 --name nominal_seed7
python scripts/verify_archive.py results
python scripts/reproduce_archive.py results/mujoco/nominal_seed7
```

For multiple seeds:

```bash
python -m turtlebot_integrated.cli suite --backend mujoco --seeds 5 --out outputs/mujoco_suite
python scripts/archive_run.py outputs/mujoco_suite --name suite_5_seeds
```

The suite records per-trial plots, trajectory logs, and aggregate metrics. Use `--video` on `suite` only when all those videos are actually needed. The single-run `--video` is sufficient to record a selected nominal/disturbed demonstration. The video will appear within that archived run, provided the renderer and codec work. Inspect `video_error.txt` if the MP4 was not generated.

## Save it to GitHub

After reviewing the new files, run from the Codespace terminal:

```bash
git status --short
git add -A
git commit -m "Add reproducible result archives and video evidence workflow"
git push
```

`outputs/` remains ignored because it can contain failed trials, huge videos and transient files. `results/` is tracked by Git, and archived experiments are deliberately never overwritten. All *important* previous results must be published into `results/` **and pushed**; the fact that a file exists in Codespaces does not make it available to a tutor who clones the repository.

The `.github/workflows/tests.yml` workflow runs tests on every push, saves temporary GitHub Actions evidence, and checks the integrity of the committed archives. `.github/workflows/mujoco-manual.yml` provides a **manual, experimental** Linux MuJoCo run and uploads temporary artifacts. GitHub Actions artifacts can expire, so manually archive and commit selected outputs into `results/` for lasting access.

Standard Git has a 100 MB maximum per file on GitHub, with warnings above 50 MB. Compress videos below 50 MB when possible, or configure Git LFS for larger clips; Git LFS data may be subject to account storage and bandwidth limits. Do not commit system secrets, credentials, camera recordings of people, or other sensitive inputs.

## A tutor can check the archive

```bash
python scripts/verify_archive.py results
python scripts/reproduce_archive.py results/tests/initial_python_tests
python scripts/reproduce_archive.py results/kinematic/nominal_seed7_software_baseline
```

After real physics trials are committed, replace the final path with `results/mujoco/<experiment-name>`.

**Caution:** a reproduction runs the code currently checked out. `manifest.json` stores the source commit when the run was originally made; checking out that revision (when present) is needed for rigorous code-version comparisons. The initially archived software samples were made before a Git revision was available in the packaging environment; subsequent runs inside Codespaces will record it.
