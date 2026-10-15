# Upload and versioning

1. Create a new **private** GitHub repository (e.g., `turtlebot3-sensing-estimation-control`) using the project-associated account; keep it private during development.
2. Extract the ZIP and open the project root in Codespaces or your local IDE.
3. Initialise, add, commit and push:

```bash
git init -b main
git add .
git commit -m "Initial Track A simulation and hardware integration baseline"
git remote add origin https://github.com/YOUR_ACCOUNT/YOUR_PRIVATE_REPO.git
git push -u origin main
```

4. Keep `outputs/` excluded by `.gitignore` during testing. Before submission, copy **selected validated MuJoCo results** to `docs/results/` with clear conditions and commit them. Upload short video(s) to GitHub Releases or a supported host if too large for git.
5. After your review, correct AI attribution, test on the target platform and change the repository to **public** before the assessment deadline if required by Canvas. Record commit hashes and test versions.

No connector access to your GitHub account is required. This ZIP contains all source files so you can upload it as a standalone folder.
