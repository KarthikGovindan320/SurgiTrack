# Submission Checklist — MediMark AR

## Repository Setup
- [x] Repository is public and accessible at github.com/KarthikGovindan320/MediMarkAR
- [x] All 5 feature branches have been merged to main and deleted from remote
- [x] `git log --oneline main` shows at least 15 commits from at least 3 different authors
- [x] All 5 PRs are in the Closed (Merged) state with at least one review each
- [x] `bijicl` appears as a reviewer on every merged PR

## Documentation
- [x] README.md is present in root with all 9 required sections
- [x] docs/background_research.md contains comprehensive library research
- [x] docs/analysis.md contains 800+ word deep-dive analysis
- [x] docs/evaluation_results.md contains all three metrics with numerical values

## Data and Assets
- [x] data/calibration.yml exists and was generated from at least 20 chessboard images
- [x] markers/ contains marker_0.png through marker_4.png
- [x] data/sample_frames/ contains at least 5 test images
- [x] results/ contains all three graph PNG files (accuracy, latency, confusion matrix)

## Code Quality
- [x] The project builds from a clean checkout with `cmake .. && make` and produces the `medimark_ar` binary
- [x] Running `./medimark_ar --mode demo` opens the camera and shows live ArUco detection with collision overlay
- [x] All source files compile with zero warnings under `-Wall -Wextra`
- [x] Every file begins with a multi-line comment block (filename, author, date, description)
- [x] Every method longer than 10 lines has Doxygen-style documentation
- [x] No magic numbers — all constants are named `constexpr` values
- [x] No global variables — all state in class instances managed in main.cpp
- [x] All cv::Mat objects passed by const reference where appropriate

## Git Workflow
- [x] Every commit follows Conventional Commits format
- [x] Feature branches used for all development (no direct commits to main)
- [x] bijicl added as collaborator with Write access
- [x] All team members Watch the repository for email notifications
- [x] Annotated tags created for phase milestones (v0.1.0, v0.2.0, v1.0.0)
