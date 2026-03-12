- add example/demo to quick start
- add CI to run custom consistency checks
  - [ ] ... IMPLEMENTATION.md
  - [ ] .env exists for docker-ros build vars
  - [ ] GitLab CI exists?
- is there still something like catkin_lint for package.xml or similar?
- LICENSE.md
- CHANGELOG.md
- (CONTRIBUTING.md)
- search for TODO

### Future Work

- [ ] pre-commit should work out-of-the-box in devcontainer
- [ ] integrate ament_cmake_ruff (currently not available on apt)

### Work Elsewhere

- [ ] add unit tests to ros2-pkg-create
- [ ] guide for moving repos to GitHub
  - create new repo
  - repo settings
  - important files to add
  - naming conventions
  - check package.xml deps
  - (re-generate package.xml/CMakeLists.txt?)
  - ...

### Repo settings

- Enable GHCR on Org Level: Org Settings / Actions / Workflow permissions / Read and write permissions
- Enable GH Pages: Activate GitHub Pages Documentation: Repo Settings / Pages / Branch / gh-pages
- ... check all settings, e.g., MR rules ...
