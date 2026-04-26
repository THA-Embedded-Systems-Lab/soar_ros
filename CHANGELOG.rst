^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soar_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* doc: add ci status badge
* build: add launch_testing to package.xml
* ci: use docker based images for setup-ros
* fix: add ament_cmake_test to package.xml
* ci: use cyclcon DDS as default
  Co-authored-by: Copilot <copilot@github.com>
* build: update docker configuration
* fix: replace ament_target_dependencies with target_link_libraries
* fix: upgrade minium cmake version to jazzy spec
* chore: reduce most logs from info to debug
* fix: remove Soar output log (`#6 <https://github.com/THA-Embedded-Systems-Lab/soar_ros/issues/6>`_)
  This is due to Soar output log command interference with xml socket communication
  Fixes: `#6 <https://github.com/THA-Embedded-Systems-Lab/soar_ros/issues/6>`_
* feat!: add multi agent support
* Contributors: Moritz Schmidt

0.0.2 (2024-12-17)
------------------
* ci: add jazzy and rolling tests. Disable flaky client test
* docs: Add logos and update images.
* fix: Stop SoarRunner Kernel thread on error.
* ci: Change docs build to 1.12 Doxygen version.
* build: Make ament_index_cpp dependency explicit
* docs: Fix typos and add Test Agent description.
* build: Fix tests and restrict testing to launch testing.
* ci: Publish subdirectory of rosdoc2 build process
* Contributors: Moritz Schmidt, dependabot[bot]

0.0.1 (2024-09-13)
------------------
* Initial commit
* Contributors: Moritz Schmidt
