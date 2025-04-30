^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tsid_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fixing orientation cartesian
* Contributors: vivianamorlando

0.0.4 (2025-04-29)
------------------
* Merge branch 'vmo/fix_for_tests' into 'master'
  Vmo/fix for tests
  See merge request control/tsid_framework!24
* Fix pipeline
* Fix publish of command
* Adding publisher cartesian current
* Adding possibility to have joint state not in joint command
* Adding threshold for min and max joint
* Adding topic for velocity command
* Adding topic for plot the current cmd velocity
* Adding publisher of current command for plot test
* Contributors: vivianamorlando

0.0.3 (2025-04-11)
------------------
* 0.0.2
* Update Changelog
* Merge branch 'vmo/fix_cart_vel' into 'master'
  Vmo/fix cart vel
  See merge request control/tsid_framework!20
* Merge branch 'revert-ab2a11b8' into 'vmo/fix_cart_vel'
  Revert "Update Changelog"
  See merge request control/tsid_framework!22
* Revert "Update Changelog"
  This reverts commit ab2a11b8745c4e25f57522404405d36929e9ac18
* Merge branch 'revert-589ba1a2' into 'vmo/fix_cart_vel'
  Revert "0.0.2"
  See merge request control/tsid_framework!21
* Revert "0.0.2"
  This reverts commit 589ba1a27bc5f34a21d3c592db96bdfe4b4fb7fa
* Contributors: vivianamorlando

0.0.2 (2025-04-11)
------------------
* Merge branch 'vmo/fix_cart_vel' into 'master'
  Vmo/fix cart vel
  See merge request control/tsid_framework!20
* Merge branch 'revert-ab2a11b8' into 'vmo/fix_cart_vel'
  Revert "Update Changelog"
  See merge request control/tsid_framework!22
* Revert "Update Changelog"
  This reverts commit ab2a11b8745c4e25f57522404405d36929e9ac18
* Merge branch 'revert-589ba1a2' into 'vmo/fix_cart_vel'
  Revert "0.0.2"
  See merge request control/tsid_framework!21
* Revert "0.0.2"
  This reverts commit 589ba1a27bc5f34a21d3c592db96bdfe4b4fb7fa
* Contributors: vivianamorlando

0.0.1 (2025-04-09)
------------------
* Merge branch 're-factoring' into 'master'
  Re factoring
  See merge request control/tsid_framework!17
* Removing soft limits
* Fix CMake warning for Boost component python310
* Merge branch 'add/taskjointvel' into 're-factoring'
  add joint posture to velocity
  See merge request control/tsid_framework!18
* add joint posture to velocity
* Fixing qmin shadowing privat variable
* Fix position bounds
* Fix treshold velocity
* Merge branch 'vel/joint_limit' into 're-factoring'
  Vel/joint limit
  See merge request control/tsid_framework!16
* Removing eiquadprog vendor
* clean cmake
* rm unused file
* Formatting fix for tasks
* Formatting fix for velocity controlelrs
* Formatting fix for position controllers
* Fixing cartesian
* Adding current v to problem
* fix the namespace of the cartesian vel and pos
* Change from tsid_vendor to released tsid overlay
* Adding dynamic threshold
* Initializing v_int\_
* Adding v prev
* Fix joint limit reached
* Separate update params for cartesian vel
* Fixing gain for cartesian vel simulation
* set default values for th emanipulation cube
* update the box dynamically
* add dyn manipulation cube
* add manipulation cube and its visualization
* Adding controller name to vel controller
* Adding controller name to topic
* Correct gains
* Adding check for prismatic or revolute joint
* Adding check for position
* Adding publisher desired pose
* Tuning gain
* Fixing cartesian with gravity
* Fix joint space control
* Fix cartesian traj
* Adding while cycle for joint limit flag
* Fix trajectory
* Fix v derivated
* Removing ki in vel task
* reduce threshold
* add joint_limit renforcement for velocity control
* Adding rotation time computation
* Fix orientation - to add max vel
* working torso
* pass v readed instead of v\_
* update gains
* fix q_prev\_ init
* add joint_space_torso_vel_controller yaml and launch files
* fix publisher on cartesian_velocity_controller
* fix on tsid_velocity controller
* add jointSpaceVelTsidController to the controllers
* fix velocity command
* add motor torque constant for torso_lift_joint
* gain for torso joint space
* fix uncrustify
* fix parameters for sinusoide
* fix torso lift joint space yaml
* fix q_int\_ and parameters
* Fix conversion velocity
* added effort sine command
* added velocity sinusoidal
* sinusoide in position
* fix publishers
* fix on q_int
* update posture gains
* Sync traj
* Fixing cartesian pos
* Fixing joint space contorl
* Adding kp and kd for each joint in posture task
* Rebasing
* Cleaning
* Adding rotation time computation
* separate computation parameters for trajectory
* Fix orientation - to add max vel
* Fix vel
* Adding trajectory
* clang format
* add waypoints for linear and rot interpolation
* Changing to velocity interface
* Adding joint space vel
* Refactor of velocity control
* Making dt protected
* clang format
* refactory of the cartesian space controller
* setDesired funct deleted and using directly the callback
* pass to protected some variables + delete not used func
* delete safety_controller namespace for cartesian controller
* remove not-used libraries in joint_space_controller
* getActualState function update
* general fix
* start of the refactoring creating main base class for pos. contr.
* Temporary code in reharsal
* Merge branch 'fix/remove_eiquadprog_vendor' into 'master'
  removed eiquadprog vendor dependency
  See merge request control/tsid_framework!5
* removed eiquadprog vendor
* Merge branch 'vmo/fixes' into 'master'
  Fix cartesian space
  See merge request control/tsid_framework!4
* Fix cartesian space
* Merge branch 'vmo/fixes' into 'master'
  Vmo/fixes
  See merge request control/tsid_framework!3
* Small fixes
* Adding command joint option
* fix task vel
* Adding dt in task cartesian
* Adding cartesian velocity controller
* Adding gain for velocity tasks
* Adding difference betwen joint commadn and state
* Fix yaml
* Adding boolean local frame
* Fix joint vel
* Adding joint space vel control to plugin
* Rm pinocchio vendor
* adding launch file for joint space vel control
* Adding joint space vel controller
* Implementing deactivate function
* Adding file for ee frame
* Adding launch and config for robot frame
* Adding params for local frame
* Adapting to new joint state name params
* Separating state joint from command joint
* Adding orientation
* Merge branch 'joint_space_controller' into 'master'
  Joint space controller
  See merge request ileniaperrella/tsid_framework!2
* fix readme
* joint space controller
* Merge branch 'cartesian_space_controller' into 'master'
  cartesian controller
  See merge request ileniaperrella/tsid_framework!1
* cartesian controller
* changing package  names
* Contributors: David ter Kuile, Mathias Lüdtke, danielcostanzi, ileniaperrella, vivianamorlando
