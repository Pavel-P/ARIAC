Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

## Release 2022, May 22

### Qualifiers Results
- The qualifiers have been evaluated and the output files have been uploaded in the folder of each team on Google Drive. In the qualifiers, we wanted to make sure your package could at least do pick and place. The qualifiers were evaluated on a new docker image where the following issues were fixed:
  - Issues with segmentation fault after assembly is done have been fixed. The reason of the segmentation fault was due to MoveitCommander object not being destroyed properly. This issue existed in ROS Indigo and for some reasons it is still in ROS Melodic. You can prevent the segmentation fault from happening by taking care of destroying the MoveitCommander object yourself. An example of how this is done can be found in [assembly_commander_node.py](../../nist_gear/test_competitor/nodes/../../../test_competitor/nodes/assembly_commander_node.py) (line 18) where a call to `rospy.on_shutdown(commander.shutdown)` is performed.
  - The scroring for assembly has been fixed. All the services and topics reporting the content of a briefcase are also working. Note: The test competitor code provided for assembly (assembly_commander_node.py) sometimes does not properly insert the regulator (incorrect pose but correct color and correct type). This code should be improved by competitors to achieve proper insertion.

- The files used in the qualifers can be found in the folder [qualifiers](../../nist_gear/config/trial_config/qualifiers/).
  - [qual_a.yaml](../../nist_gear/config/trial_config/qualifiers/qual_a.yaml) consists of kitting only.
  - [qual_b.yaml](../../nist_gear/config/trial_config/qualifiers/qual_b.yaml) consists of assembly only.

### Getting Ready for the Finals
- Competitors should focus on getting ready for the finals. 
- Competitors must submit the auto evaluation file by Saturday **05/28 @5pm**.
- The docker image which will be used to evaluate the competitors' system during the final has been uploaded on https://hub.docker.com. The image name is the same as the one used in the qualifiers (we have overriden the previous image).
- Getting ready means to test your system with all the agility challenges.
- For high-priority orders, can your system handle the following scenarios?
  - order_0: assembly, order_1: kitting
  - order_0: assembly, order_1: assembly
  - order_0: kitting, order_1: kitting
  - order_0: kitting, order_1: assembly
- Can your system handle two regular orders where the second order is announced after the first order is completed?
- The robot breakdown challenge has been updated. Instead of disabling a robot controllers right away, we are giving competitors 8 s to 'park' the disabled robot in a location of their choice. A reminder that the status of a robot is published on `/ariac/robot_health`.

  
## Release 2022, May 2

- We have considered some of your requests and we are extending the submission deadline to 5/7 at 5 PM EST.
## Release 2022, May 1

- A new folder was `ariac-docker` was added to the ARIAC package. It contains everything you need to get your package ready for the qualifiers.
- See the [automated evaluation](../documentation/automated_evaluation.md) for more information.

- Fixed ticket [#158](https://github.com/usnistgov/ARIAC/issues/158). Gripper change now works in both development and competition mode.
  - `roslaunch test_competitor gripper_test.launch` starts simulation and competition mode.
  - `rosrun test_competitor change_gripper_test.py` sends the gantry to the gripper station and a gripper change is successfully performed.
## Release 2022, April 30

- Added new fix for issues with movable trays.
- Note: There is no need to use the service `/ariac/kit_tray_1/lock` before submitting a kit tray nor before the use of `/ariac/kit_tray_N/move_to_station`. Before submission or before moving to a station the movable tray and parts in the movable tray will be automatically locked.
- Scripts and docker image will be released during the day.

## Release 2022, April 28

- Fixed issues with movable trays and parts falling of agv2, agv3, and agv4.
- Added a new service to check parts connected to a briefcase. See the [API](../documentation/api.md#process-management)
- Changed the service  `/ariac/kit_tray_X/get_content` to `/ariac/agvX/content`. Although the service `/ariac/kit_tray_X/get_content` is still advertised, do not use it. Now the movable tray and parts inside the movable tray are reported. See the [API](../documentation/api.md#cheats)
- Changed the submission date for the evaluation scripts. The new date is now Monday, 05/02 @5pm to give competitors time to test the new updates. If the updates show more stable results we will release the Docker image and the instructions for the evaluation scripts for the qualifiers on Saturday, 30th.
- **Information on the qualifiers**:
  - The qualifiers will consist of trials where kitting and assembly will be required.
  - The purpose of the qualifiers is to check that competitors made at least some efforts to do kitting and assembly.
  - During the qualifiers, the challenges will only include: flipped part, faulty parts, and sensor blackout.
  - We will address minor issues while you test the new updates.

## Release 2022, April 11
- Updated [wiki](../documentation/competition_specifications.md#movable-trays) to address questions posted in ticket [#133](https://github.com/usnistgov/ARIAC/issues/133).

## Release 2022, April 11
- Fixed AGVs x offset when shipped to assembly stations. 
  - Ticket [#139](https://github.com/usnistgov/ARIAC/issues/139)

## Release 2022, April 9

- Added a ROS service to lock/unlock a movable tray on an AGV. For instance, to lock/unlock a movable tray on AGV1, one can do:
  
    ```bash
    rosservice call /ariac/kit_tray_1/lock
    rosservice call /ariac/kit_tray_1/unlock
    ```

    The movable tray will slightly be lifted from the AGV, so make sure you modify your z position when placing a part on the movable tray. More information on this service can be found on the API page (see section [Process Management](../documentation/api.md#process-management)).

- Edited the Installation page (see section [Install ROS and Gazebo](../tutorials/installation.md#install-ros-and-gazebo)) to specify that **at least** Gazebo  9.16 is required.

- Updated [Readme.md](../../README.md#important-dates) with dates for the qualifiers and the finals.

## Release 2022, April 5

- Updated [scoring](../documentation/scoring.md) documentation with more information on movable trays.
## Release 2022, March 26


* Wiki released along with the software.
* Please report any broken links on the wiki at zeid.kootbally@gmail.com
