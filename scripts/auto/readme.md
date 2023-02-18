# File-Based Auto Functionanilty and Usage

Contents
========
- [Auto](#auto)
    - [Auto commands](#auto-commands)
        - [Trajectory](#trajectory)
        - [State](#state)
        - [Time](#time)
        - [Reset odom](#reset-odom)
        - [Action](#action)
        - [Split](#split)
        - [Xmode](#xmode)
        - [Elevarm](#elevarm)
        - [Accelearation](#acceleration)
- [Points](#points)

## Auto

Auto files are composed of lines of commands in the following format:
```
command_name,command_arg_1,command_arg_2,...
```
Example auto file:
```
reset_odom,start
trajectory,start,bugs
time,500
state,flywheel,FLYWHEEl_DEFAULT
```
Note that there are no spaces. 

Auto files are located in the roboRIO under `~/auto/autos/`.

Autos, as well as point and action files, are CSV files. 

## Auto Commands

Auto docs notation:
- `<>` means it is a required field
- `[]` means it is an optional field
- `""` means a choice is a literal

### Trajectory
Constructs a trajectory between 2+ (see below) points.

Syntax: `trajectory,<start_point>,<end_point>,<rotation>,["normal"/"reversed"]`

Trajectories placed in succession will merge into one trajectory. As such, the robot may not land exactly on the inbetween points specified. Note that this merging occurs only between trajectories with the same config. As an example,
```
trajectory,p1,p2,0
trajectory,p2,p1,0,reversed
```
would result in two different trajectories being created. If there is a command placed between two trajectories, they are not put together

The trajectory config is normal by default.

### State
Sets the state for a subsystem.

Syntax: `state,<subsystem_name>,<state_name>`

<!-- @TODO Write out the available subsystems and the available states -->

### Time
Creates a delay between actions

Syntax: `time,<wait_time>`

wait_time is in milliseconds

### Reset odom
Resets odometry to a specified pose

Syntax: `reset_odom,<intial_point>,<initial_heading>`

### Action
***ACTIONS ARE BROKEN as of 2/18/23***

Runs an action

Syntax: `action,<action_name>`

`action` is a special command. It's essentially, in itself, an auto. Actions can use all the commands found in a regular auto, including other actions. As such, they can be nested (_**Nesting is untested**_). The action files use the exact same syntax as a regular auto file. The only difference between an action and a regular auto is that actions are unavailable to run and autos cannot be run within another auto.

Actions are found in the `~/auto/actions/` folder. 

An action's name is just its filename, without the ending. So for example, if an action file is name `testAction.csv`, its name as written in the `action` command would be `testAction`.

### Split

Breaks up trajectories

Syntax: `split,[velocity],["multiplier"]`

The split command works to break up trajectories, as combined trajectories sometimes act strangely. It does this by setting the end and start velocities for the trajectories around it, so there's a transitional velocity between them. You can choose to change this transitional velocity with the `velocity` parameter. If you put `multiplier` after, it'll treat the number you entered as a multiplier to the default transitional velocity. So entering `split,0.75,multiplier` will set the velocities to `4.952 * 0.75 = 3.714m/s`. 

The split command doesn't have to be inbetween two trajectories, but it'll only affect trajectories directly adjacent to the line (unless the `acceleration` command as a neighbour, in which case it'll affect the trajectory beyond it)

Note that this command only works with the trajectory command to do nothing - by itself, its useless. 

### Xmode

Puts robot into xmode

Syntax: `xmode`

Turns all wheels into the center of the robot so its unable to move around.

### Elevarm

Moves the elevarm

Syntax: `elevarm,<piece>,<direction>,<position>,["parallel"]`

Move the elevarm to a specified position and direction. Contrary to `state`, the auto does not move on until the elevarm reaches the deadband, unless you enter `parallel`, in which case its non-interrupting.

### Acceleration

Changes trajectory acceleration

Syntax: `acceleration/accel,<acceleration>,["multiplier"]`

Sets the acceleration used to calculate the feed forward movement. Contrary to split, this applies the acceleration to all trajectories written after it, so you don't need to place it right before each trajectory.

## Points

Point files are located under `~/auto/points/`

Point files are made of lines of the following:
```
point_name,point_x,point_y
```
An example point file:
```
foo,5.5,2
bar_1,6,6.9
0baz,1,3
```
Note that there are no spaces.

Point files contain all the points used in autos and actions. Point files can be updated independently of autos, as autos use the point names.