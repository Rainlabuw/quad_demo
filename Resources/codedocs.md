# quad_demo codedocs
The following is a list of available server commands that can be sent from a client to the crazyflieServer node on the ROS network via publishing to the crazyflieServer/Commands topic. These commands are executed in the ```ros_bridge_execute``` function of the ```CrazyflieServer``` class inside crazyflieServer.py.
## Format
All commands are in the following format:
```
commandname#argument1#argument2#argument3#...
```
Commands are delimted by hash marks ```#```. The first delimited token ```commandname``` is the command to be executed by the crazyflieServer, while the following tokens represents the arguments/parameters of the command. At this time, all arguments are required to properly execute a command.

For example, the command for moving the drone to absolute world coordinates x=1m, y=0.5m, z=1.5m, yaw=0 in two seconds using the high level commander would be:
```
hl_SetPos#1#0.5#1.5#0#2
```

## Command List
**hl_SetPos** 
```
hl_SetPos#x#y#z#yaw#duration
```
Argument Count: 5, Duration: duration seconds

**hl_TakeOffAll**
```
hl_TakeOffAll
```
Argument Count: 0, Duration: 2 seconds

**hl_LandAll**
```
hl_LandAll
```
Argument Count: 0, Duration: 2 seconds

**hl_TakeOffAll**
```
hl_TakeOffAll
```
Argument Count: 0, Duration: 0 seconds

**sp_SetPos**
```
sp_SetPos#x#y#z#yaw
```
Argument Count: 4, Duration: 0 seconds

**sp_LandAll**
```
sp_LandAll
```
Argument Count: 0, Duration: 0 seconds

**spKill**
```
spKill
```
Argument Count: 0, Duration: 0 seconds

**shutdowm**
```
shutdowm
```
Argument Count: 0, Duration: 0 seconds

## Invalid Commands
Any other command strings are considered invalid. They will be logged but silently ignored by the server script.
