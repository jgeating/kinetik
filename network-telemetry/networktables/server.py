import time
import ntcore
from wpimath.kinematics import SwerveModuleState

inst = ntcore.NetworkTableInstance.getDefault()
inst.startServer()

SwerveModuleState.WPIStruct

table = inst.getTable("datatable")
# Start publishing topics within that table that correspond to the X and Y values
# for some operation in your program.
# The topic names are actually "/datatable/x" and "/datatable/y".
xPub = table.getDoubleTopic("x").publish()
yPub = table.getDoubleTopic("y").publish()

x = 1.5
y = 3

xPub.set(1.5)
yPub.set(3)

my_states = inst.getStructArrayTopic("MyStates", SwerveModuleState.WPIStruct).publish()

while True:
    x += 1
    y += 2
    xPub.set(x)
    yPub.set(y)
    time.sleep(1)
    print("sleeping...")