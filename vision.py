import threading
from networktables import NetworkTables

serverIP = '10.41.88.2'

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; COnnected=%s', % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server=serverIP)
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting...")
    if not notified[0]:
        cond.wait()

NetworkTables.create("Raspi")
# Processing Code
print("Connected successfully to %s", % serverIP)
