import time
from multiprocessing import Process,Pipe
def power_loop(connection):
    count = 0
    while True:
        count += 0.5
        # block
        time.sleep(0.5)
        # send data
        connection.send(count)