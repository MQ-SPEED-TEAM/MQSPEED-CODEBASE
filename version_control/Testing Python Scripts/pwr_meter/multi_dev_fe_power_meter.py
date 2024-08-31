"""
Example of using mulitple devices: a PowerMeter and FitnessEquipment.

Also demos Workout feature of FitnessEquipment, where the device has a thread and sends info to the master.

Refer to subparsers/influx for another example of creating multiple devices at runtime
"""
from openant.easy.node import Node
from openant.devices import ANTPLUS_NETWORK_KEY
from openant.devices.power_meter import PowerMeter, PowerData
from openant.devices.fitness_equipment import (
    FitnessEquipment,
    FitnessEquipmentData,
    Workout,
)


def main(connection = None):
    import logging

    logging.basicConfig(level=logging.INFO)
    node = Node()
    node.set_network_key(0x00, ANTPLUS_NETWORK_KEY)
    devices = []
    # create workout intervals for FE
    workouts = [
        Workout.from_ramp(
            start=150, stop=150, step=350, period=30.0, peak=500, cycles=8
        )
    ]

    devices.append(PowerMeter(node))
    devices.append(FitnessEquipment(node))

    def on_found(device):
        print(f"Device {device} found and receiving")

        if type(device) == FitnessEquipment and len(workouts) > 0:
            device.start_workouts(workouts)

    def on_device_data(page: int, page_name: str, data):
        if isinstance(data, PowerData):
            print(data.average_power)
            try:
                connection.send(data.average_power)
            except:
                print("no pipe attached")
        elif isinstance(data, FitnessEquipmentData):
            pass

    for d in devices:
        d.on_found = lambda: on_found(d)
        d.on_device_data = on_device_data
# 
    try:
        node.start()
#         while True:
#             if power_data_received and time.time() - last_power_update_time > 0.3:
#                 print(0)
#                 print("looped")
#                 power_data_received = False
#             time.sleep(0.3)
    except usb.coreUSBError as e:
        if e.errno == 13:
            print("Access denied ")
        else:
            print(f"USB error: {e}")
    finally:
        for d in devices:
            d.close_channel()
        node.stop()


if __name__ == "__main__":
    main()
