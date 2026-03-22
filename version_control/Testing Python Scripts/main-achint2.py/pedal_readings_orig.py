from openant.easy.node import Node
from openant.devices import ANTPLUS_NETWORK_KEY
from openant.devices.power_meter import PowerMeter, PowerData



def ant_main(connection = None):
    import logging

    logging.basicConfig(level=logging.INFO)
    node = Node()
    node.set_network_key(0x00, ANTPLUS_NETWORK_KEY)
    devices = []
    # create workout intervals for FE

    devices.append(PowerMeter(node))

    def on_found(device):
        print(f"Device {device} found and receiving")

    def on_device_data(page: int, page_name: str, data):
        if isinstance(data, PowerData):
#             print(f"PowerMeter {page_name} ({page}) update: {data}")
            try:
                connection.send([data.instantaneous_power,data.cadence])
            except:
                pass

    for d in devices:
        d.on_found = lambda d=d: on_found(d)
        d.on_device_data = on_device_data

    try:
        print(f"Starting {devices}, press Ctrl-C to finish")
        node.start()
# Uncomment to use keyboard interrupt to stop program when running pedal_readings as main. DO NOT uncomment when using as helper function with main.
#     except KeyboardInterrupt:
#         print("Closing ANT+ device...")
    finally:
        for d in devices:
            d.close_channel()
        node.stop()


if __name__ == "__main__":
    main()

