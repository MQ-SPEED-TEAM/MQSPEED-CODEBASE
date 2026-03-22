def get_virtual_size():
    with open("/sys/class/graphics/fb0/virtual_size", "r") as f:
        width, height = map(int, f.read().strip().split(","))
        return width, height

w, h = get_virtual_size()
print(f"Virtual screen size: {w}x{h}")
