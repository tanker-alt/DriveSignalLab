"""Capture current CARLA spectator view without advancing the world."""
import argparse
import queue
import carla

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output')
args = parser.parse_args()
client = carla.Client('localhost', 2000)
client.set_timeout(10)
world = client.get_world()
bp = world.get_blueprint_library().find('sensor.camera.rgb')
bp.set_attribute('image_size_x', '1200')
bp.set_attribute('image_size_y', '1000')
bp.set_attribute('fov', '80')
camera = world.spawn_actor(bp, world.get_spectator().get_transform())
frames = queue.Queue()
try:
    camera.listen(frames.put)
    frame = frames.get(timeout=15)
    frame.save_to_disk(args.output)
finally:
    camera.stop()
    camera.destroy()
