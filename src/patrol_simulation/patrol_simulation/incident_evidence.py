"""Save actual robot camera pixels and their geometric relation to a LiDAR incident."""
import math
from PIL import Image


def camera_projection(pose, location, width, height):
    dx, dy = location[0]-pose[0], location[1]-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    forward, left = c*dx+s*dy-0.43, -s*dx+c*dy
    if forward <= 0.1:
        return None
    focal = width/(2*math.tan(1.2/2))
    u, v = width/2-left*focal/forward, height/2+0.12*focal/forward
    if not (0 <= u < width and 0 <= v < height) or math.hypot(forward, left) > 30:
        return None
    return [round(u), round(v)]


def save_camera_frame(message, path):
    formats = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'),
               'rgba8': ('RGBA', 'RGBA'), 'bgra8': ('RGBA', 'BGRA'), 'mono8': ('L', 'L')}
    if message.encoding not in formats:
        raise ValueError(f'Unsupported camera encoding: {message.encoding}')
    mode, raw = formats[message.encoding]
    image = Image.frombytes(mode, (message.width, message.height), bytes(message.data),
                            'raw', raw, message.step, 1)
    image.save(path)
