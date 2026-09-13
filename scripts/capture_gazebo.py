#!/usr/bin/env python3
"""Capture only a running Gazebo GUI window for local visual review."""
import argparse
import ctypes as C
import os
from pathlib import Path
import re
import subprocess
import time


def composite_capture(window):
    """Read the window's own backing pixmap, including under Xwayland."""
    from PIL import Image
    class XImage(C.Structure):
        _fields_ = [('width', C.c_int), ('height', C.c_int), ('xoffset', C.c_int),
                    ('format', C.c_int), ('data', C.c_void_p), ('byte_order', C.c_int),
                    ('bitmap_unit', C.c_int), ('bitmap_bit_order', C.c_int),
                    ('bitmap_pad', C.c_int), ('depth', C.c_int), ('bytes_per_line', C.c_int),
                    ('bits_per_pixel', C.c_int), ('red_mask', C.c_ulong),
                    ('green_mask', C.c_ulong), ('blue_mask', C.c_ulong)]
    x11, comp = C.CDLL('libX11.so.6'), C.CDLL('libXcomposite.so.1')
    x11.XOpenDisplay.argtypes, x11.XOpenDisplay.restype = [C.c_char_p], C.c_void_p
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Cannot open the X display.')
    comp.XCompositeNameWindowPixmap.argtypes = [C.c_void_p, C.c_ulong]
    comp.XCompositeNameWindowPixmap.restype = C.c_ulong
    pixmap = comp.XCompositeNameWindowPixmap(display, window)
    x11.XGetGeometry.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong)] + [C.POINTER(C.c_int)]*2 + [C.POINTER(C.c_uint)]*4
    root, x, y = C.c_ulong(), C.c_int(), C.c_int()
    width, height, border, depth = [C.c_uint() for _ in range(4)]
    x11.XGetGeometry(display, pixmap, C.byref(root), C.byref(x), C.byref(y),
                     C.byref(width), C.byref(height), C.byref(border), C.byref(depth))
    x11.XGetImage.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint, C.c_ulong, C.c_int]
    x11.XGetImage.restype = C.POINTER(XImage)
    captured = x11.XGetImage(display, pixmap, 0, 0, width.value, height.value, C.c_ulong(-1), 2)
    if not captured:
        raise RuntimeError('Cannot read the Gazebo backing pixmap.')
    info = captured.contents
    if info.bits_per_pixel != 32 or info.byte_order != 0:
        raise RuntimeError('Unsupported X image pixel format.')
    result = Image.frombytes('RGB', (info.width, info.height),
                             C.string_at(info.data, info.bytes_per_line*info.height),
                             'raw', 'BGRX', info.bytes_per_line)
    x11.XDestroyImage.argtypes = [C.POINTER(XImage)]
    x11.XDestroyImage(captured)
    x11.XFreePixmap.argtypes = [C.c_void_p, C.c_ulong]
    x11.XFreePixmap(display, pixmap)
    x11.XCloseDisplay.argtypes = [C.c_void_p]
    x11.XCloseDisplay(display)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='validation/building_desktop.png')
    parser.add_argument('--launch-log', help='Use the display of an isolated Gazebo validation launch.')
    parser.add_argument('--focus-model', help='Move the GUI camera to a model before capturing.')
    args = parser.parse_args()
    if args.launch_log:
        log = Path(args.launch_log).read_text()
        pid = re.search(r'\[gazebo-1\]: process started with pid \[(\d+)\]', log).group(1)
        for item in Path('/proc', pid, 'environ').read_bytes().split(b'\0'):
            if item.startswith((b'DISPLAY=', b'XAUTHORITY=', b'GZ_PARTITION=')):
                key, value = item.decode().split('=', 1)
                os.environ[key] = value
    if args.focus_model:
        from gz.transport13 import Node
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.boolean_pb2 import Boolean
        node = Node()
        ok, response = node.request('/gui/move_to', StringMsg(data=args.focus_model), StringMsg, Boolean, 3000)
        if not ok or not response.data:
            raise SystemExit('Gazebo did not accept the camera move.')
        time.sleep(2)
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    windows = re.findall(r'(0x[0-9a-f]+) "Gazebo Sim": \("mutter-x11-frames"', tree)
    composited = bool(windows)
    if not composited:
        windows = re.findall(r'(0x[0-9a-f]+) "Gazebo Sim": \("gz-sim-gui"', tree)
    if len(windows) != 1:
        raise SystemExit(f'Expected one Gazebo GUI window, found {len(windows)}.')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if composited:
        shot = composite_capture(int(windows[0], 16))
        if not shot.getbbox():
            raise SystemExit('The Gazebo window capture is empty.')
        shot.save(output)
    else:
        for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR'):
            if '/snap/' in os.environ.get(key, ''):
                os.environ.pop(key)
        os.environ['QT_QPA_PLATFORM'] = 'xcb'
        os.environ['QT_STYLE_OVERRIDE'] = 'Fusion'
        from PyQt5.QtWidgets import QApplication
        app = QApplication([])
        shot = app.primaryScreen().grabWindow(int(windows[0], 16))
        if shot.isNull() or not shot.save(str(output)):
            raise SystemExit('Cannot capture the Gazebo window.')
    print(output.resolve())


if __name__ == '__main__':
    main()
