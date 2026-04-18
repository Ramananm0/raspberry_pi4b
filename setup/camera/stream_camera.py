#!/usr/bin/env python3
"""MJPEG camera stream — view at http://<pi-ip>:8080/"""
import sys, io, os, threading
sys.path.insert(0, "/usr/local/lib/python3/dist-packages")
sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")
os.environ["LIBCAMERA_IPA_MODULE_PATH"] = "/usr/local/lib/aarch64-linux-gnu/libcamera/ipa"

import unittest.mock as _mock
sys.modules.setdefault("pykms", _mock.MagicMock())
sys.modules.setdefault("kms",  _mock.MagicMock())

from http.server import BaseHTTPRequestHandler, HTTPServer
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

PORT = 8080


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<html><body style='margin:0;background:#000'>"
                b"<img src='/stream' style='width:100%;height:100vh;object-fit:contain'>"
                b"</body></html>"
            )
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )
            except Exception:
                pass
        elif self.path == "/snap":
            buf = io.BytesIO()
            cam.capture_file(buf, format="jpeg")
            data = buf.getvalue()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_response(404)
            self.end_headers()


cam = Picamera2()
cfg = cam.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},
    controls={"FrameRate": 15},
)
cam.configure(cfg)
output = StreamingOutput()
cam.start_recording(MJPEGEncoder(), FileOutput(output))

print(f"Live stream : http://localhost:{PORT}/")
print(f"Snapshot    : http://localhost:{PORT}/snap")

HTTPServer(("", PORT), Handler).serve_forever()
