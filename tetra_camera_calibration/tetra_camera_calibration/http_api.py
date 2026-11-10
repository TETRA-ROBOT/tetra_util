"""웹 관제 API 가 중계하는 로컬 HTTP 인터페이스. 127.0.0.1 에만 열어 로봇 밖에서는 직접 닿지 않는다.

ROS 서비스로 만들면 웹 API(rclnodejs) 쪽 메시지 정의가 늘어나서, 상태·명령은 JSON, 미리보기는 MJPEG 로 준다.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from typing import Dict, Optional, Protocol

BOUNDARY = 'tetracalibframe'
MAX_BODY_BYTES = 4096


class CalibratorCommands(Protocol):
    def status(self) -> Dict[str, object]: ...
    def latest_jpeg(self, after: int, timeout: float) -> Optional[tuple]: ...
    def command(self, name: str, body: Dict[str, object]) -> Dict[str, object]: ...
    def touch(self) -> None: ...


COMMANDS = {'/capture', '/calibrate', '/apply', '/reset', '/restore-default', '/auto'}


def make_handler(commands: CalibratorCommands, stop_event: threading.Event) -> type:

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: object) -> None:  # 요청마다 찍지 않는다
            return

        def do_GET(self) -> None:
            commands.touch()
            if self.path == '/status':
                self._json(200, commands.status())
            elif self.path == '/stream':
                self._stream()
            else:
                self._json(404, {'success': False, 'message': 'not found'})

        def do_POST(self) -> None:
            commands.touch()
            if self.path not in COMMANDS:
                self._json(404, {'success': False, 'message': 'not found'})
                return
            body = self._read_body()
            if body is None:
                self._json(400, {'success': False, 'message': 'invalid JSON body'})
                return
            result = commands.command(self.path.lstrip('/'), body)
            self._json(200 if result.get('success') else 409, result)

        def _read_body(self) -> Optional[Dict[str, object]]:
            length = int(self.headers.get('Content-Length') or 0)
            if length <= 0:
                return {}
            if length > MAX_BODY_BYTES:
                return None
            try:
                parsed = json.loads(self.rfile.read(length).decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                return None
            return parsed if isinstance(parsed, dict) else None

        def _json(self, code: int, payload: Dict[str, object]) -> None:
            data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header('Content-Type', f'multipart/x-mixed-replace;boundary={BOUNDARY}')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            sequence = 0
            try:
                while not stop_event.is_set():
                    frame = commands.latest_jpeg(sequence, 2.0)
                    if frame is None:
                        continue
                    sequence, jpeg = frame
                    commands.touch()
                    self.wfile.write(
                        f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


def start_server(port: int, commands: CalibratorCommands, stop_event: threading.Event) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(commands, stop_event))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name='calibration-http', daemon=True).start()
    return server

