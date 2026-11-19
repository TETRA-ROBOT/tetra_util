"""웹 관제 API 가 중계하는 로컬 HTTP 인터페이스 (127.0.0.1 에만 연다). tetra_camera_calibration 과 같은 방식이다."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from typing import Dict, Optional, Protocol

MAX_BODY_BYTES = 4096
COMMANDS = {'/straight', '/rotation', '/stop', '/apply', '/restore-default', '/clear'}


class CalibratorCommands(Protocol):
    def status(self) -> Dict[str, object]: ...
    def command(self, name: str, body: Dict[str, object]) -> Dict[str, object]: ...
    def touch(self) -> None: ...


def make_handler(commands: CalibratorCommands) -> type:

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: object) -> None:  # 요청마다 찍지 않는다
            return

        def do_GET(self) -> None:
            commands.touch()
            if self.path == '/status':
                self._json(200, commands.status())
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

    return Handler


def start_server(port: int, commands: CalibratorCommands) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(commands))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name='odom-calibration-http', daemon=True).start()
    return server
