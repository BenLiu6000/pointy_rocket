from __future__ import annotations

import json
import secrets
import sys
import time
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path

from openrocket_import import import_rocket_body
from rocketpy_adapter import parse_motor_curve, run_simulation


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
CSRF_TOKEN = secrets.token_urlsafe(32)
MAX_BODY_BYTES = 1_200_000
REQUESTS = {}


class GuiHandler(BaseHTTPRequestHandler):
    server_version = "PointyRocketGui/1"

    def do_GET(self):
        if self.path == "/":
            self.send_html((STATIC_DIR / "index.html").read_text().replace("__CSRF_TOKEN__", CSRF_TOKEN))
            return
        if self.path == "/static/app.css":
            self.send_file(STATIC_DIR / "app.css", "text/css; charset=utf-8")
            return
        if self.path == "/static/app.js":
            self.send_file(STATIC_DIR / "app.js", "application/javascript; charset=utf-8")
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        if self.path not in ("/api/simulate", "/api/fetch-motor", "/api/import-body"):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not self.allowed_request():
            self.send_json({"error": "Too many requests."}, HTTPStatus.TOO_MANY_REQUESTS)
            return
        if self.headers.get("X-CSRF-Token") != CSRF_TOKEN:
            self.send_json({"error": "Invalid CSRF token."}, HTTPStatus.FORBIDDEN)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_json({"error": "Invalid request length."}, HTTPStatus.BAD_REQUEST)
            return
        if length <= 0 or length > MAX_BODY_BYTES:
            self.send_json({"error": "Request body is too large."}, HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
            return
        try:
            payload = json.loads(self.rfile.read(length))
            if self.path == "/api/fetch-motor":
                self.send_json(fetch_motor(payload))
                return
            if self.path == "/api/import-body":
                self.send_json(import_rocket_body(str(payload.get("fileName", ""))[:160], str(payload.get("fileContent", ""))))
                return
            file_name = str(payload.get("fileName", ""))[:160]
            file_content = str(payload.get("fileContent", ""))
            specs = payload.get("specs", {})
            if not isinstance(specs, dict):
                raise ValueError("Specs payload must be an object.")
            curve = parse_motor_curve(file_name, file_content)
            self.send_json(run_simulation(curve, specs))
        except Exception as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)

    def allowed_request(self):
        now = time.monotonic()
        host = self.client_address[0]
        recent = [stamp for stamp in REQUESTS.get(host, []) if now - stamp < 60]
        if len(recent) >= 30:
            REQUESTS[host] = recent
            return False
        recent.append(now)
        REQUESTS[host] = recent
        return True

    def send_html(self, content):
        self.send_response(HTTPStatus.OK)
        self.common_headers("text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(content.encode())

    def send_file(self, path, content_type):
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.common_headers(content_type)
        self.end_headers()
        self.wfile.write(path.read_bytes())

    def send_json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.common_headers("application/json; charset=utf-8")
        self.end_headers()
        self.wfile.write(body)

    def common_headers(self, content_type):
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Strict-Transport-Security", "max-age=31536000; includeSubDomains")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; form-action 'self'; base-uri 'none'; frame-ancestors 'none'",
        )

    def log_message(self, format_value, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), format_value % args))


def main():
    port = 8765
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    server = ThreadingHTTPServer(("127.0.0.1", port), GuiHandler)
    print(f"Pointy Rocket GUI running at http://127.0.0.1:{port}")
    server.serve_forever()


def fetch_motor(payload):
    url = str(payload.get("url", "")).strip()
    if len(url) > 500:
        raise ValueError("Motor URL is too long.")
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme != "https" or not parsed.hostname or not parsed.hostname.endswith("thrustcurve.org"):
        raise ValueError("Use an HTTPS thrustcurve.org motor file URL.")
    request = urllib.request.Request(url, headers={"User-Agent": "PointyRocketSimulator/1"})
    with urllib.request.urlopen(request, timeout=12) as response:
        final = urllib.parse.urlparse(response.geturl())
        if final.scheme != "https" or not final.hostname or not final.hostname.endswith("thrustcurve.org"):
            raise ValueError("Motor download redirected outside thrustcurve.org.")
        content = response.read(MAX_BODY_BYTES + 1)
    if len(content) > MAX_BODY_BYTES:
        raise ValueError("Fetched motor file is too large.")
    file_name = Path(final.path).name or "thrustcurve_motor.eng"
    return {"fileName": file_name[:160], "fileContent": content.decode("utf-8", errors="replace")}


if __name__ == "__main__":
    main()
