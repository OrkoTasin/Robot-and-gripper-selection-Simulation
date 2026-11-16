"""Dependency-free HTTP entry point for the local and Vercel deployments."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from robot import HOME, PARTS, TRAY, JAW, JOINT_LABELS, camera, grip, move, pose, release, setup
from selection import DEFAULT, TOOLS, recommend

ROOT = Path(__file__).resolve().parent


def cell(data):
    model, tool = setup(data)
    return dict(home=HOME, pose=pose(HOME,model), parts=PARTS, tray=TRAY, jaw=JAW,
                model=model, tool_id=tool, tool=TOOLS[tool], joint_labels=JOINT_LABELS[model])


ACTIONS = {"/api/move": move, "/api/camera": camera, "/api/grip": grip, "/api/release": release,
           "/api/recommend": recommend, "/api/setup": cell}
FILES = {
    "/": ("index.html", "text/html"),
    "/app.js": ("app.js", "text/javascript"),
    "/style.css": ("style.css", "text/css"),
    "/robots.csv": ("robots.csv", "text/csv"),
    "/selection-ui.js": ("selection-ui.js", "text/javascript"),
}


class handler(BaseHTTPRequestHandler):
    def route(self):
        url = urlsplit(self.path)
        if url.path in ("/api","/api/","/api/index"):
            action = parse_qs(url.query).get("action",["cell"])[0]
            return "/api/" + action
        return url.path

    def send(self, status, content, mime="application/json"):
        payload = content if isinstance(content, bytes) else json.dumps(content).encode()
        self.send_response(status)
        self.send_header("Content-Type", mime + "; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store" if mime == "application/json" else "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(payload)

    def do_GET(self):
        route = self.route()
        if route == "/api/cell":
            self.send(200, dict(cell({}), defaults=DEFAULT))
        elif route in FILES:
            file, mime = FILES[route]
            self.send(200, (ROOT/"public"/file).read_bytes(), mime)
        else:
            self.send(404, {"error": "Page not found."})

    do_HEAD = do_GET

    def do_POST(self):
        action = ACTIONS.get(self.route())
        if action is None:
            self.send(404, {"error": "Unknown command."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("Invalid request length.")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError("Expected a JSON object.")
            result = action(data)
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            self.send(422, {"error": str(exc)})
            return
        self.send(200, {"result": result})


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser(description="Run the local robot workcell.")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    print(f"Simulation running at http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()
