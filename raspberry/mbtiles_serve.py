#!/usr/bin/env python3
import sys, os, re, sqlite3, json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

MB = sys.argv[1] if len(sys.argv)>1 else "map.mbtiles"
PORT = int(sys.argv[2]) if len(sys.argv)>2 else 8000
ROOT = os.getcwd()
if not os.path.exists(MB): print("MBTiles not found:", MB); sys.exit(1)

conn = sqlite3.connect(MB, check_same_thread=False); conn.row_factory=sqlite3.Row
cur = conn.cursor()
def meta(k, default=None):
    cur.execute("SELECT value FROM metadata WHERE name=?", (k,)); r=cur.fetchone()
    return r["value"] if r else default
FMT=(meta("format","png") or "png").lower(); MINZ=int(meta("minzoom","0")); MAXZ=int(meta("maxzoom","18"))
print("[MBTILES] format=",FMT,"zoom",MINZ,"~",MAXZ)

class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        p=urlparse(self.path).path
        m=re.match(r"^/tiles/(\d+)/(\d+)/(\d+)\.(?:png|jpg|jpeg)$", p)
        if m:
            z,x,y=map(int,m.groups())
            if z<MINZ or z>MAXZ: self.send_error(404,"zoom out of range"); return
            tms_y=(1<<z)-1-y
            cur.execute("SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?", (z,x,tms_y))
            row=cur.fetchone()
            if not row: self.send_error(404,"tile not found"); return
            data=row["tile_data"]; ctype="image/png" if FMT.startswith("png") else "image/jpeg"
            self.send_response(200); self.send_header("Content-Type",ctype); self.send_header("Content-Length",str(len(data)))
            self.end_headers(); self.wfile.write(data); return
        if p=="/tiles.json":
            body=json.dumps({"format":FMT,"minzoom":MINZ,"maxzoom":MAXZ}).encode()
            self.send_response(200); self.send_header("Content-Type","application/json"); self.send_header("Content-Length",str(len(body)))
            self.end_headers(); self.wfile.write(body); return
        return super().do_GET()

if __name__=="__main__":
    os.chdir(ROOT)
    httpd=ThreadingHTTPServer(("0.0.0.0",PORT), Handler)
    print(f"[SERVE] http://0.0.0.0:{PORT} (serving {MB} + static files)")
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
