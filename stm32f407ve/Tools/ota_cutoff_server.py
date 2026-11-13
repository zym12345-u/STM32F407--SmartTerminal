"""OTA 断网测试服务器：app.meta 正常，app.bin 只发指定字节后关闭连接。"""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class CutoffHandler(SimpleHTTPRequestHandler):
    cut_bytes = 64 * 1024

    def copyfile(self, source, outputfile):
        if self.path.split("?", 1)[0] != "/app.bin":
            return super().copyfile(source, outputfile)

        data = source.read(self.cut_bytes)
        outputfile.write(data)
        outputfile.flush()
        self.close_connection = True
        self.log_message("[CUT] app.bin sent %d bytes, closing TCP", len(data))
        return None


def main():
    parser = argparse.ArgumentParser(description="模拟 OTA 固件下载中途断网")
    parser.add_argument("--directory", required=True, help="包含 app.bin/app.meta 的目录")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--cut-bytes", type=int, default=64 * 1024)
    args = parser.parse_args()

    directory = Path(args.directory).resolve()
    image_size = (directory / "app.bin").stat().st_size
    if args.cut_bytes <= 0 or args.cut_bytes >= image_size:
        raise SystemExit(f"cut-bytes 必须在 1～{image_size - 1} 之间")

    CutoffHandler.cut_bytes = args.cut_bytes
    handler = partial(CutoffHandler, directory=str(directory))
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    print(f"[CUT] directory={directory}", flush=True)
    print(f"[CUT] app.bin size={image_size}, send only {args.cut_bytes} bytes", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
