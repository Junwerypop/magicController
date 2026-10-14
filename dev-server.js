const fs = require("node:fs");
const http = require("node:http");
const path = require("node:path");

const WEB_ROOT = path.join(__dirname, "web");
const ROOT_PREFIX = WEB_ROOT + path.sep;
const PORT = Number(process.env.PORT || 3000);
const MIME_TYPES = {
  ".css": "text/css; charset=utf-8",
  ".html": "text/html; charset=utf-8",
  ".ico": "image/x-icon",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".svg": "image/svg+xml",
};

http.createServer((req, res) => {
  const startedAt = Date.now();
  let status = 200;
  let pathname;

  if (req.method !== "GET" && req.method !== "HEAD") {
    res.writeHead(405, { "Allow": "GET, HEAD", "Content-Type": "text/plain; charset=utf-8" });
    res.end("Method not allowed");
    console.log(`${req.method} ${req.url} 405 in ${Date.now() - startedAt}ms`);
    return;
  }

  // Browsers may request this implicitly even when the page declares a data URI icon.
  if (req.url.split("?", 1)[0] === "/favicon.ico") {
    res.writeHead(204, { "Cache-Control": "no-store" });
    res.end();
    console.log(`${req.method} /favicon.ico 204 in ${Date.now() - startedAt}ms`);
    return;
  }

  try {
    pathname = decodeURIComponent(new URL(req.url, "http://localhost").pathname);
  } catch {
    status = 400;
  }

  let filePath;
  if (status === 200) {
    filePath = pathname === "/"
      ? path.join(WEB_ROOT, "index.html")
      : path.resolve(WEB_ROOT, `.${pathname}`);
    if (filePath !== path.join(WEB_ROOT, "index.html") && !filePath.startsWith(ROOT_PREFIX)) {
      status = 403;
    }
  }

  const finish = (body = "") => {
    res.writeHead(status, {
      "Cache-Control": "no-store",
      "Content-Type": "text/plain; charset=utf-8",
    });
    res.end(body);
    console.log(`${req.method} ${pathname || req.url} ${status} in ${Date.now() - startedAt}ms`);
  };

  if (status !== 200) {
    finish(status === 400 ? "Bad request" : "Forbidden");
    return;
  }

  fs.stat(filePath, (statError, stat) => {
    if (statError || !stat.isFile()) {
      status = 404;
      finish("Not found");
      return;
    }

    status = 200;
    res.writeHead(status, {
      "Cache-Control": "no-store",
      "Content-Length": stat.size,
      "Content-Type": MIME_TYPES[path.extname(filePath).toLowerCase()] || "application/octet-stream",
    });
    if (req.method === "HEAD") res.end();
    else fs.createReadStream(filePath).pipe(res);
    console.log(`${req.method} ${pathname} ${status} in ${Date.now() - startedAt}ms`);
  });
}).listen(PORT, "127.0.0.1", () => {
  console.log(`Magic Wand Controller available at http://localhost:${PORT}`);
});
