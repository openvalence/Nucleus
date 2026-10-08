// serve.mjs -- the lab's static server: this folder, the twin's kinetic.wasm, the Phosphor player's
// modules (verbatim, from the sibling checkout) and the Nucleus trace fixture.
//
//   node tools/kinetic-lab/serve.mjs [port]      (port 0 or absent: a free one; the URL is printed)
//
// Constraints:
// - Serves only what the page needs; no directory listing, no path outside the four roots.
// - Opens nothing: the operator opens the printed URL (host rule: no focus-stealing windows).
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { join, resolve, extname } from 'node:path';

const here = fileURLToPath(new URL('.', import.meta.url));
const nucleus = resolve(here, '..', '..');
const ROOTS = {
  '/': { dir: here, files: new Set(['index.html', 'lab.js', 'lab.css', 'host.js']) },
  '/player/': { dir: resolve(nucleus, '..', 'Phosphor', 'plugins', 'factory', 'funscript-player'), files: new Set(['funscript.js', 'interp.js', 'scheduler.js']) },
  '/fixtures/': { dir: resolve(nucleus, 'test', 'fixtures'), files: new Set(['kinetic_trace.json']) },
};
const WASM = process.env.KINETIC_WASM || join(nucleus, 'tools', 'kinetic-wasm', 'build', 'kinetic.wasm');
const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8',
  '.json': 'application/json', '.wasm': 'application/wasm' };

export function serve(port = 0) {
  const server = createServer(async (req, res) => {
    const url = new URL(req.url, 'http://x');
    let path = url.pathname === '/' ? '/index.html' : url.pathname;
    let file = null;
    if (path === '/kinetic.wasm') file = WASM;
    else {
      for (const [prefix, root] of Object.entries(ROOTS)) {
        const name = path.startsWith(prefix) ? path.slice(prefix.length) : null;
        if (name && root.files.has(name)) { file = join(root.dir, name); break; }
      }
    }
    if (!file) { res.writeHead(404); res.end('not here'); return; }
    try {
      const body = await readFile(file);
      res.writeHead(200, { 'content-type': MIME[extname(file)] || 'application/octet-stream', 'cache-control': 'no-store' });
      res.end(body);
    } catch (e) {
      res.writeHead(404);
      res.end(e.code === 'ENOENT' ? `missing: ${file} (build the twin first, see README.md)` : String(e));
    }
  });
  return new Promise((ok) => server.listen(port, '127.0.0.1', () => ok({ server, url: `http://127.0.0.1:${server.address().port}/` })));
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const { url } = await serve(Number(process.argv[2]) || 0);
  console.log(`Kinetic² lab: ${url}`);
  console.log(`twin: ${WASM}`);
}
