#!/bin/sh
# Pre-deploy check, run on the computer from the repo root: every Python module compiles
# and every page script parses. A syntax error in a page script stops the whole page's
# scripts without any visible error; this caught a duplicate `const` in app.js.
#   sh tools/check-sources.sh
set -e
cd "$(dirname "$0")/.."
python3 - <<'EOF'
import pathlib, sys
bad = []
for f in sorted(pathlib.Path('python').glob('*.py')) + sorted(pathlib.Path('host').glob('*.py')):
    try:
        compile(f.read_text(encoding='utf-8'), str(f), 'exec')
    except SyntaxError as exc:
        bad.append(f'{f}: {exc}')
print('\n'.join(bad) or 'python ok')
sys.exit(1 if bad else 0)
EOF
if command -v node >/dev/null 2>&1; then
    for f in assets/*.js; do
        node -e "new Function(require('fs').readFileSync(process.argv[1], 'utf8'))" "$f" ||
            { echo "syntax error in $f"; exit 1; }
    done
    echo "page scripts ok"
else
    echo "node not found: page scripts not checked"
fi
