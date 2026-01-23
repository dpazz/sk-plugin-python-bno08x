"""
MIT License

Copyright (c) 2026 dpazz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

import sys, json, threading;

n = 0
def outputSk():
    global n
    skData = {'updates': [{ 'values': [{'path': 'some.path', 'value': n}]}]}
    sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData))
    sys.stdout.write('\n')
    sys.stdout.flush()
    n += 1
    threading.Timer(1.0, outputSk).start()

threading.Timer(1.0, outputSk).start()

for line in iter(sys.stdin.readline, b''):
    try:
        data = json.loads(line)
        sys.stderr.write(json.dumps(data))
    except:
        sys.stderr.write('error parsing json\n')
        sys.stderr.write(line)

