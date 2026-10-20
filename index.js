const { spawn } = require('child_process')
const os = require('os')
const path = require('path')

const schema = require('./schema')

const pkgData = require('./package.json')

module.exports = function (app) {
  let child
 return {
    start: options => {
      // Python virtualenv: SK_PY_BNO08X_VENV if set, otherwise ~/.env of the user running
      // Signal K (it was hard-coded to /home/pi/.env, which breaks for any other user and
      // had to be edited by hand after every install/update).
      const MY_PYTHON_ENV = process.env.SK_PY_BNO08X_VENV || path.join(os.homedir(), '.env')
      let MY_PYTHON = path.join(MY_PYTHON_ENV, 'bin', 'python3')
      const setError = msg => (typeof app.setPluginError === 'function' ? app.setPluginError(msg) : console.error(msg))
      child = spawn(MY_PYTHON, ['plugin.py'], { cwd: __dirname })
      const thisChild = child

      child.stdout.on('data', data => {
        // app.debug(data.toString())
        try {
          data.toString().split(/\r?\n/).forEach(line => {
            // console.log(JSON.stringify(line))
            if (line.length > 0) {
              app.handleMessage(undefined, JSON.parse(line))
            }
          })
        } catch (e) {
          console.error(e.message)
        }
      })

      child.stderr.on('data', fromChild => {
        console.error(fromChild.toString())
      })

      child.on('error', err => {
        console.error(err)
        setError('Cannot run ' + MY_PYTHON + ': ' + err.message)
      })

      child.on('exit', (code, signal) => {
        // only report when the child dies on its own (not after stop())
        if (child === thisChild && code !== 0 && code !== null) {
          setError('plugin.py exited with code ' + code + ' (see the server log)')
        }
      })

      // a child that failed to start (or died) must not crash the server with EPIPE
      child.stdin.on('error', err => console.error(err.message))
      child.stdin.write(JSON.stringify(options))
      child.stdin.write('\n')
    },
    stop: () => {
      if (child) {
	try {
          if (child.pid !== undefined && process.kill(child.pid, 0)) {
            process.kill(child.pid);
          }
        } catch (error) {
          console.error(error);
          child = undefined;
        }      
        child = undefined;
      }
    },
    schema,
    id: pkgData.name,
    name: pkgData.description,
    description:pkgData.signalk.displayName	
  }
}
