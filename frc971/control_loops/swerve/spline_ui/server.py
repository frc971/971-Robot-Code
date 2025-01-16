from flask import Flask, send_from_directory, request, jsonify
import mimetypes

import trajectory_solver

mimetypes.init()

app = Flask(__name__)

static_path = "www/static_files/"


# Serve web contents.
@app.route('/<path:path>', methods=["GET"])
def static_proxy(path):
    path = path.split("?")[0]
    # For some reason .map files aren't in the default python mimetypes.
    if (path[-4:] == ".map"):
        return send_from_directory(static_path,
                                   path,
                                   mimetype="application/json")
    # Send the correct mimetype - not sure why this isn't automatically part of the send_from_directory function.
    mt = mimetypes.types_map["." + path.split(".")[-1]]
    return send_from_directory(static_path, path, mimetype=mt)


@app.route('/', methods=["GET"])
def index():
    return send_from_directory(static_path, "index.html")


# Respond with the optimized trajectory.
@app.route('/api/solve', methods=["POST"])
def api():
    reqdata = request.json
    paths = reqdata["paths"]
    global_constraints = reqdata["global_constraints"]
    sols = trajectory_solver.solve(paths, global_constraints)
    return jsonify(sols)


@app.errorhandler(500)
def server_error(e):
    return 'An internal error occurred [server.py] %s' % e, 500


if __name__ == '__main__':
    app.run(host='localhost', port=1180, debug=True)
