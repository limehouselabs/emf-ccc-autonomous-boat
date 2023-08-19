from flask import Flask, render_template, request, redirect, Response
import os, string, logging
import time

app = Flask(__name__)

location = (53.03415, 13.30660)
heading = 0
targets = []
last_update = None

@app.route('/boat', methods=['GET'])
def index():
    return render_template('index.html')

@app.route('/boat/update', methods=['POST'])
def update():
    global location, heading, last_update
    data = request.json
    location = (data["lat"], data["lon"])
    heading = data["heading"]
    if heading < 0:
        print(heading)
        heading = 360 - -heading
    last_update = time.time()
    print(location, heading)
    return {"targets": targets}

@app.route('/boat/get_status', methods=['GET'])
def get_status():
    return {"lat": location[0], "lon": location[1], "heading": heading, "targets": targets}

@app.route('/boat/add_target', methods=['POST'])
def add_target():
    global targets
    targets.append((request.json["lat"], request.json["lon"]))
    print(targets)
    return {"status": True}

@app.route('/boat/remove_target', methods=['POST'])
def remove_target():
    global targets
    try:
        targets.remove((request.json["lat"], request.json["lon"]))
    except Exception as e:
        pass

    return {"status": True}

@app.route('/boat/clear_targets', methods=['GET'])
def clear_targets():
    global targets
    targets = []
    return {"status": True}
