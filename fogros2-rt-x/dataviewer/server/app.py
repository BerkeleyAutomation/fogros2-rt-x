from flask import Flask, send_from_directory, request
from flask_cors import CORS
from glob import glob
import dataset
import os
from export_rlds import export_rlds, get_orchestrators

app = Flask(__name__)
app.json.sort_keys = False
CORS(app)

db_connection_prefix = "sqlite:///../data"


@app.route("/db")
def get_dbs():
    return [os.path.splitext(os.path.basename(f))[0] for f in glob("../data/*.db")]


@app.route("/db/<db_name>")
def get_tables(db_name):
    db = dataset.connect(f"{db_connection_prefix}/{db_name}.db")
    return db.tables


@app.route("/db/<db_name>/<table_name>")
@app.route("/db/<db_name>/<table_name>/<limit>")
@app.route("/db/<db_name>/<table_name>/<limit>/<page>")
def get_table_from_db(db_name, table_name, limit=50, page=0):
    sortby = request.args.get("sortby")
    order = sortby.split(",") if sortby else []
    db = dataset.connect(f"{db_connection_prefix}/{db_name}.db")
    table = db.get_table(table_name)
    query_res = db.query(f"SELECT COUNT(1) FROM {table_name}")
    count = next(query_res)["COUNT(1)"]
    return {
        "count": count,
        "data": list(
            table.all(_limit=int(limit), _offset=int(page) * int(limit), order_by=order)
        ),
    }


@app.route("/db/<db_name>/<table_name>/delete/<row_id>")
def delete_row(db_name, table_name, row_id):
    db = dataset.connect(f"{db_connection_prefix}/{db_name}.db")
    table = db.get_table(table_name)
    deleted = table.delete(id=int(row_id))
    return {"deleted": deleted}, 200


@app.route("/db/<db_name>/<table_name>/update/<row_id>")
def update_value(db_name, table_name, row_id):
    col = request.args.get("col")
    val = request.args.get("val")
    if col and val:
        db = dataset.connect(f"{db_connection_prefix}/{db_name}.db")
        table = db.get_table(table_name)
        data = {"id": int(row_id), col: val}
        updated = table.update(data, ["id"])
        return {"updated": updated}, 200
    return {}, 400


@app.route("/export/<dataset_name>/<destination>")
def export(dataset_name, destination):
    orchestrator = request.args.get("orchestrator")
    observation_topics = request.args.get("o").split(',')
    action_topics = request.args.get("a").split(',')
    step_topics = request.args.get("s").split(',')
    success = export_rlds(
        observation_topics, action_topics, step_topics, dataset_name, destination
    )
    if success:
        return {}, 200
    return {}, 400


@app.route("/export/options")
def get_export_options():
    observation_topics = ["/wrist_image", "/image", "/end_effector_state", "/state"]
    action_topics = ["/action"]
    step_topics = [
        "/language_embedding",
        "/language_instruction",
        "/discount",
        "/reward",
    ]
    return {
        "orchestrators": get_orchestrators(),
        "topics": {
            "observation_topics": observation_topics,
            "action_topics": action_topics,
            "step_topics": step_topics,
        },
    }


@app.route("/image/<path:path>")
def get_image(path):
    return send_from_directory("image", path)


@app.after_request
def after_request(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    return response
