from flask import Flask
from flask_cors import CORS
from glob import glob
import dataset
import os

app = Flask(__name__)
app.json.sort_keys = False
CORS(app)


@app.route("/db")
def get_dbs():
    return [os.path.basename(f) for f in glob("../data/*.db")]


@app.route("/db/<db_name>")
def get_tables(db_name):
    db = dataset.connect(f"sqlite:///{db_name}")
    print(db.tables)
    return db.tables


@app.route("/db/<db_name>/<table_name>")
@app.route("/db/<db_name>/<table_name>/<limit>")
def get_table_from_db(db_name, table_name, limit=10):
    db = dataset.connect(f"sqlite:///{db_name}")
    table = db.get_table(table_name)
    return list(table.all(_limit=limit))
