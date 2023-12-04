
class BaseDataBaseConnector():
    def __init__(**kwargs):
        pass

    def query(query_string):
        pass

    def insert(rows):
        pass


class BigQueryConnector(BaseDataBaseConnector):
    def __init__(project_name, dataset_name, table_name):
        self.project_name = project_name
        self.dataset_name = dataset_name
        self.table_name = table_name
        self.table_id = f"{project_name}.{dataset_name}.{table_name}"
        self.client = bigquery.Client(project_name = project_name)

    def query(query_string):
        query_job = self.client.query(query_string)
        return query_job.to_dataframe()

    def insert(rows):
        errors = self.client.insert_rows_json(self.table_id, rows)
        if errors:
            raise BigQueryError(errors)