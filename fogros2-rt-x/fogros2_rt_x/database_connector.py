class BaseDataBaseConnector:
    def __init__(self):
        pass

    def query(self, query_string):
        pass

    def insert(self, rows):
        pass


from google.cloud import bigquery
class BigQueryConnector(BaseDataBaseConnector):
    def __init__(self, project_name, dataset_name, table_name):
        self.project_name = project_name
        self.dataset_name = dataset_name
        self.table_name = table_name
        self.table_id = f"{project_name}.{dataset_name}.{table_name}"
        self.client = bigquery.Client(project=project_name)

    def query(self, query_string):
        query_job = self.client.query(query_string)
        return query_job.to_dataframe()

    def insert(self, rows):
        errors = self.client.insert_rows_json(self.table_id, rows)
        if errors:
            raise BigQueryError(errors)


'''
# Initialize the database class
db = SQLiteDB('example.db')

# Create a table
columns = {"name": "TEXT", "position": "TEXT", "salary": "REAL"}
db.create_table("employees", columns)

# Insert data
data = {"name": "John Doe", "position": "Manager", "salary": 80000.0}
db.insert_data("employees", data)

# Update data
updates = {"salary": 85000.0}
conditions = {"name": "John Doe"}
db.update_data("employees", updates, conditions)

# Query data
print(db.query_data("SELECT * FROM employees"))

# Close the database
db.close()

'''
import sqlite3
class SqliteConnector(BaseDataBaseConnector):
    def __init__(self, db_name):
        """Initialize the database connection"""
        self.connection = sqlite3.connect(db_name)
        self.cursor = self.connection.cursor()

    def execute_query(self, query, params=()):
        """Execute a general SQL query"""
        try:
            self.cursor.execute(query, params)
            self.connection.commit()
        except sqlite3.Error as e:
            print(f"An error occurred: {e}")

    def construct_create_table_query(self, table_name, columns):
        """Construct a CREATE TABLE query"""
        columns_with_types = ', '.join([f"{col_name} {data_type}" for col_name, data_type in columns.items()])
        return f"CREATE TABLE IF NOT EXISTS {table_name} (id INTEGER PRIMARY KEY, {columns_with_types})"

    def create_table(self, table_name, columns):
        """Create a table using the constructed query"""
        query = self.construct_create_table_query(table_name, columns)
        self.execute_query(query)

    def construct_insert_query(self, table_name, columns):
        """Construct an INSERT query"""
        placeholders = ', '.join(['?' for _ in columns])
        column_names = ', '.join(columns)
        return f"INSERT INTO {table_name} ({column_names}) VALUES ({placeholders})"

    def insert_data(self, table_name, data):
        """Insert data into a table"""
        columns = data.keys()
        insert_query = self.construct_insert_query(table_name, columns)
        self.execute_query(insert_query, tuple(data.values()))

    def query_data(self, query, params=()):
        """Query data from the database"""
        self.cursor.execute(query, params)
        return self.cursor.fetchall()

    def construct_update_query(self, table_name, columns, conditions):
        """Construct an UPDATE query"""
        set_clause = ', '.join([f"{col} = ?" for col in columns])
        where_clause = ' AND '.join([f"{cond} = ?" for cond in conditions])
        return f"UPDATE {table_name} SET {set_clause} WHERE {where_clause}"

    def update_data(self, table_name, updates, conditions):
        """Update data in the database"""
        update_query = self.construct_update_query(table_name, updates.keys(), conditions.keys())
        self.execute_query(update_query, tuple(updates.values()) + tuple(conditions.values()))

    def add_column(self, table_name, column, data_type):
        """Add a new column to a table"""
        alter_query = f"ALTER TABLE {table_name} ADD COLUMN {column} {data_type}"
        self.execute_query(alter_query)

    def close(self):
        """Close the database connection"""
        self.connection.close()