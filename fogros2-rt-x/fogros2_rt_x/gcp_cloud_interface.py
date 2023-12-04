

class GcpCloudInterface:
    def __init__(self, bucket_name):
        self.bucket_name = bucket_name
        self.storage_client = storage.Client()
        self.bucket = self.storage_client.bucket(bucket_name)

    def upload(self, source_file_name, destination_blob_name):
        blob = self.bucket.blob(destination_blob_name)
        blob.upload_from_filename(source_file_name)

    def download(self, source_blob_name, destination_file_name):
        blob = self.bucket.blob(source_blob_name)
        blob.download_to_filename(destination_file_name)

    def delete(self, blob_name):
        blob = self.bucket.blob(blob_name)
        blob.delete()

    def list_blobs(self):
        blobs = self.bucket.list_blobs()
        return blobs

    def list_blobs_with_prefix(self, prefix, delimiter=None):
        blobs = self.bucket.list_blobs(prefix=prefix, delimiter=delimiter)
        return blobs

    def list_blobs_with_delimiter(self, delimiter):
        blobs = self.bucket.list_blobs(delimiter=delimiter)
        return blobs

    def list_blobs_with_prefix_delimiter(self, prefix, delimiter):
        blobs = self.bucket.list_blobs(prefix=prefix, delimiter=delimiter)
        return blobs

    def list_blobs_with_prefix_delimiter(self, prefix, delimiter):
        blobs = self.bucket.list_blobs(prefix=prefix, delimiter=delimiter)
        return blobs

    def list_blobs_with_delimiter(self, delimiter):
        blobs = self.bucket.list_blobs(delimiter=delimiter)
        return blobs

    def list_blobs_with_prefix_delimiter(self, prefix, delimiter):
        blobs = self.bucket.list_blobs(prefix=prefix, delimiter=delimiter)
        return blobs

    def list_blobs_with_delimiter(self, delimiter):
        blobs = self.bucket.list_blobs(delimiter=delimiter)
        return blobs

    def list_blobs_with_prefix_delimiter(self, prefix, delimiter):
        blobs = self.bucket.list_blobs(prefix=prefix, delimiter=delimiter)
        return blobs

    def list_blobs_with_delimiter(self, delimiter):
        blobs = self.bucket.list_blobs(delimiter=delimiter)
        return blobs

    def list_blobs_with_prefix_delimiter(self, prefix, delimiter):
        blobs = self.bucket.list_blobs(prefix=prefix, delimiter=delimiter)
        return blobs

    def list_blobs_with_delimiter(self, delimiter):
        blobs = self.bucket.list_blobs(delimiter=delimiter)
        return blobs