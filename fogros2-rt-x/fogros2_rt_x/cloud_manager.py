

from .cloud_interfaces.gcp_cloud_interface import GcpCloudInterface
import os, sys

class CloudManager:
    def __init__(self, local_dataset_dir, cloud_dataset_dst):
        self.cloud_api_interface = GcpCloudInterface()
    
    def sync(self):
        # check the local dataset directory for new files
        # upload any new files to the cloud
        # download any new files from the cloud

        # listing blobs
        blobs = self.cloud_api_interface.list_blobs()
        for blob in blobs:
            print(blob.name)
        
        # check local directory
        local_dir_files = os.listdir(self.local_dataset_dir)
        for file in local_dir_files:
            print(file)
        
        to_upload = local_dir_files - blobs
        for file in to_upload:
            self.cloud_api_interface.upload(file, file)
