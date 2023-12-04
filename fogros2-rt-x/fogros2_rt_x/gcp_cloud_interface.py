# Copyright 2022 The Regents of the University of California (Regents)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


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
