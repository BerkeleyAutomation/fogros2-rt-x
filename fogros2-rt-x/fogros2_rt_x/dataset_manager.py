import logging
from .bag_manager import BagManager
from .backend_writer import CloudBackendWriter
from .conf_base import *


class DatasetManager:
    def __init__(
        self,
        orchestrator,
        dataset_directory,
        observation_topics,
        action_topics,
        step_topics,
    ):
        self.logger = logging.getLogger(__name__)
        self.dataset_directory = dataset_directory
        self.orchestrator = orchestrator

        self.bag_manager = BagManager(
            self.orchestrator,
            observation_topics,
            action_topics,
            step_topics,
        )
        (
            observation_spec,
            action_spec,
            step_spec,
        ) = self.bag_manager.generate_tensorflow_configuration_file()

        print(step_spec)
        self.config = BaseDatasetConfig(
            dataset_name="berkeley_fanuc_manipulation",
            save_path="/home/ubuntu/open-x-embodiment/playground_ds",  # TODO: take it down
            observation_spec=observation_spec,
            action_spec=action_spec,
            step_spec=step_spec,
        )

        self.writer = CloudBackendWriter(
            data_directory="/home/ubuntu/open-x-embodiment/playground_ds",
            max_episodes_per_file=1,
            ds_config=self.config.get_rlds_dataset_config(),
            logger=self.logger,
            metadata_database=None,
        )
        self.orchestrator.set_writer(self.writer)

        self.bag_manager.iterate_through_all_messages()
