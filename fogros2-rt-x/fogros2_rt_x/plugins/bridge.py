

from .conf_base import *
from .orchestrator_base import PerPeriodTopicOrchestrator

# bridge dataset
class BridgeDatasetConfig(BaseDatasetConfig):
    def __init__(self):
        # the name of the dataset
        DATASET_NAME = "bridge"
        # the path can be a local directory or a google cloud storage bucket
        SAVE_PATH = "/home/ubuntu/open-x-embodiment/playground_ds"
        # this is used for bigquery project metadata storage
        # the project should be accessible through google cloud account
        BIG_QUERY_PROJECT = "fogros2-rt-x"
        OBSERVATION_SPEC = [
            FeatureSpec("image", Image(shape=(480, 640, 3), dtype=tf.uint8)),
            FeatureSpec("natural_language_embedding", Tensor(shape=(512,), dtype=tf.float32)),
            FeatureSpec("natural_language_instruction", Text()),
            FeatureSpec("state", Tensor(shape=(7,), dtype=tf.float32)),
        ]

        ACTION_SPEC = [
            FeatureSpec("open_gripper", Scalar(dtype=tf.bool)),
            FeatureSpec("rotation_delta", Tensor(shape=(3,), dtype=tf.float32), is_triggering_topic=True),
            FeatureSpec("terminate_episode", Scalar(dtype=tf.float64)),
            FeatureSpec("world_vector", Tensor(shape=(3,), dtype=tf.float32)),
        ]

        STEP_SPEC = [
            FeatureSpec("reward", Scalar(dtype=tf.float64)),
            FeatureSpec("discount", Scalar(dtype=tf.float64), default_value=0.0),
            FeatureSpec("is_first", Scalar(dtype=tf.bool)),
            FeatureSpec("is_last", Scalar(dtype=tf.bool)),
            FeatureSpec("is_terminal", Scalar(dtype=tf.bool)),
        ]

        super().__init__(
            dataset_name=DATASET_NAME,
            save_path=SAVE_PATH,
            big_query_project=BIG_QUERY_PROJECT,
            observation_spec=OBSERVATION_SPEC,
            action_spec=ACTION_SPEC,
            step_spec=STEP_SPEC,
        )

def GET_CONFIG():
    return BridgeDatasetConfig()

def GET_ORCHESTRATOR():
    return PerPeriodTopicOrchestrator(GET_CONFIG())