from .conf_base import *
from .orchestrator_base import PerPeriodTopicOrchestrator


# bridge dataset
class BridgeDatasetConfig(BaseDatasetConfig):
    def __init__(self):
        # the name of the dataset
        # need to match the file name in the plugins folder
        DATASET_NAME = "bridge"
        # the path can be a local directory or a google cloud storage bucket
        # it can also be a google cloud storage bucket
        # SAVE_PATH = "gs://open-x-embodiment/playground_ds"
        SAVE_PATH = "/home/ubuntu/open-x-embodiment/playground_ds"
        OBSERVATION_SPEC = [
            FeatureSpec("image", Image(shape=(480, 640, 3), dtype=tf.uint8)),
            FeatureSpec(
                "natural_language_embedding", Tensor(shape=(512,), dtype=tf.float32)
            ),
            FeatureSpec("natural_language_instruction", Text()),
            FeatureSpec("state", Tensor(shape=(7,), dtype=tf.float32)),
        ]

        ACTION_SPEC = [
            FeatureSpec("open_gripper", Scalar(dtype=tf.bool)),
            FeatureSpec(
                "rotation_delta",
                Tensor(shape=(3,), dtype=tf.float32),
                is_triggering_topic=True,
            ),
            FeatureSpec("terminate_episode", Scalar(dtype=tf.float64)),
            FeatureSpec("world_vector", Tensor(shape=(3,), dtype=tf.float32)),
        ]

        STEP_SPEC = [
            FeatureSpec("reward", Scalar(dtype=tf.float64)),
            FeatureSpec("discount", Scalar(dtype=tf.float64), default_value=0.0),
        ]

        super().__init__(
            dataset_name=DATASET_NAME,
            save_path=SAVE_PATH,
            observation_spec=OBSERVATION_SPEC,
            action_spec=ACTION_SPEC,
            step_spec=STEP_SPEC,
        )


# MUST INCLUDE: GET_CONFIG, GET_ORCHESTRATOR
# GET_CONFIG returns the dataset config
# GET_ORCHESTRATOR returns the orchestrator
def GET_CONFIG():
    return BridgeDatasetConfig()


def GET_ORCHESTRATOR():
    return PerPeriodTopicOrchestrator(GET_CONFIG())
