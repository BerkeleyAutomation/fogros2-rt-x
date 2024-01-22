


from .bag_manager import BagManager

class DatasetManager():
    def __init__(
        self,
        dataset_directory, 
        observation_topics, 
        action_topics,
        step_topics,
    ):
        self.dataset_directory = dataset_directory
        self.bag_manager = BagManager()
        self.configuration = self.bag_manager.generate_tensorflow_configuration_file(
            observation_topics,
            action_topics,
            step_topics,
        )
        print(self.configuration)
    
