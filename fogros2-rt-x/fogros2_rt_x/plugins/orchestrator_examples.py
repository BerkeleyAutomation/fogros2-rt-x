from .orchestrator_base import BaseTopicOrchestrator

class StressTestTopicOrchestrator(BaseTopicOrchestrator):
    def __init__(self, reward=0.0, discount=1.0):
        super().__init__(reward, discount)

    def on_observation_topic(self, topic_name, data):
        super().on_observation_topic(topic_name, data)
        self._new_step()

    def on_action_topic(self, topic_name, data):
        super().on_action_topic(topic_name, data)
        self._new_step()

    def on_step_topic(self, topic_name, data):
        super().on_step_topic(topic_name, data)
        self._new_episode()


class PerTimeIntervalTopicOrchestrator(BaseTopicOrchestrator):
    def __init__(
        self, per_step_interval=0.1,  # seconds
        per_episode_interval=1.0, # seconds
        reward=0.0, 
        discount=1.0
    ):
        super().__init__(reward, discount)
        self.last_timestamp = None
        self.last_episode_timestamp = None
        self.per_step_interval = per_step_interval * 1e9
        self.per_episode_interval = per_episode_interval * 1e9

    def on_timestamp(self, timestamp, topic_name):
        super().on_timestamp(timestamp, topic_name)
        if (
            topic_name not in self.step
            and topic_name not in self.action
            and topic_name not in self.observation
        ):
            return

        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            self.last_episode_timestamp = timestamp
            return

        if timestamp - self.last_timestamp > self.per_step_interval:
            self._new_step()
            self.last_timestamp = timestamp
        if timestamp - self.last_episode_timestamp > self.per_episode_interval:
            self._new_episode()
            self.last_episode_timestamp = timestamp
