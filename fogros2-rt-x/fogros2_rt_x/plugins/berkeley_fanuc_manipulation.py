

from .orchestrator_examples import PerTimeIntervalTopicOrchestrator
observation_topics = ["/wrist_image", "/image", "/end_effector_state", "/state"]
action_topics = ["/action"]
step_topics = ["/language_embedding", "/language_instruction", "/discount", "/reward"]
orchestrator=PerTimeIntervalTopicOrchestrator(
    per_step_interval=0.1,  # seconds
    per_episode_interval=1.0, # seconds
)