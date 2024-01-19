

from .database_connector import SqliteConnector
from .backend_writer import CloudBackendWriter
import logging 
import pickle 
import dm_env
from envlogger import step_data
# based on dataset configuration 
# read sql database
# export to rlds format 

class DatasetExporter():
    def __init__(self, config):
        self.config = config
        self.dataset_name = self.config.dataset_name
        self.dataset_config = self.config.get_rlds_dataset_config()
        self.logger =  logging.getLogger(__name__) 
        self.writer = CloudBackendWriter(
            data_directory=self.config.save_path,
            max_episodes_per_file=1,
            ds_config=self.dataset_config,
            logger=self.logger,
            metadata_database=None,
        )
        self.storage_backend = SqliteConnector("fogros_rt_x.db")
        self.episode_id = None 

    def execute(self):
        # query database for all episodes that should_export is not 0
        results = self.storage_backend.query_data_with_condition(
            table_name=self.dataset_name,
            columns=["episode_id", "step", "observation", "action"],
            conditions={"should_export": 1},
            use_or = True,
        )
        
        # write each step to rlds format
        for i in range(len(results)):
            result = results[i]
            episode_id = result[0]
            step = pickle.loads(result[1])
            observation = pickle.loads(result[2])
            action = pickle.loads(result[3])
            
            # figure out the step_type 
            # if the episode_id is different from the next one, 
            # then the step_type is LAST
            # if it is different from the previous one, it is FIRST
            # otherwise it is MID
            if i == len(results) - 1:
                step_type = "LAST"
            elif i == 0:
                step_type = "FIRST"
            else:
                if episode_id != results[i-1][0]:
                    step_type = dm_env.StepType.FIRST
                elif episode_id != results[i+1][0]:
                    step_type = dm_env.StepType.LAST
                else:
                    step_type = dm_env.StepType.MID
            

            timestep = dm_env.TimeStep(
                step_type=step_type,
                reward=step["reward"],
                discount=step["discount"],
                observation=observation,
            )

            data = step_data.StepData(
                timestep=timestep, 
                action=action, 
                custom_data=None
            )

            self.logger.info(data)

            if step_type == dm_env.StepType.FIRST:
                self.writer.record_step(data, is_new_episode=True)
            else:
                self.writer.record_step(data, is_new_episode=False)
        
        # close the writer
        self.writer.close()