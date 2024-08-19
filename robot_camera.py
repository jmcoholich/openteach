import hydra
from openteach.components import RealsenseCameras
import time

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'camera')
def main(configs):
    cameras = RealsenseCameras(configs)
    processes = cameras.get_processes()

    for process in processes:
        process.start()
        time.sleep(5)  # this prevents errors resulting from processes blocking each other's access to the realsense cameras

    for process in processes:
        process.join()
        time.sleep(5)

if __name__ == '__main__':
    main()