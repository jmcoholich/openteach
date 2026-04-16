import hydra

from openteach.components import Collector


def join_processes(processes):
    try:
        for process in processes:
            process.join()
    except KeyboardInterrupt:
        print('\nSIGINT received. Waiting for recorder processes to finish saving...')
        try:
            for process in processes:
                process.join()
        except KeyboardInterrupt:
            print('\nSecond SIGINT received. Terminating recorder processes; current recording may be incomplete.')
            for process in processes:
                if process.is_alive():
                    process.terminate()
            for process in processes:
                process.join()
            raise SystemExit(130)  # noqa: B904


@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'collect_data')
def main(configs):
    collector = Collector(configs, configs.demo_num)
    processes = collector.get_processes()

    for process in processes:
        process.start()

    join_processes(processes)

if __name__ == '__main__':
    main()
