import sys

import hydra

from openteach.components import TeleOperator


def _normalize_control_mode_arg():
    """Allow argparse-style --control-mode while keeping Hydra overrides."""
    normalized_args = [sys.argv[0]]
    args = iter(sys.argv[1:])
    for arg in args:
        if arg in ("--control-mode", "control-mode"):
            try:
                normalized_args.append(f"control_mode={next(args)}")
            except StopIteration as exc:
                raise SystemExit("--control-mode requires a value") from exc
        elif arg.startswith("--control-mode="):
            normalized_args.append(f"control_mode={arg.split('=', 1)[1]}")
        elif arg.startswith("control-mode="):
            normalized_args.append(f"control_mode={arg.split('=', 1)[1]}")
        else:
            normalized_args.append(arg)
    sys.argv = normalized_args


@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'teleop')
def main(configs):
    teleop = TeleOperator(configs)
    processes = teleop.get_processes()

    for process in processes:
        process.start()

    try:
        for process in processes:
            process.join()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Waiting for teleop processes to shut down...")
        for process in processes:
            process.join(timeout=30)
        for process in processes:
            if process.is_alive():
                process.terminate()
        for process in processes:
            process.join()

if __name__ == '__main__':
    _normalize_control_mode_arg()
    main()
