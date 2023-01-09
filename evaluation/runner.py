from __future__ import print_function
from tools.parsers import ParseFrontendMetrics
import os

class Runner(object):

    def __init__(self, executable_path, output_path, dataset_path, param_file, vdo_slam_executable = "vdo_slam_kitti") -> None:
        self.executable_path = os.path.abspath(executable_path)
        self.output_path = os.path.abspath(output_path)
        self.dataset_path = os.path.abspath(dataset_path)
        self.param_file = os.path.abspath(param_file)
        self.vdo_slam_executable = vdo_slam_executable



    def spin(self):
        import threading
        import time
        import subprocess

        def vdo_slam_thread(thread_return):
            full_executable = self.executable_path + "/" + self.vdo_slam_executable
            print(full_executable)
            # print("Running executable at {}")
            command = f" cd {self.executable_path} && ./{self.vdo_slam_executable} \
                {self.param_file} {self.dataset_path} \
                --output_path={self.output_path} \
                --logtostderr=1 \
                --colorlogtostderr=1 \
                --log_prefix=1 \
                --v=7"
            return_code = subprocess.call(command, shell=True)
            if return_code is 0:
                thread_return['success'] = True
            else:
                thread_return['success'] = False

        thread_return={'success': False}
        thread = threading.Thread(target=vdo_slam_thread, args=(thread_return,))
        thread.start()

        while thread.is_alive():
            time.sleep(0.100)
        thread.join()
        return thread_return['success']
        


def run(args):
    executable_path = os.path.abspath("../build")
    output_logs_folder = os.path.abspath("../output_logs") #the default location

    if args.output_logs:
        output_logs_folder =os.path.abspath(args.output_logs)

    if args.dataset_path and args.param_file:
        result = Runner( executable_path, output_logs_folder, args.dataset_path, args.param_file).spin()
        print(f"Dataset was run with status {result}")
    elif args.dataset_path or args.param_file:
        print("Args must be run with both -d/--dataset_path and -p/--param_file")
        return

    if args.plot:
        print(f"Running plots from {output_logs_folder}")
        frontend_metrics = ParseFrontendMetrics(output_logs_folder)
        frontend_metrics.plot()

def parser():
    import argparse
    basic_desc = "VDO-SLAM plotter and metric app"

    shared_parser = argparse.ArgumentParser(add_help=True, description="{}".format(basic_desc))

    # input_opts = shared_parser.add_argument_group("input options")
    input_opts = shared_parser.add_argument_group("input options")
    output_opts = shared_parser.add_argument_group("output options")

    input_opts.add_argument("-d", "--dataset_path",
                                 help="Absoltute or relative path to dataset")
    input_opts.add_argument("-p", "--param_file",
                                 help="Absolute or relative path to param file")

    output_opts.add_argument(
        "--plot", action="store_true", help="show plot window",)
    output_opts.add_argument(
        "--output_logs", help="Where the output logs should be stored",)
    output_opts.add_argument("--save_plots", action="store_true",
                             help="Save plots?")
    output_opts.add_argument("--write_website", action="store_true",
                             help="Write website with results?")
    output_opts.add_argument("--save_boxplots", action="store_true",
                             help="Save boxplots?")
    output_opts.add_argument("--save_results", action="store_true",
                             help="Save results?")
    output_opts.add_argument("-v", "--verbose_sparkvio", action="store_true",
                             help="Make SparkVIO log all verbosity to console. Useful for debugging if a run failed.")

    main_parser = argparse.ArgumentParser(description="{}".format(basic_desc))
    sub_parsers = main_parser.add_subparsers(dest="subcommand")
    sub_parsers.required = True
    return shared_parser

import argcomplete
import sys
if __name__ == '__main__':
    # log.setLevel("INFO")
    parser = parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    # try:
    if run(args):
        sys.exit(os.EX_OK)
    # except Exception as e:
    #     print("error: ", e)
    #     raise Exception("Main evaluation run failed.")
