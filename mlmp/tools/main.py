import sys
import argparse
import subprocess

def initParser() -> argparse.ArgumentParser:

    # should add the readme file to description
    parser = argparse.ArgumentParser(prog=__name__,
                                     description='Interface for using the mlmp exe')
    parser.add_argument('--input-file,i',
                        help="input file to run on", type=str, required=True)
    parser.add_argument('--time-limit,t',
                        help="run time limit", type=int, default=1)
    parser.add_argument('--verbose,v',
                        help="verbose", type=bool, default=False)
    parser.add_argument('--algo,a', 
                        help="algorithm in use", type=str, default="BiQRRT")

    return parser


def runJob(args: list):
    
    executable_path = '../src/mlmp'  
    
    
    result = subprocess.run([executable_path] + args, capture_output=True, text=True)
    
    if result.returncode == 0:
        print("Execution successful!")
        print("Output:")
        print(result.stdout)
    else:
        print("Execution failed!")
        print("Error:")
        print(result.stderr)



def main():
    parser = initParser() 
    args = parser.parse_args()

    runJob(['--input-file', '../src/inputs/experiment/{}'.format(args.input_file), 
            '--time-limit', str(args.time_limit),
            '--v', '1' if args.v else '0',
            '--algo', args.algo])


if __name__ == "__main__":
    rc = 1
    try:
        main()
        rc = 0
    except Exception as e:
        print('Error: %s' % e, file=sys.stderr)
    sys.exit(rc)