
import numpy as np
import matplotlib.pyplot as plt
import sys
import subprocess
import re
from concurrent.futures import ThreadPoolExecutor, as_completed


executablePath = '../src/mlmp'
scenes = ['scene2.json']
algorithms = ['BiQRRT', 'QRRT', 'QRRTStar', 'QMP', 'QMPStar']
timeLimits = [1, 10, 30, 60, 120]
repetitions = 5
colors = ['blue', 'orange', 'green', 'red', 'purple']

# Function to run the algorithm
def runAlgorithm(algorithm, scene, timeLimit):
    print("run algorithm {} on {} for {} sec".format(algorithm, scene, timeLimit))
    args = ['--input-file', '../src/inputs/experiment/{}'.format(scene),
            '--algo', algorithm,
              '--time-limit', str(timeLimit)]
    result = subprocess.run([executablePath] + args, capture_output=True, text=True)
    if result.returncode == 0:
        output = result.stdout.strip()
        if "Not Found" in output:
            return None, None
        try:
            runtime = re.search(r'Runtime\n([\d.]+)', output).group(1)
            length = re.search(r'Length\n([\d.]+)', output).group(1)
            return float(runtime), float(length)
        except AttributeError:
            return None, None
    else:
        return None, None


def runExperiment():
    results = {scene: {algorithm: {time_limit: [] for time_limit in timeLimits} for algorithm in algorithms} for scene in scenes}
    
    with ThreadPoolExecutor() as executor:
        future_to_params = {
            executor.submit(runAlgorithm, algorithm, scene, time_limit): (scene, algorithm, time_limit)
            for scene in scenes
            for algorithm in algorithms
            for time_limit in timeLimits
            for _ in range(repetitions)
        }

        for future in as_completed(future_to_params):
            scene, algorithm, time_limit = future_to_params[future]
            try:
                time_taken, optimality = future.result()
                if time_taken is not None and optimality is not None:
                    results[scene][algorithm][time_limit].append((time_taken, optimality))
            except Exception as exc:
                print(f'Generated an exception: {exc}', file=sys.stderr)

    return results


def analyzeResults(results):
    analysis = {scene: {algorithm: {time_limit: {'avg_time': 0, 'avg_optimality': 0, 'success_rate': 0} for time_limit in timeLimits} for algorithm in algorithms} for scene in scenes}
    for scene in scenes:
        for algorithm in algorithms:
            for timeLimit in timeLimits:
                times = [result[0] for result in results[scene][algorithm][timeLimit]]
                optimalities = [result[1] for result in results[scene][algorithm][timeLimit]]
                if times and optimalities:
                    avgTime = np.mean(times)
                    avgOptimality = np.mean(optimalities)
                    successRate = len(times) / repetitions
                    analysis[scene][algorithm][timeLimit] = {'avg_time': avgTime, 'avg_optimality': avgOptimality, 'success_rate': successRate}
    return analysis


# Plot the results
def plot(analysis):
    for scene in scenes:
        plt.figure(figsize=(12, 6))
        for idx, algorithm in enumerate(algorithms):
            # style = line_styles[idx % len(line_styles)]
            # marker = markers[idx % len(markers)]
            times = timeLimits
            optimalities = [analysis[scene][algorithm][f'{time_limit}']['avg_optimality'] for time_limit in timeLimits]
            plt.plot(times, optimalities, label=algorithm)
        plt.xlabel('Time Limit (s)')
        plt.ylabel('Average Optimality')
        plt.title(f'Optimality Path over Time for {scene}')
        plt.legend()
        plt.ylim(bottom=0)
        plt.xlim(left=1)
        plt.savefig(f'optimality_{scene}.png')
        plt.clf()

    for scene in scenes:
        plt.figure(figsize=(12, 6))
        for idx, algorithm in enumerate(algorithms):
            successRate = np.mean([analysis[scene][algorithm][f'{timeLimit}']['success_rate'] for timeLimit in timeLimits])
            plt.bar(algorithm, successRate, label=algorithm, color=colors[idx])
        plt.xlabel('Algorithm')
        plt.ylabel('Success Rate')
        plt.title('Success Rate of Each Algorithm Over Each Scene')
        plt.legend()
        plt.ylim(bottom=0, top=1)
        plt.savefig(f'success_rate_{scene}.png')
        plt.clf()



def main():
    results = runExperiment()
    analysis = analyzeResults(results)
    plot(analysis)
    print(analysis)

if __name__ == "__main__":
    rc = 1
    try:
        main()
        rc = 0
    except Exception as e:
        print('Error: %s' % e, file=sys.stderr)
    sys.exit(rc)