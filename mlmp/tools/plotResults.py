import matplotlib.pyplot as plt
import numpy as np
import json
import sys


colors = ['blue', 'orange', 'green', 'red', 'purple']

def plotResults(analysis, scenes, timeLimits, algorithms):
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



def main(scenes: list, timeLimits: list, algorithms: list):
    results = {}
    with open('./results.json', 'r') as f:
        results = json.load(f)
    plotResults(results, scenes, timeLimits, algorithms)



if __name__ == "__main__":
    rc = 1
    try:
        main(scenes=['scene1.json', 'scene2.json', 'scene3.json', 'scene4.json', 'scene5.json'], 
             algorithms = ['BiQRRT', 'QRRT', 'QRRTStar', 'QMP', 'QMPStar'], 
             timeLimits = [1, 10, 30, 60, 120])
        rc = 0
    except Exception as e:
        print('Error: %s' % e, file=sys.stderr)
    sys.exit(rc)