import datetime
from enum import Enum
import json
import os
from matplotlib import pyplot as plt
import numpy as np

class jWord(Enum):
    SCM = 'scm'
    GT = "gt"
    
class Algo(Enum):
    FPCMCI = 'fpcmci'
    PCMCI = 'pcmci'
    
class Metric(Enum):
    TIME = 'time'
    SHD = "shd"

plotLabel = {
    Metric.TIME : 'Time [s]',
    Metric.SHD : 'SHD',
    Algo.FPCMCI : 'F-PCMCI',
    Algo.PCMCI : 'PCMCI',
    }

titleLabel = {
    Metric.TIME : 'Time',
    Metric.SHD : 'SHD',
    }
   
    
def extract_data(file_path, algorithms, metric):
    ext_data = {algo.value: {"samples" : list(), "mean" : float, "confidence" : float} for algo in algorithms}

    since = datetime.datetime(1900, 1, 1, 0, 0, 0, 0)
    with open(file_path) as json_file:
        r = json.load(json_file)
        
        for i in r.keys():
            if metric == Metric.TIME:
                for algo in algorithms:
                    t = datetime.datetime.strptime(r[i][algo.value][metric.value], '%H:%M:%S.%f')
                    ext_data[algo.value]["samples"].append((t - since).total_seconds())
            else:
                for algo in algorithms:
                    ext_data[algo.value]["samples"].append((r[i][algo.value][metric.value]))
                
    for algo in algorithms:
        ext_data[algo.value]["mean"] = np.mean(ext_data[algo.value]["samples"])
        ext_data[algo.value]["confidence"] = np.std(ext_data[algo.value]["samples"])
    
    return ext_data
           
    
def compare(resfolder, algorithms, metric, nvars, plotStyle, show = False):
   
    toPlot = {algo.value: {"samples" : list(), "means" : list(), "confidences" : list()} for algo in algorithms}
    
    for n in range(nvars[0],nvars[1]+1):
        res_path = os.getcwd() + "/results/" + resfolder + "/" + str(n) + ".json"
        
        ext_data = extract_data(res_path, algorithms, metric)
        for algo in algorithms:
            toPlot[algo.value]["samples"].append(ext_data[algo.value]["samples"])
            toPlot[algo.value]["means"].append(ext_data[algo.value]["mean"])
            toPlot[algo.value]["confidences"].append(ext_data[algo.value]["confidence"])
           
    fig, ax = plt.subplots(figsize=(6,4))

    for algo in algorithms:
        plt.errorbar(range(nvars[0], nvars[1]+1), toPlot[algo.value]["means"], toPlot[algo.value]["confidences"], 
                     marker=plotStyle[algo]['marker'], capsize = 5, color = plotStyle[algo]['color'], linestyle = plotStyle[algo]['linestyle'])
            
    plt.xticks(range(nvars[0], nvars[1]+1))
    plt.xlabel("Percentage of samples")
    plt.ylabel(plotLabel[metric])
    bbox_to_anchor = (0, 1.05, 1, .105)
    plt.legend([plotLabel[algo] for algo in algorithms], loc=9, bbox_to_anchor=bbox_to_anchor, ncol=7, mode='expand',)
    plt.grid()
    # plt.title(titleLabel[metric] + ' comparison')
        
    if show:
        plt.show()
    else:
        plt.savefig(os.getcwd() + "/results/" + resfolder + "/" + metric.value + '.pdf')
        plt.savefig(os.getcwd() + "/results/" + resfolder + "/" + metric.value + '.png')

    
if __name__ == '__main__':   

    resfolder = ['rebuttal/nconfounded_nonlin_1250_250']
    vars = [0, 7]
    
    bootstrap = True
    algorithms = [Algo.PCMCI, Algo.FPCMCI]
    plot_style = {Algo.PCMCI: {"marker" : 'x', "color" : 'g', "linestyle" : ':'},
                  Algo.FPCMCI: {"marker" : '^', "color" : 'r', "linestyle" : '--'}, 
                  }

    for r in resfolder:
        for metric in [Metric.TIME, Metric.SHD]:
            compare(r, algorithms, metric, vars, plot_style)
