from copy import deepcopy
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
    FPCMCI_g = 'fpcmci_g'
    FPCMCI_k = 'fpcmci_k'
    PCMCI = 'pcmci'
    
class Metric(Enum):
    TIME = 'time'
    SHD = "shd"

plotLabel = {
    Metric.TIME : 'Time [s]',
    Metric.SHD : 'SHD',
    Algo.FPCMCI_g : 'F-PCMCI gaussian',
    Algo.FPCMCI_k : 'F-PCMCI',
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
    
    for n in nvars:
        res_path = os.getcwd() + resfolder + "/" + str(n) + ".json"
        
        ext_data = extract_data(res_path, algorithms, metric)
        for algo in algorithms:
            toPlot[algo.value]["samples"].append(ext_data[algo.value]["samples"])
            toPlot[algo.value]["means"].append(ext_data[algo.value]["mean"])
            toPlot[algo.value]["confidences"].append(ext_data[algo.value]["confidence"])
           
    fig, ax = plt.subplots(figsize=(6,4))

    for algo in algorithms:
        plt.errorbar(nvars, toPlot[algo.value]["means"], toPlot[algo.value]["confidences"], 
                     marker=plotStyle[algo]['marker'], capsize = 5, color = plotStyle[algo]['color'], linestyle = plotStyle[algo]['linestyle'])
            
    plt.xticks(nvars)
    plt.xlabel("Sampling frequency [Hz]")
    plt.ylabel(plotLabel[metric])
    bbox_to_anchor = (0, 1.05, 1, .105)
    plt.legend([plotLabel[algo] for algo in algorithms])
    # plt.legend([plotLabel[algo] for algo in algorithms], loc=9, bbox_to_anchor=bbox_to_anchor, ncol=2, mode='expand',)
    plt.grid()
    # plt.title(titleLabel[metric] + ' comparison')
        
    if show:
        plt.show()
    else:
        plt.savefig(os.getcwd() + resfolder + "/" + metric.value + '.pdf')
        plt.savefig(os.getcwd() + resfolder + "/" + metric.value + '.png')
        
        
def plot_allagents(resfolder, algorithms, metric, nvars, show = False):
    agents = ["A" + str(i) for i in range(1, 16)]
    dict_res = {algo.value: list() for algo in algorithms}
    toPlot = {a: deepcopy(dict_res) for a in agents}
    since = datetime.datetime(1900, 1, 1, 0, 0, 0, 0)

    for n in nvars:
        res_path = os.getcwd() + resfolder + "/" + str(n) + ".json"
        with open(res_path) as json_file:
            r = json.load(json_file)
        
        for a in agents:
            if metric == Metric.TIME:
                for algo in algorithms:
                    t = datetime.datetime.strptime(r[a][algo.value][metric.value], '%H:%M:%S.%f')
                    toPlot[a][algo.value].append((t - since).total_seconds())
            else:
                for algo in algorithms:
                    toPlot[a][algo.value].append((r[a][algo.value][metric.value]))
                        
    for a in agents:
        plt.figure()            
        for algo in algorithms:
            plt.plot(nvars, toPlot[a][algo.value], label=algo.value)
            
        plt.xticks(nvars)
        plt.xlabel("Time horizon")
        plt.ylabel(plotLabel[metric])
        bbox_to_anchor = (0, 1.05, 1, .105)
        plt.ylim(0,max(toPlot[a][Algo.PCMCI.value] + toPlot[a][Algo.FPCMCI_k.value])+1)
        # plt.ylim(0,max(toPlot[a][Algo.PCMCI.value] + toPlot[a][Algo.FPCMCI_g.value] + toPlot[a][Algo.FPCMCI_k.value])+1)
        plt.legend()
        plt.grid()
        plt.title(a)
            
        if show:
            plt.show()
        else:
            # plt.savefig(os.getcwd() + resfolder + "/" + a + '.pdf')
            plt.savefig(os.getcwd() + resfolder + "/" + a + '.png')
    
if __name__ == '__main__':   

    resfolder = ['/results/ROMAN2024/Evaluation_Hz']
    # vars = [x / 10 for x in range(1, 11)]
    vars = ['0.5', '1', '2', '3.33', '10']

    
    bootstrap = True
    algorithms = [Algo.PCMCI, Algo.FPCMCI_k]
    plot_style = {
                  Algo.PCMCI: {"marker" : 'x', "color" : 'g', "linestyle" : ':'},
                  Algo.FPCMCI_g: {"marker" : '^', "color" : 'r', "linestyle" : '--'}, 
                  Algo.FPCMCI_k: {"marker" : 'o', "color" : 'b', "linestyle" : '-'}, 
                 }
    
    # plot_allagents(resfolder[0], algorithms, Metric.SHD, vars, show=False)

    for r in resfolder:
        for metric in [Metric.TIME, Metric.SHD]:
            compare(r, algorithms, metric, vars, plot_style)
