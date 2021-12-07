# Safe-and-Sample-efficient-Reinforcement-Learning-for-Clustered-Dynamic-Uncertain-Environments

## Table of Contents
- [Install](#install)
- [Usage](#usage)

## Install

```
conda create -n safe-rl
conda install python=3.7.9
pip install tensorflow==2.2.1
pip install keras
pip install matplotlib
pip install gym
pip install cvxopt
```

or

```
conda create -n safe-rl python=3.7.9 -y
conda activate safe-rl
pip install -r requirements.txt
```

## Usage

```
python train.py --display {none, pyglet} --explore rnd --no-qp --mode safe
```
- `--display` can be either `none` or `pyglet` (visulization).
- `--explore` specifies the exploration strategy that the robot uses. 
- `--no-qp` means that we use vanilla SSA.
- `--mode` means that we use which approach: safe set algorithm to guarantee safety

