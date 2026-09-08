# Causal analysis

Dataset and estimation code behind Chapter 5 of the thesis *Causal Effect Estimation of
Robot Actions for Human Aware Navigation* (F. Baldo, University of Padua, 2026) and
Section 5 of the AIRO 2026 paper *Estimating the Causal Impact of Social Navigation
Signals in Corridor Scenarios*.

## Contents

| file | what it is |
|---|---|
| `episodes_100_v1.csv` | 100 observational episodes, one row each, six binary DAG variables |
| `hrisim_causal_analysis.ipynb` | reproduces every estimate reported in the thesis and regenerates Figure 5.1 |

## The dataset

Each row is one corridor traversal, either `WP_SPAWN → WP_TABLE` or the return leg. The
simulation that produced it is in this repository; the recording and extraction pipeline
is described in Chapter 4 of the thesis.

| column | meaning | 1 | 0 |
|---|---|---|---|
| `episode_id` | traversal index, in collection order | | |
| `Pi` | pedestrian occupancy at decision time | congested | open |
| `A` | robot action (LED signal) | signal emitted | no signal |
| `Pe` | pedestrian occupancy after the response window | congested | open |
| `S` | geometric clearance available for the transit | sufficient | insufficient |
| `T` | task outcome | success | timeout |
| `O` | confounder: static obstacles in the junction | present | absent |

`Pi` and `Pe` are computed over pedestrians alone against the perceptual threshold
*G*<sub>soc</sub> = 1.15 m; `S` is computed over walls, pedestrians and obstacles together
against the stricter physical threshold *G*<sub>phys</sub> = 0.80 m. All 100 episodes are
complete: no missing values, no discarded runs.

The per-timestep extraction from which this file is aggregated (0.1 s resolution, robot
and agent poses, obstacle positions) is not included here.

## Running the notebook

```bash
pip install pandas numpy scipy matplotlib
jupyter notebook hrisim_causal_analysis.ipynb
```

`pyagrum` and `causal-learn` are optional. Every estimate reported in the thesis is
computed in plain pandas, so the notebook runs end to end without them; the cells that
use them — the constrained hill-climbing step and the cross-check of the propagated
estimates — are guarded and skip with a message if the import fails.

```bash
pip install pyagrum causal-learn      # optional, for the full run
```

## What it reproduces

| estimate | value |
|---|---|
| naive, unadjusted | −0.207 |
| backdoor adjustment on `{O}`, observed frequencies | +0.061 |
| same effect propagated along the chain DAG | +0.026 |
| backdoor on `{Pi, O}`, overlap cells only (64 episodes) | +0.108 |
| same, propagated along the chain with `Pi → Pe` | +0.036 |
| `{Pi, O}` with the empty stratum filled by the prior | −0.123 (artefact) |
| estimate on the DAG learned by hill-climbing | +0.040 (needs `pyagrum`) |

Every estimate computed on observed data alone is positive. The naive figure is
confounded and sign-inverted, a Simpson's paradox driven by the static obstacles. The one
other negative value comes from smoothing over the stratum `Pi=0, O=0, A=1`, which holds
36 of the 100 episodes and contains no episode with `A = 1`: the policy never signals when
the corridor is perceived as clear, so that counterfactual is not estimable from these
data.

The notebook asserts the marginals against the values reported in the thesis and fails
loudly on any mismatch.
