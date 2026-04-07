# PID Tuning

## Overview

This repository contains the implementation of a PID tuning procedure
based on classical methods. The goal is to estimate the
controller parameters (Kp, Ti, Td) from experimental data obtained from
a process step response.

The code was developed as part of a scientific study focused on control
of gas injection systems and is intended to support reproducibility of
the results presented in the associated article.

## Repository Structure

-   `tuningPID.py` or notebook: Main script containing the
    implementation of the tuning algorithms.

-   `data/` (Excel or CSV files): Experimental data used for system
    identification and controller tuning.

-   `results/`: Output plots and computed PID parameters.

## Requirements

The code was developed using Python 3.x. The following libraries are
required:

-   numpy
-   pandas
-   matplotlib

Install dependencies using:

    pip install numpy pandas matplotlib

## How to Run

1.  Place your experimental data (step response) in the appropriate
    folder.
2.  Update the file path in the script if necessary.
3.  Run the script:

    jupyter notebook Otimization_controlparameters.ipynb

## Methodology

The implemented tuning procedure is based on Ziegler-Nichols methods,
which rely on extracting key parameters from the process response:

-   Process gain
-   Dead time
-   Time constant

From these values, the PID parameters are computed according to standard
tuning rules.

## Input Data

The input data should contain:

-   Time vector
-   Process output (e.g., pressure)

The data must represent a step response of the system.

## Output

The script provides:

-   Estimated PID parameters (Kp, Ti, Td)
-   Plots of the process response
-   Optional validation plots

## Notes

-   The quality of the tuning depends on the quality of the experimental
    data.
-   Noise in the measurements may affect parameter estimation.
-   It is recommended to preprocess the data if necessary.

## Citation

If you use this code, please cite the associated article.

## License

This project is intended for academic use. Please check the journal
requirements regarding code sharing and licensing.
